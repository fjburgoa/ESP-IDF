/* Osciloscopio didactico, ADC continuo de dos canales (GPIO4 y GPIO5).
 * ESP-IDF 6.1; ESP32-S3-DevKitC-1; UART0 por el puente USB CH343P.
 * Formato de trama (little endian):
 *   0..3 magic "SCP1"; 4..7 secuencia; 8..11 muestras/s POR CANAL;
 *   12..13 numero de pares; 14..15 flags (bit 0 = simulacion);
 *   16..19 FNV-1a de la carga; 20.. pares intercalados uint16 CH1, CH2.
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_adc/adc_continuous.h"
#include "soc/soc_caps.h"

#define SCOPE_UART UART_NUM_0
#define BAUDRATE 921600
#define FS_CHANNEL_HZ 5000U
#define PAIRS_PER_FRAME 250U
#define FRAME_BYTES (20U + 4U * PAIRS_PER_FRAME)
#define ADC_READ_BYTES 1024U
#define ADC_SAMPLES_PER_READ (ADC_READ_BYTES / SOC_ADC_DIGI_RESULT_BYTES)

/* Estaticos para no agotar la pila de app_main. */
static uint8_t frame[FRAME_BYTES];
static uint8_t adc_raw[ADC_READ_BYTES];
static adc_continuous_data_t adc_samples[ADC_SAMPLES_PER_READ];

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
    for (unsigned i = 0; i < 4; ++i)
        p[i] = (uint8_t)(v >> (8U * i));
}

static uint32_t fnv1a(const uint8_t *data, size_t size)
{
    uint32_t hash = 2166136261U;
    for (size_t i = 0; i < size; ++i)
    {
        hash ^= data[i];
        hash *= 16777619U;
    }
    return hash;
}

void app_main(void)
{
    /* La consola IDF no debe compartir UART0 con los paquetes binarios.
     * En menuconfig: Component config > ESP System Settings > Channel for
     * console output = None (o USB Serial/JTAG, si esta disponible).
     * Los mensajes ROM anteriores a app_main pueden salir igualmente.
     */
    esp_log_level_set("*", ESP_LOG_NONE);

    const uart_config_t config = {
        .baud_rate = BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(SCOPE_UART, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(SCOPE_UART, &config));
    ESP_ERROR_CHECK(uart_set_pin(SCOPE_UART, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    adc_unit_t unit4, unit5;
    adc_channel_t channel4, channel5;
    ESP_ERROR_CHECK(adc_continuous_io_to_channel(9, &unit4, &channel4));
    ESP_ERROR_CHECK(adc_continuous_io_to_channel(8, &unit5, &channel5));
    ESP_ERROR_CHECK((unit4 == ADC_UNIT_1 && unit5 == ADC_UNIT_1 && channel4 != channel5)
                        ? ESP_OK
                        : ESP_ERR_INVALID_ARG);

    adc_continuous_handle_t adc = NULL;
    const adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = 8192,
        .conv_frame_size = 256,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc));

    adc_digi_pattern_config_t pattern[2] = {0};
    pattern[0].atten = ADC_ATTEN_DB_12;
    pattern[0].channel = channel4;
    pattern[0].unit = ADC_UNIT_1;
    pattern[0].bit_width = ADC_BITWIDTH_12;
    pattern[1].atten = ADC_ATTEN_DB_12;
    pattern[1].channel = channel5;
    pattern[1].unit = ADC_UNIT_1;
    pattern[1].bit_width = ADC_BITWIDTH_12;
    /* Se envian cuentas ADC sin calibrar; 3,3 V puede saturar la entrada
     * del ADC con esta atenuacion, aunque sea una tension GPIO admisible. */
    const adc_continuous_config_t adc_cfg = {
        .pattern_num = 2,
        .adc_pattern = pattern,
        .sample_freq_hz = 2 * FS_CHANNEL_HZ, /* conversiones totales */
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    ESP_ERROR_CHECK(adc_continuous_config(adc, &adc_cfg));

    memcpy(frame, "SCP1", 4);
    put32(frame + 8, FS_CHANNEL_HZ);
    put16(frame + 12, PAIRS_PER_FRAME);
    put16(frame + 14, 0U); /* bit 0 a cero: adquisicion real */

    uint32_t sequence = 0;
    unsigned pairs = 0;
    bool have_ch1 = false;
    uint16_t first_sample = 0;
    ESP_ERROR_CHECK(adc_continuous_start(adc));
    for (;;)
    {
        uint32_t bytes_read = 0;
        const esp_err_t err = adc_continuous_read(adc, adc_raw, sizeof(adc_raw),
                                                  &bytes_read, 1000);
        if (err == ESP_ERR_TIMEOUT)
            continue;
        if (err == ESP_ERR_INVALID_STATE)
        {
            /* El pool perdio conversiones: descartar el bloque incompleto. */
            have_ch1 = false;
            pairs = 0;
            sequence++;
        }
        else if (err != ESP_OK)
            ESP_ERROR_CHECK(err);

        if (!bytes_read)
            continue;
        uint32_t sample_count = 0;
        ESP_ERROR_CHECK(adc_continuous_parse_data(adc, adc_raw, bytes_read,
                                                  adc_samples, &sample_count));
        for (uint32_t i = 0; i < sample_count; ++i)
        {
            const adc_continuous_data_t *sample = &adc_samples[i];
            if (!sample->valid || sample->unit != ADC_UNIT_1)
            {
                have_ch1 = false;
                continue;
            }
            if (sample->channel == channel4)
            {
                first_sample = (uint16_t)sample->raw_data;
                have_ch1 = true;
            }
            else if (sample->channel == channel5 && have_ch1)
            {
                put16(frame + 20 + 4 * pairs, first_sample);
                put16(frame + 22 + 4 * pairs, (uint16_t)sample->raw_data);
                have_ch1 = false;
                if (++pairs == PAIRS_PER_FRAME)
                {
                    put32(frame + 4, sequence++);
                    put32(frame + 16, fnv1a(frame + 20, FRAME_BYTES - 20));
                    if (uart_write_bytes(SCOPE_UART, frame, FRAME_BYTES) != (int)FRAME_BYTES)
                        sequence++; /* El navegador podra detectar el salto. */
                    pairs = 0;
                }
            }
        }
    }
}
