/**
 * @file mainGPS.c
 * @brief Prueba de comunicación UART entre ESP32-S3 y receptor GPS NEO-6M.
 *
 * OBJETIVO
 * --------
 * Este programa permite verificar que el ESP32-S3 recibe correctamente las
 * sentencias NMEA transmitidas por un módulo u-blox NEO-6M.
 *
 * No realiza todavía el análisis completo de posición, velocidad o satélites.
 * Su función es:
 *
 *   1. Inicializar UART1.
 *   2. Probar automáticamente varias velocidades de comunicación.
 *   3. Detectar sentencias NMEA válidas que comiencen por '$'.
 *   4. Mostrar cada sentencia recibida por el monitor serie.
 *   5. Mantener estadísticas básicas de bytes, líneas y errores.
 *
 * CONEXIONES
 * ----------
 *
 *     ESP32-S3                         NEO-6M
 *     --------                         -------
 *
 *     GPIO45  UART1 RX   <------------ TX
 *     GPIO47  UART1 TX   ------------> RX
 *     3V3 o 5V            ------------> VCC
 *     GND                 ------------> GND
 *
 * IMPORTANTE:
 *   - TX y RX se conectan cruzados.
 *   - Debe existir una masa común entre ESP32-S3 y GPS.
 *   - El GPS transmite sentencias NMEA incluso aunque no tenga FIX.
 *
 * CONFIGURACIÓN UART
 * ------------------
 *
 *     UART       : UART_NUM_1
 *     TX ESP32   : GPIO47
 *     RX ESP32   : GPIO45
 *     Formato    : 8N1
 *     Baud rates : 9600, 38400, 19200, 57600, 115200 y 4800
 *
 * El NEO-6M suele venir configurado de fábrica a 9600 baudios.
 */

/*
 * ============================================================================
 * RESUMEN DE SENTENCIAS NMEA RECIBIDAS
 * ============================================================================
 *
 * +---------+------------------------------------------------+----------------+
 * | MENSAJE | INFORMACIÓN PRINCIPAL                          | USO EN EL EFIS  |
 * +---------+------------------------------------------------+----------------+
 * | GGA     | Hora UTC, latitud, longitud, calidad del FIX,  | Imprescindible  |
 * |         | satélites usados, HDOP y altitud.              |                |
 * +---------+------------------------------------------------+----------------+
 * | RMC     | Estado de navegación, posición, velocidad      | Imprescindible  |
 * |         | sobre el suelo, rumbo sobre el suelo y fecha.  |                |
 * +---------+------------------------------------------------+----------------+
 * | GSA     | Tipo de FIX 1D/2D/3D, satélites utilizados,    | Diagnóstico     |
 * |         | PDOP, HDOP y VDOP.                             | opcional        |
 * +---------+------------------------------------------------+----------------+
 * | BDGSA   | Igual que GSA, pero referido a satélites       | Prescindible si |
 * |         | BeiDou.                                        | no se separan   |
 * |         |                                                | constelaciones  |
 * +---------+------------------------------------------------+----------------+
 * | GSV     | Satélites GPS visibles: identificador PRN,     | Diagnóstico o   |
 * |         | elevación, azimut y relación señal/ruido SNR.  | pantalla SAT    |
 * |         | Puede ocupar varias sentencias consecutivas.   |                |
 * +---------+------------------------------------------------+----------------+
 * | BDGSV   | Igual que GSV, pero para satélites BeiDou.     | Diagnóstico     |
 * |         |                                                | opcional        |
 * +---------+------------------------------------------------+----------------+
 * | ZDA     | Hora UTC, día, mes y año.                      | Redundante si   |
 * |         |                                                | se usa RMC      |
 * +---------+------------------------------------------------+----------------+
 *
 * IDENTIFICADORES DE CONSTELACIÓN / TALKER ID
 * -------------------------------------------
 *
 *   GP  -> GPS
 *   BD  -> BeiDou
 *   GL  -> GLONASS
 *   GA  -> Galileo
 *   GN  -> Solución combinada de varias constelaciones GNSS
 *
 * EJEMPLOS:
 *
 *   $GNGGA  -> GGA con solución GNSS combinada.
 *   $GNRMC  -> RMC con solución GNSS combinada.
 *   $GPGSV  -> Información de satélites GPS visibles.
 *   $BDGSV  -> Información de satélites BeiDou visibles.
 *
 * CONFIGURACIÓN PREVISTA PARA EL EFIS
 * -----------------------------------
 *
 * Mantener:
 *
 *   GGA -> posición, altitud, FIX, satélites y HDOP.
 *   RMC -> velocidad, ground track, fecha y hora.
 *
 * Opcionales:
 *
 *   GSA -> diagnóstico de geometría y tipo de FIX.
 *   GSV -> futura pantalla de estado de satélites.
 *
 * Desactivables:
 *
 *   BDGSA, BDGSV y ZDA, si la información no se utiliza explícitamente.
 *
 * ============================================================================
 */

#include <ctype.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define GPS_UART_PORT UART_NUM_1
#define GPS_UART_TX_GPIO 5
#define GPS_UART_RX_GPIO 7
#define GPS_UART_RX_BUFFER_SIZE 2048U
#define GPS_UART_TX_BUFFER_SIZE 256U
#define GPS_LINE_BUFFER_SIZE 256U
#define GPS_READ_BLOCK_SIZE 128U
#define GPS_READ_TIMEOUT_MS 100U
#define GPS_AUTODETECT_TIME_MS 1800U
#define GPS_TASK_STACK_SIZE 4096U
#define GPS_TASK_PRIORITY 5U
#define GPS_ENABLE_TX_TEST 0
#define GPS_TX_TEST_PERIOD_MS 5000U

static void gps_set_baud_115200(void);

static const char *TAG = "GPS_TEST";

typedef struct
{
    bool valid;
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float speed_knots;
    float ground_track_deg;
    float hdop;
    uint8_t satellites;
    uint8_t fix_type;

    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    uint8_t day;
    uint8_t month;
    uint16_t year;
} gps_data_t;

typedef struct
{
    uint32_t baud_rate;
    uint32_t bytes_received;
    uint32_t lines_received;
    uint32_t nmea_lines;
    uint32_t overflow_lines;
    uint32_t invalid_characters;
} gps_statistics_t;

static gps_statistics_t s_statistics = {0};

static const uint32_t s_candidate_baud_rates[] = {
    9600U,
    38400U,
    19200U,
    57600U,
    115200U,
    4800U,
};

static esp_err_t gps_uart_install(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(
        GPS_UART_PORT,
        GPS_UART_RX_BUFFER_SIZE,
        GPS_UART_TX_BUFFER_SIZE,
        0,
        NULL,
        0);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo instalar UART%d: %s",
                 GPS_UART_PORT, esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(GPS_UART_PORT, &uart_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo configurar UART%d: %s",
                 GPS_UART_PORT, esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(
        GPS_UART_PORT,
        GPS_UART_TX_GPIO,
        GPS_UART_RX_GPIO,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudieron asignar los pines UART: %s",
                 esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "UART%d instalada: TX=GPIO%d, RX=GPIO%d, formato 8N1",
             GPS_UART_PORT, GPS_UART_TX_GPIO, GPS_UART_RX_GPIO);

    return ESP_OK;
}

static esp_err_t gps_uart_set_baud_rate(uint32_t baud_rate)
{
    esp_err_t err = uart_set_baudrate(GPS_UART_PORT, baud_rate);

    if (err == ESP_OK)
    {
        uart_flush_input(GPS_UART_PORT);
        s_statistics.baud_rate = baud_rate;
        ESP_LOGI(TAG, "Probando GPS a %" PRIu32 " baudios", baud_rate);
    }
    else
    {
        ESP_LOGE(TAG, "No se pudo seleccionar %" PRIu32 " baudios: %s",
                 baud_rate, esp_err_to_name(err));
    }

    return err;
}

static bool gps_is_printable_nmea_character(uint8_t character)
{
    return ((character >= 0x20U) && (character <= 0x7EU)) ||
           (character == '\r') ||
           (character == '\n');
}

static bool gps_line_looks_like_nmea(const char *line)
{
    if (line == NULL)
        return false;

    const size_t length = strlen(line);
    if (length < 7U)
        return false;

    if (line[0] != '$')
        return false;

    if (!isalpha((unsigned char)line[1]) ||
        !isalpha((unsigned char)line[2]) ||
        !isalpha((unsigned char)line[3]) ||
        !isalpha((unsigned char)line[4]) ||
        !isalpha((unsigned char)line[5]))
    {
        return false;
    }

    return line[6] == ',';
}

static bool gps_nmea_checksum_is_valid(const char *line)
{
    if ((line == NULL) || (line[0] != '$'))
        return false;

    const char *asterisk = strchr(line, '*');
    if (asterisk == NULL)
        return true;

    if ((asterisk[1] == '\0') || (asterisk[2] == '\0'))
        return false;

    uint8_t checksum = 0U;
    for (const char *p = line + 1; p < asterisk; ++p)
        checksum ^= (uint8_t)*p;

    unsigned int received_checksum = 0U;
    if (sscanf(asterisk + 1, "%2x", &received_checksum) != 1)
        return false;

    return checksum == (uint8_t)received_checksum;
}

static bool gps_autodetect_baud_rate(uint32_t *detected_baud_rate)
{
    if (detected_baud_rate == NULL)
        return false;

    uint8_t rx_data[GPS_READ_BLOCK_SIZE];
    char line[GPS_LINE_BUFFER_SIZE];

    for (size_t baud_index = 0;
         baud_index < (sizeof(s_candidate_baud_rates) / sizeof(s_candidate_baud_rates[0]));
         ++baud_index)
    {
        const uint32_t baud_rate = s_candidate_baud_rates[baud_index];

        if (gps_uart_set_baud_rate(baud_rate) != ESP_OK)
            continue;

        size_t line_position = 0U;
        const TickType_t start_tick = xTaskGetTickCount();
        const TickType_t test_duration = pdMS_TO_TICKS(GPS_AUTODETECT_TIME_MS);

        while ((xTaskGetTickCount() - start_tick) < test_duration)
        {
            const int received = uart_read_bytes(
                GPS_UART_PORT,
                rx_data,
                sizeof(rx_data),
                pdMS_TO_TICKS(GPS_READ_TIMEOUT_MS));

            if (received <= 0)
                continue;

            for (int i = 0; i < received; ++i)
            {
                const uint8_t character = rx_data[i];

                if (!gps_is_printable_nmea_character(character))
                {
                    line_position = 0U;
                    continue;
                }

                if ((character == '\r') || (character == '\n'))
                {
                    if (line_position > 0U)
                    {
                        line[line_position] = '\0';

                        if (gps_line_looks_like_nmea(line))
                        {
                            *detected_baud_rate = baud_rate;
                            ESP_LOGI(TAG,
                                     "GPS detectado a %" PRIu32 " baudios. Primera sentencia: %s",
                                     baud_rate, line);
                            uart_flush_input(GPS_UART_PORT);
                            return true;
                        }

                        line_position = 0U;
                    }

                    continue;
                }

                if (character == '$')
                    line_position = 0U;

                if (line_position < (sizeof(line) - 1U))
                    line[line_position++] = (char)character;
                else
                    line_position = 0U;
            }
        }

        ESP_LOGW(TAG, "No se reconocieron sentencias NMEA a %" PRIu32 " baudios",
                 baud_rate);
    }

    return false;
}

static void gps_print_statistics(void)
{
    ESP_LOGI(TAG,
             "ESTADISTICAS: baud=%" PRIu32
             ", bytes=%" PRIu32
             ", lineas=%" PRIu32
             ", NMEA=%" PRIu32
             ", overflow=%" PRIu32
             ", caracteres_invalidos=%" PRIu32,
             s_statistics.baud_rate,
             s_statistics.bytes_received,
             s_statistics.lines_received,
             s_statistics.nmea_lines,
             s_statistics.overflow_lines,
             s_statistics.invalid_characters);
}

static void gps_process_line(const char *line)
{
    if ((line == NULL) || (line[0] == '\0'))
        return;

    ++s_statistics.lines_received;

    if (!gps_line_looks_like_nmea(line))
    {
        ESP_LOGW(TAG, "Línea no reconocida: %s", line);
        return;
    }

    ++s_statistics.nmea_lines;

    if (gps_nmea_checksum_is_valid(line))
        ESP_LOGI(TAG, "NMEA OK: %s", line);
    else
        ESP_LOGW(TAG, "NMEA con checksum incorrecto: %s", line);
}

static void gps_receive_task(void *argument)
{
    (void)argument;

    uint8_t rx_data[GPS_READ_BLOCK_SIZE];
    char line[GPS_LINE_BUFFER_SIZE];
    size_t line_position = 0U;
    TickType_t last_statistics_tick = xTaskGetTickCount();

    ESP_LOGI(TAG, "Tarea de recepción GPS iniciada");

    for (;;)
    {
        const int received = uart_read_bytes(
            GPS_UART_PORT,
            rx_data,
            sizeof(rx_data),
            pdMS_TO_TICKS(GPS_READ_TIMEOUT_MS));

        if (received > 0)
        {
            s_statistics.bytes_received += (uint32_t)received;

            for (int i = 0; i < received; ++i)
            {
                const uint8_t character = rx_data[i];

                if (!gps_is_printable_nmea_character(character))
                {
                    ++s_statistics.invalid_characters;
                    continue;
                }

                if ((character == '\r') || (character == '\n'))
                {
                    if (line_position > 0U)
                    {
                        line[line_position] = '\0';
                        gps_process_line(line);
                        line_position = 0U;
                    }
                    continue;
                }

                if (character == '$')
                    line_position = 0U;

                if (line_position < (sizeof(line) - 1U))
                {
                    line[line_position++] = (char)character;
                }
                else
                {
                    ++s_statistics.overflow_lines;
                    ESP_LOGW(TAG, "Sentencia demasiado larga; se descarta");
                    line_position = 0U;
                }
            }
        }

        const TickType_t now = xTaskGetTickCount();
        if ((now - last_statistics_tick) >= pdMS_TO_TICKS(10000U))
        {
            gps_print_statistics();
            last_statistics_tick = now;
        }
    }
}

#if GPS_ENABLE_TX_TEST
static void gps_tx_test_task(void *argument)
{
    (void)argument;

    static const char test_message[] =
        "$GPTXT,01,01,02,ESP32-S3 UART TEST*00\r\n";

    for (;;)
    {
        const int sent = uart_write_bytes(
            GPS_UART_PORT,
            test_message,
            sizeof(test_message) - 1U);

        ESP_LOGI(TAG, "Prueba TX: enviados %d bytes por GPIO%d",
                 sent, GPS_UART_TX_GPIO);

        vTaskDelay(pdMS_TO_TICKS(GPS_TX_TEST_PERIOD_MS));
    }
}
#endif

void app_main(void)
{
    ESP_LOGI(TAG, "Prueba UART ESP32-S3 <-> NEO-6M");
    ESP_LOGI(TAG, "Cableado: GPS TX -> GPIO%d; GPS RX <- GPIO%d",
             GPS_UART_RX_GPIO, GPS_UART_TX_GPIO);

    ESP_ERROR_CHECK(gps_uart_install());

    uint32_t detected_baud_rate = 0U;

    if (gps_autodetect_baud_rate(&detected_baud_rate))
    {
        ESP_LOGI(TAG, "GPS detectado a %lu baudios", detected_baud_rate);

        // gps_set_baud_115200();

        vTaskDelay(pdMS_TO_TICKS(200));

        // uart_set_baudrate(GPS_UART_PORT, 115200);

        ESP_LOGI(TAG, "UART cambiada a 115200");
    }

    BaseType_t task_result = xTaskCreate(
        gps_receive_task,
        "gps_receive",
        GPS_TASK_STACK_SIZE,
        NULL,
        GPS_TASK_PRIORITY,
        NULL);

    if (task_result != pdPASS)
    {
        ESP_LOGE(TAG, "No se pudo crear la tarea de recepción GPS");
        abort();
    }

#if GPS_ENABLE_TX_TEST
    task_result = xTaskCreate(
        gps_tx_test_task,
        "gps_tx_test",
        2048,
        NULL,
        4,
        NULL);

    if (task_result != pdPASS)
    {
        ESP_LOGE(TAG, "No se pudo crear la tarea de prueba TX");
        abort();
    }
#endif

    ESP_LOGI(TAG, "app_main finalizada; las tareas GPS continúan ejecutándose");
}

//---------------------------------------------------------------------------
static void gps_set_baud_115200(void)
{
    static const uint8_t ubx_cfg_prt[] =
        {
            0xB5, 0x62,
            0x06, 0x00,
            0x14, 0x00,

            0x01, // UART1
            0x00,
            0x00, 0x00,
            0xD0, 0x08, 0x00, 0x00,
            0x00, 0xC2, 0x01, 0x00, // 115200 baudios
            0x07, 0x00,
            0x03, 0x00,
            0x00, 0x00,
            0x00, 0x00,

            0xC0, 0x7E};

    uart_write_bytes(
        GPS_UART_PORT,
        (const char *)ubx_cfg_prt,
        sizeof(ubx_cfg_prt));

    uart_wait_tx_done(
        GPS_UART_PORT,
        pdMS_TO_TICKS(100));
}
