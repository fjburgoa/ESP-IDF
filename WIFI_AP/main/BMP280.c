#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "BMP280.h"
#include "esp_check.h"
#include <math.h>
#include "websocket.h"

#define I2C_PORT I2C_NUM_0

#define BMP280_ADDR 0x76 // prueba 0x77 si no responde
#define REG_CHIP_ID 0xD0
#define REG_RESET 0xE0
#define REG_CTRL_MEAS 0xF4
#define REG_CONFIG 0xF5
#define REG_PRESS_MSB 0xF7
#define REG_CALIB 0x88

TaskHandle_t xHandleBMP = NULL; // Handler a la tarea

/*
 * Últimas medidas válidas publicadas por el driver.
 * La altitud ya existía en el proyecto; se añade temperature.
 */
float altitude = 0.0f;
float temperature = 0.0f;
float vertical_speed = 0.0f;
float pressure_hpa = 0.0f;

#define BMP280_PERIOD_MS 100U // 10Hz

/*
 * Variómetro electrónico.
 * La derivada de la altitud amplifica mucho el ruido de presión, por lo que
 * se aplica un paso bajo de primer orden sobre la velocidad vertical.
 */
#define VSI_FILTER_TAU_S 1.0f
#define VSI_LIMIT_FPM 4000.0f
#define FPM_TO_MPS (0.3048f / 60.0f)
#define VSI_LIMIT_MPS (VSI_LIMIT_FPM * FPM_TO_MPS)

/* Estado del filtro del variómetro. */
static bool s_vsi_initialized = false;
static float s_previous_altitude_m = 0.0f;
static float s_vertical_speed_filtered_mps = 0.0f;

/*
 * Entrada de altitud de prueba. main.c puede activarla para comprobar el
 * variómetro sin necesidad de mover físicamente el BMP280.
 */
static portMUX_TYPE s_test_altitude_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_test_altitude_enabled = false;
static float s_test_altitude_m = 0.0f;

esp_err_t bmp280_start(void);

static const char *TAG = "BMP280";

static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t t_fine;

/*------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------*/
esp_err_t bmp280_write8(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_write_to_device(I2C_PORT, BMP280_ADDR, buf, sizeof(buf), pdMS_TO_TICKS(100));
}
/*------------------------------------------------------------------------------*/
esp_err_t bmp280_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, BMP280_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(100));
}
/*------------------------------------------------------------------------------*/
uint16_t u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
/*------------------------------------------------------------------------------*/
int16_t s16_le(const uint8_t *p)
{
    return (int16_t)u16_le(p);
}
/*------------------------------------------------------------------------------*/
esp_err_t bmp280_read_calibration(void)
{
    uint8_t c[24];
    ESP_RETURN_ON_ERROR(bmp280_read(REG_CALIB, c, sizeof(c)), TAG, "Error leyendo calibracion");

    dig_T1 = u16_le(&c[0]);
    dig_T2 = s16_le(&c[2]);
    dig_T3 = s16_le(&c[4]);

    dig_P1 = u16_le(&c[6]);
    dig_P2 = s16_le(&c[8]);
    dig_P3 = s16_le(&c[10]);
    dig_P4 = s16_le(&c[12]);
    dig_P5 = s16_le(&c[14]);
    dig_P6 = s16_le(&c[16]);
    dig_P7 = s16_le(&c[18]);
    dig_P8 = s16_le(&c[20]);
    dig_P9 = s16_le(&c[22]);

    return ESP_OK;
}
/*------------------------------------------------------------------------------*/
int32_t bmp280_compensate_T(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;

    return T; // centésimas de ºC
}
/*------------------------------------------------------------------------------*/
uint32_t bmp280_compensate_P(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);

    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);

    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0)
    {
        return 0;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;

    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

    return (uint32_t)p; // Pa * 256
}
/*------------------------------------------------------------------------------*/
esp_err_t bmp280_init(void)
{
    uint8_t id = 0;
    ESP_RETURN_ON_ERROR(bmp280_read(REG_CHIP_ID, &id, 1), TAG, "No responde el BMP280");

    ESP_LOGI(TAG, "Chip ID: 0x%02X", id);

    if (id == 0x58)
    {
        ESP_LOGI(TAG, "Detectado BMP280");
    }
    else if (id == 0x60)
    {
        ESP_LOGI(TAG, "Detectado BME280");
    }
    else
    {
        ESP_LOGW(TAG, "Chip ID inesperado");
    }

    ESP_ERROR_CHECK(bmp280_write8(REG_RESET, 0xB6));
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_ERROR_CHECK(bmp280_read_calibration());

    /*
     * Para obtener velocidad vertical necesitamos una actualización de presión
     * bastante más rápida que los 500 ms originales.
     *
     * REG_CONFIG:
     *   t_sb   = 001 -> 62.5 ms
     *   filter = 010 -> IIR x4
     *   spi3w  = 0
     *   ESP_ERROR_CHECK(bmp280_write8(REG_CONFIG, 0b00101000));
     */
    /*
     * REG_CONFIG:
     *   t_sb   = 001 -> 62.5 ms
     *   filter = 100 -> IIR x16
     *   spi3w  = 0
     */
    ESP_ERROR_CHECK(bmp280_write8(REG_CONFIG, 0b00110000));

    /*
     * REG_CTRL_MEAS:
     *   temperatura x1
     *   presión x4
     *   modo normal
     */
    ESP_ERROR_CHECK(bmp280_write8(REG_CTRL_MEAS, 0b00101111));

    return ESP_OK;
}
/*------------------------------------------------------------------------------*/
esp_err_t bmp280_read_measurement(float *temp_c, float *press_hpa)
{
    uint8_t d[6];

    ESP_RETURN_ON_ERROR(bmp280_read(REG_PRESS_MSB, d, sizeof(d)), TAG, "Error leyendo datos");

    int32_t adc_P = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
    int32_t adc_T = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);

    int32_t T = bmp280_compensate_T(adc_T);
    uint32_t P = bmp280_compensate_P(adc_P);

    *temp_c = T / 100.0f;
    *press_hpa = (P / 256.0f) / 100.0f;

    return ESP_OK;
}
/*------------------------------------------------------------------------------*/
float bmp280_altitude_m(float pressure_hpa, float qnh_hpa)
{
    return 44330.0f * (1.0f - powf(pressure_hpa / qnh_hpa, 1.0f / 5.255f));
}

/*------------------------------------------------------------------------------*/
static float bmp280_vertical_speed_mps(float altitude_m, float dt_s)
{
    if (!isfinite(altitude_m) || !isfinite(dt_s) || (dt_s <= 0.0f))
    {
        return s_vertical_speed_filtered_mps;
    }

    if (!s_vsi_initialized)
    {
        s_previous_altitude_m = altitude_m;
        s_vertical_speed_filtered_mps = 0.0f;
        s_vsi_initialized = true;
        return 0.0f;
    }

    const float raw_vertical_speed_mps = (altitude_m - s_previous_altitude_m) / dt_s;

    s_previous_altitude_m = altitude_m;

    /*
     * Discretización exacta de un paso bajo de primer orden:
     *
     *   y[k] = y[k-1] + alpha (x[k] - y[k-1])
     *   alpha = 1 - exp(-dt/tau)
     */
    float alpha = 1.0f - expf(-dt_s / VSI_FILTER_TAU_S);

    if (!isfinite(alpha) || (alpha < 0.0f))
        alpha = 0.0f;
    else if (alpha > 1.0f)
        alpha = 1.0f;

    s_vertical_speed_filtered_mps += alpha * (raw_vertical_speed_mps - s_vertical_speed_filtered_mps);

    /* El instrumento HTML está graduado hasta +/-VSI_LIMIT_MPS ft/min. */
    if (s_vertical_speed_filtered_mps > VSI_LIMIT_MPS)
        s_vertical_speed_filtered_mps = VSI_LIMIT_MPS;
    else if (s_vertical_speed_filtered_mps < -VSI_LIMIT_MPS)
        s_vertical_speed_filtered_mps = -VSI_LIMIT_MPS;

    return s_vertical_speed_filtered_mps;
}

/*------------------------------------------------------------------------------*/
void bmp280_set_test_altitude(float altitude_m, bool enable)
{
    portENTER_CRITICAL(&s_test_altitude_mux);

    s_test_altitude_m = altitude_m;
    s_test_altitude_enabled = enable;

    portEXIT_CRITICAL(&s_test_altitude_mux);
}

/*------------------------------------------------------------------------------*/
static void bmp280_get_test_altitude(bool *enabled, float *altitude_m)
{
    portENTER_CRITICAL(&s_test_altitude_mux);

    *enabled = s_test_altitude_enabled;
    *altitude_m = s_test_altitude_m;

    portEXIT_CRITICAL(&s_test_altitude_mux);
}

/*------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------*/
void BMP280Task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(BMP280_PERIOD_MS);
    const float dt_s = (float)BMP280_PERIOD_MS / 1000.0f;

    float temp = 0.0f;
    float pressure = 0.0f;

    while (1)
    {
        if (bmp280_read_measurement(&temp, &pressure) == ESP_OK)
        {
            const float qnh = websocket_get_qnh();
            const float barometric_altitude_m = bmp280_altitude_m(pressure, qnh);

            bool test_enabled = false;
            float test_altitude_m = 0.0f;

            bmp280_get_test_altitude(&test_enabled, &test_altitude_m);

            /*
             * En modo normal se utiliza la altitud barométrica.
             * En modo de prueba se utiliza exclusivamente la altitud triangular
             * generada en main.c. El cálculo de velocidad vertical es idéntico
             * en ambos casos.
             */
            const float selected_altitude_m = test_enabled ? test_altitude_m : barometric_altitude_m;

            altitude = selected_altitude_m;
            vertical_speed = bmp280_vertical_speed_mps(selected_altitude_m, dt_s);
            temperature = temp;
            pressure_hpa = pressure;
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

/*------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------*/
esp_err_t bmp280_start(void)
{
    ESP_ERROR_CHECK(bmp280_init());

    int ucParamToPass = 0; // dummy

    // Crear la tarea
    BaseType_t ok = xTaskCreate(BMP280Task, "alt_meas", 4096, &ucParamToPass, 5, &xHandleBMP);

    if (ok != pdPASS)
    {
        xHandleBMP = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "BMP iniciado a %u Hz", 1000U / BMP280_PERIOD_MS);

    return ESP_OK;
}
