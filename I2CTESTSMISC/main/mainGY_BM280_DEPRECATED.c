/******************************************************************************
 * BMP280 / BME280 Altimeter Utilities
 *
 * Este módulo proporciona funciones para convertir medidas de presión
 * atmosférica obtenidas mediante un sensor BMP280/BME280 en altitud
 * barométrica respecto al nivel medio del mar (MSL), utilizando como
 * referencia el valor de QNH introducido por el usuario.
 *
 * Funcionalidades:
 *   - Cálculo de altitud sobre el nivel del mar a partir de la presión
 *     medida y el QNH de referencia.
 *   - Cálculo de la presión atmosférica teórica correspondiente a una
 *     determinada altitud y QNH.
 *   - Cálculo de diferencias de altitud entre dos medidas de presión.
 *
 * Fundamento:
 *   Los cálculos se basan en la atmósfera estándar internacional (ISA)
 *   para la troposfera y emplean la ecuación barométrica simplificada:
 *
 *       h = 44330 · [1 - (P / QNH)^(1 / 5.255)]
 *
 *   donde:
 *       h   = altitud sobre el nivel del mar [m]
 *       P   = presión absoluta medida por el sensor [hPa]
 *       QNH = presión reducida al nivel del mar [hPa]
 *
 * Aplicaciones:
 *   - Altímetros barométricos.
 *   - Estimación de altura en proyectos con ESP32.
 *   - Medición de desnivel y cambios de altitud.
 *   - Instrumentación aeronáutica y meteorológica educativa.
 *
 * Nota:
 *   La precisión final depende de la exactitud del valor de QNH empleado,
 *   de la calibración del sensor y de las desviaciones respecto a la
 *   atmósfera estándar ISA.
 *
 * Autor: Javier Burgoa + GPT
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_check.h"
#include <math.h>

#define I2C_PORT I2C_NUM_0
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_FREQ_HZ 100000

#define BMP280_ADDR 0x76 // prueba 0x77 si no responde
#define REG_CHIP_ID 0xD0
#define REG_RESET 0xE0
#define REG_CTRL_MEAS 0xF4
#define REG_CONFIG 0xF5
#define REG_PRESS_MSB 0xF7
#define REG_CALIB 0x88

static const char *TAG = "BMP280";

static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t t_fine;

static esp_err_t i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    return i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

static esp_err_t bmp280_write8(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_write_to_device(
        I2C_PORT, BMP280_ADDR, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

static esp_err_t bmp280_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        I2C_PORT, BMP280_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

static uint16_t u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t s16_le(const uint8_t *p)
{
    return (int16_t)u16_le(p);
}

static esp_err_t bmp280_read_calibration(void)
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

static int32_t bmp280_compensate_T(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) *
            ((int32_t)dig_T2)) >>
           11;

    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
              ((adc_T >> 4) - ((int32_t)dig_T1))) >>
             12) *
            ((int32_t)dig_T3)) >>
           14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;

    return T; // centésimas de ºC
}

static uint32_t bmp280_compensate_P(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);

    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
           ((var1 * (int64_t)dig_P2) << 12);

    var1 = (((((int64_t)1) << 47) + var1)) *
               ((int64_t)dig_P1) >>
           33;

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

static esp_err_t bmp280_init(void)
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

    // standby 500 ms, filtro x4
    ESP_ERROR_CHECK(bmp280_write8(REG_CONFIG, 0b10010000));

    // temperatura x1, presión x1, modo normal
    ESP_ERROR_CHECK(bmp280_write8(REG_CTRL_MEAS, 0b00100111));

    return ESP_OK;
}

static esp_err_t bmp280_read_measurement(float *temp_c, float *press_hpa)
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

/******************************************************************************
 * Calculate altitude from atmospheric pressure using the complete barometric
 * equation with temperature correction.
 *
 * Unlike the simplified ISA equation (44330 m constant), this formulation
 * replaces the standard sea-level temperature (288.15 K) with the measured
 * ambient temperature. This partially compensates for deviations from the
 * International Standard Atmosphere (ISA) and generally provides a better
 * estimate of the geometric altitude when the actual atmospheric temperature
 * differs significantly from ISA conditions.
 *
 * NOTE:
 * - The ideal temperature is the mean temperature of the air column between
 *   the reference level (QNH) and the measurement point.
 * - Using the BMP280 temperature is an approximation and may introduce errors
 *   if the sensor is heated by the PCB, enclosure or direct sunlight.
 * - For aviation altimeters, the standard ISA equation should be preferred,
 *   since aircraft altimeters are calibrated to ISA and use only the QNH
 *   setting.
 *
 * Inputs:
 *   pressure_hpa     Measured atmospheric pressure [hPa]
 *   qnh_hpa          Sea-level pressure (QNH) [hPa]
 *   temperature_C    Measured ambient temperature [°C]
 *
 * Returns:
 *   Estimated altitude above mean sea level [m]
 ******************************************************************************/

//  SIMPLIFICADO
float bmp280_altitude_m(float pressure_hpa, float qnh_hpa)
{
    return 44330.0f *
           (1.0f - powf(pressure_hpa / qnh_hpa, 1.0f / 5.255f));
}

/*
//Con corrección por la temperatura
float bmp280_altitude_temp_m(float pressure_hpa,
                             float qnh_hpa,
                             float temperature_C)
{
    const float L = 0.0065f;                   // ISA lapse rate [K/m]
    const float exponent = 1.0f / 5.2558797f;  // gM/RL
    float temperature_K = temperature_C + 273.15f;

    return (temperature_K / L) *  (1.0f - powf(pressure_hpa / qnh_hpa, exponent));
}
*/

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_init());
    ESP_ERROR_CHECK(bmp280_init());

    while (1)
    {
        float temp, pressure;

        if (bmp280_read_measurement(&temp, &pressure) == ESP_OK)
        {
            float altitude = bmp280_altitude_m(pressure, 1015.0f);

            ESP_LOGI(TAG, "T = %.2f ºC | P = %.2f hPa | h = %.1f m ", temp, pressure, altitude);
        }
        else
        {
            ESP_LOGE(TAG, "Error leyendo sensor");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}