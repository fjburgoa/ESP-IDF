#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_check.h"

#include "BM280_v61.h"
#include "EFIS_I2C.h"

static const char *TAG = "BMP280";

static i2c_master_dev_handle_t s_bmp280 = NULL;

static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t t_fine;


/* -------------------------------------------------------------------------- */
/* Acceso I2C                                                                 */
/* -------------------------------------------------------------------------- */

esp_err_t bmp280_write8(uint8_t reg, uint8_t data)
{
    if (s_bmp280 == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[2] = {reg, data};

    return i2c_master_transmit(
        s_bmp280,
        buf,
        sizeof(buf),
        BMP280_I2C_TIMEOUT_MS);
}


esp_err_t bmp280_read(uint8_t reg, uint8_t *data, size_t len)
{
    if ((s_bmp280 == NULL) || (data == NULL) || (len == 0))
    {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit_receive(
        s_bmp280,
        &reg,
        1,
        data,
        len,
        BMP280_I2C_TIMEOUT_MS);
}


/* -------------------------------------------------------------------------- */
/* Conversión de calibración                                                  */
/* -------------------------------------------------------------------------- */

uint16_t u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}


int16_t s16_le(const uint8_t *p)
{
    return (int16_t)u16_le(p);
}


/* -------------------------------------------------------------------------- */
/* Calibración                                                                */
/* -------------------------------------------------------------------------- */

esp_err_t bmp280_read_calibration(void)
{
    uint8_t c[24];

    ESP_RETURN_ON_ERROR(
        bmp280_read(REG_CALIB, c, sizeof(c)),
        TAG,
        "Error leyendo calibracion");

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


/* -------------------------------------------------------------------------- */
/* Compensación Bosch                                                         */
/* -------------------------------------------------------------------------- */

int32_t bmp280_compensate_T(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 =
        ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) *
         ((int32_t)dig_T2)) >>
        11;

    var2 =
        (((((adc_T >> 4) - ((int32_t)dig_T1)) *
           ((adc_T >> 4) - ((int32_t)dig_T1))) >>
          12) *
         ((int32_t)dig_T3)) >>
        14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;

    return T; /* centésimas de ºC */
}


uint32_t bmp280_compensate_P(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);

    var1 =
        ((var1 * var1 * (int64_t)dig_P3) >> 8) +
        ((var1 * (int64_t)dig_P2) << 12);

    var1 =
        (((((int64_t)1) << 47) + var1) *
         ((int64_t)dig_P1)) >>
        33;

    if (var1 == 0)
    {
        return 0;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;

    var1 =
        (((int64_t)dig_P9) *
         (p >> 13) *
         (p >> 13)) >>
        25;

    var2 =
        (((int64_t)dig_P8) * p) >>
        19;

    p =
        ((p + var1 + var2) >> 8) +
        (((int64_t)dig_P7) << 4);

    return (uint32_t)p; /* Pa * 256 */
}


/* -------------------------------------------------------------------------- */
/* Inicialización                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t bmp280_init(void)
{
    if (s_bmp280 != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * El bus I2C debe haber sido creado previamente por efis_i2c_init().
     * Aquí solamente registramos el BMP280/BME280 en ese bus.
     */
    ESP_RETURN_ON_ERROR(
        efis_i2c_probe(BMP280_ADDR),
        TAG,
        "BMP280/BME280 no detectado");

    ESP_RETURN_ON_ERROR(
        efis_i2c_add_device(
            BMP280_ADDR,
            &s_bmp280),
        TAG,
        "No se pudo registrar BMP280/BME280");

    uint8_t id = 0;

    ESP_RETURN_ON_ERROR(
        bmp280_read(REG_CHIP_ID, &id, 1),
        TAG,
        "No responde el BMP280");

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
        ESP_LOGW(TAG, "Chip ID inesperado: 0x%02X", id);
    }

    ESP_RETURN_ON_ERROR(
        bmp280_write8(REG_RESET, 0xB6),
        TAG,
        "Error haciendo reset");

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_RETURN_ON_ERROR(
        bmp280_read_calibration(),
        TAG,
        "Error leyendo calibracion");

    /* standby 500 ms, filtro x4 */
    ESP_RETURN_ON_ERROR(
        bmp280_write8(REG_CONFIG, 0b10010000),
        TAG,
        "Error configurando REG_CONFIG");

    /* temperatura x1, presión x1, modo normal */
    ESP_RETURN_ON_ERROR(
        bmp280_write8(REG_CTRL_MEAS, 0b00100111),
        TAG,
        "Error configurando REG_CTRL_MEAS");

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Medida                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t bmp280_read_measurement(float *temp_c, float *press_hpa)
{
    if ((temp_c == NULL) || (press_hpa == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t d[6];

    ESP_RETURN_ON_ERROR(
        bmp280_read(REG_PRESS_MSB, d, sizeof(d)),
        TAG,
        "Error leyendo datos");

    int32_t adc_P =
        ((int32_t)d[0] << 12) |
        ((int32_t)d[1] << 4) |
        (d[2] >> 4);

    int32_t adc_T =
        ((int32_t)d[3] << 12) |
        ((int32_t)d[4] << 4) |
        (d[5] >> 4);

    int32_t T = bmp280_compensate_T(adc_T);
    uint32_t P = bmp280_compensate_P(adc_P);

    *temp_c = T / 100.0f;
    *press_hpa = (P / 256.0f) / 100.0f;

    return ESP_OK;
}


float bmp280_altitude_m(float pressure_hpa, float qnh_hpa)
{
    return 44330.0f *
           (1.0f - powf(pressure_hpa / qnh_hpa, 1.0f / 5.255f));
}
