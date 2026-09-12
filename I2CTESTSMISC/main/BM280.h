#ifndef _BM280_H
#define _BM280_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * Dirección I2C del BMP280/BME280.
 * Cambiar a 0x77 si SDO está a nivel alto.
 */
#define BMP280_ADDR 0x76

#define BMP280_I2C_TIMEOUT_MS 100

#define REG_CHIP_ID   0xD0
#define REG_RESET     0xE0
#define REG_CTRL_MEAS 0xF4
#define REG_CONFIG    0xF5
#define REG_PRESS_MSB 0xF7
#define REG_CALIB     0x88


esp_err_t bmp280_write8(uint8_t reg, uint8_t data);
esp_err_t bmp280_read(uint8_t reg, uint8_t *data, size_t len);

uint16_t u16_le(const uint8_t *p);
int16_t s16_le(const uint8_t *p);

esp_err_t bmp280_read_calibration(void);

int32_t bmp280_compensate_T(int32_t adc_T);
uint32_t bmp280_compensate_P(int32_t adc_P);

esp_err_t bmp280_init(void);

esp_err_t bmp280_read_measurement(
    float *temp_c,
    float *press_hpa);

float bmp280_altitude_m(
    float pressure_hpa,
    float qnh_hpa);

#endif
