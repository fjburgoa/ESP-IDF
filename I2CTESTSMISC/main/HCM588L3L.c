/*

Driver para acceder al HCM588L3L

*/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "HCM588L3L.h"
#include <math.h>

#define I2C_MASTER_NUM I2C_NUM_0

#define HMC5883L_I2C_ADDRESS 0x1E // 0x3C >> 1

// HMC5883L Register Map
#define HMC5883L_REG_CONFIG_A 0x00
#define HMC5883L_REG_CONFIG_B 0x01
#define HMC5883L_REG_MODE 0x02
#define HMC5883L_REG_DATA_X_MSB 0x03
#define HMC5883L_REG_DATA_X_LSB 0x04
#define HMC5883L_REG_DATA_Z_MSB 0x05
#define HMC5883L_REG_DATA_Z_LSB 0x06
#define HMC5883L_REG_DATA_Y_MSB 0x07
#define HMC5883L_REG_DATA_Y_LSB 0x08
#define HMC5883L_REG_STATUS 0x09
#define HMC5883L_REG_ID_A 0x0A
#define HMC5883L_REG_ID_B 0x0B
#define HMC5883L_REG_ID_C 0x0C

// Configuration Values
#define HMC5883L_AVERAGING_8 0x60 // 8 samples averaged
#define HMC5883L_AVERAGING_1 0x00 // 1 samples averaged
#define HMC5883L_RATE_15HZ 0x10   // 15 Hz data rate
#define HMC5883L_RATE_30HZ 0x14   // 30 Hz data rate
#define HMC5883L_BIAS_NORMAL 0x00 // Normal measurement mode

#define HMC5883L_GAIN_1370 0x00 // ±0.88 Ga, Gain = 1370 LSb/Gauss
#define HMC5883L_GAIN_1090 0x20 // ±1.3 Ga, Gain = 1090 LSb/Gauss
#define HMC5883L_GAIN_820 0x40  // ±1.9 Ga, Gain = 820 LSb/Gauss
#define HMC5883L_GAIN_660 0x60  // ±2.5 Ga, Gain = 660 LSb/Gauss
#define HMC5883L_GAIN_440 0x80  // ±4.0 Ga, Gain = 440 LSb/Gauss
#define HMC5883L_GAIN_390 0xA0  // ±4.7 Ga, Gain = 390 LSb/Gauss
#define HMC5883L_GAIN_330 0xC0  // ±5.6 Ga, Gain = 330 LSb/Gauss
#define HMC5883L_GAIN_230 0xE0  // ±8.1 Ga, Gain = 230 LSb/Gauss

#define HMC5883L_MODE_CONTINUOUS 0x00
#define HMC5883L_MODE_SINGLE 0x01
#define HMC5883L_MODE_IDLE 0x02

static const char *TAG = "HCM588L3L";

// Function to write to HMC5883L register -----------------------------------------------
esp_err_t hmc5883l_write_register(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};

    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, HMC5883L_I2C_ADDRESS, write_buf, sizeof(write_buf), pdMS_TO_TICKS(1000));
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Failed to write register 0x%02X: %s", reg_addr, esp_err_to_name(ret));

    return ret;
}

// Function to read from HMC5883L register -----------------------------------------------
esp_err_t hmc5883l_read_registers(uint8_t reg_addr, uint8_t *data, size_t len)
{
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, HMC5883L_I2C_ADDRESS, &reg_addr, 1, data, len, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Failed to read registers starting at 0x%02X: %s", reg_addr, esp_err_to_name(ret));

    return ret;
}

// Function to initialize HMC5883L  ------------------------------------------------------
esp_err_t hmc5883l_init(void)
{
    // Check device identity
    uint8_t id[3];
    esp_err_t ret = hmc5883l_read_registers(HMC5883L_REG_ID_A, id, 3);
    if (ret != ESP_OK)
        return ret;

    if (id[0] != 'H' || id[1] != '4' || id[2] != '3')
    {
        ESP_LOGE(TAG, "Invalid device ID: %c%c%c (expected H43)", id[0], id[1], id[2]);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "HMC5883L found with ID: %c%c%c", id[0], id[1], id[2]);

    // Configure averaging, data rate, and bias
    ret = hmc5883l_write_register(HMC5883L_REG_CONFIG_A, HMC5883L_AVERAGING_8 | HMC5883L_RATE_30HZ | HMC5883L_BIAS_NORMAL);
    if (ret != ESP_OK)
        return ret;

    // Configure gain
    ret = hmc5883l_write_register(HMC5883L_REG_CONFIG_B, HMC5883L_GAIN_1090);
    if (ret != ESP_OK)
        return ret;

    // Set continuous measurement mode
    ret = hmc5883l_write_register(HMC5883L_REG_MODE, HMC5883L_MODE_CONTINUOUS);
    if (ret != ESP_OK)
        return ret;

    ESP_LOGI(TAG, "HMC5883L initialized successfully");
    return ESP_OK;
}

// Function to read raw magnetometer data  --------------------------------------------------------------------------
esp_err_t hmc5883l_read_raw(hmc5883l_data_t *data)
{
    uint8_t raw_data[6];

    // Read all 6 data registers (X, Z, Y)
    esp_err_t ret = hmc5883l_read_registers(HMC5883L_REG_DATA_X_MSB, raw_data, 6);
    if (ret != ESP_OK)
        return ret;

    // Convert to 16-bit values (HMC5883L uses X, Z, Y order)
    data->x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    data->z = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    data->y = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    return ESP_OK;
}

// Function to check if data is ready -----------------------------------------------------------------------------------------
esp_err_t hmc5883l_data_ready(bool *ready)
{
    uint8_t status;
    esp_err_t ret = hmc5883l_read_registers(HMC5883L_REG_STATUS, &status, 1);
    if (ret != ESP_OK)
    {
        return ret;
    }

    *ready = (status & 0x01) != 0;
    return ESP_OK;
}

// #define HMC5883L_LSB_PER_GAUSS 1090.0f
#define MAG_X_OFFSET (-2.0f)
#define MAG_Y_OFFSET (-126.5f)

// Main function to read sensor data -------------------------------------------------------------------------
float hmc5883l_calculate_heading(void)
{
    float heading_deg = 0.0;

    hmc5883l_data_t raw_data; //, calibrated_data;

    if (hmc5883l_read_raw(&raw_data) == ESP_OK)
    {
        float x_cal = (float)raw_data.x - MAG_X_OFFSET;
        float y_cal = (float)raw_data.y - MAG_Y_OFFSET;

        float heading_rad = atan2f(y_cal, x_cal);

        if (heading_rad < 0.0f)
            heading_rad += 2.0f * M_PI;

        heading_deg = heading_rad * 180.0f / M_PI;
    }

    return heading_deg;
}

#define TEXT_LEN 32

//------------------------------------------------------------------------------------------------
char *heading_to_direction(float heading_deg)
{
    /*
     * Sectores de 45 grados:
     *
     * N:  337.5 ... 360 y 0 ... 22.5
     * NE: 22.5  ... 67.5
     * E:  67.5  ... 112.5
     * SE: 112.5 ... 157.5
     * S:  157.5 ... 202.5
     * SO: 202.5 ... 247.5
     * O:  247.5 ... 292.5
     * NO: 292.5 ... 337.5
     */

    char *directions[] = {
        "N", "NE", "E", "SE",
        "S", "SO", "O", "NO"};

    // Normalización al intervalo [0, 360)
    heading_deg = fmodf(heading_deg, 360.0f);

    if (heading_deg < 0.0f)
    {
        heading_deg += 360.0f;
    }

    int sector = (int)((heading_deg + 22.5f) / 45.0f) % 8;

    return directions[sector];
}