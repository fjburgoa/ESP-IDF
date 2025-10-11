
/*
Este código lee en bruto los datos del HMC588L3L
*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "HMC5883L";

// HMC5883L I2C Configuration
#define HMC5883L_I2C_PORT       I2C_NUM_0
#define HMC5883L_I2C_FREQ_HZ    100000  // 100kHz
#define HMC5883L_I2C_SDA_PIN    GPIO_NUM_8
#define HMC5883L_I2C_SCL_PIN    GPIO_NUM_9
#define HMC5883L_I2C_ADDRESS    0x1E    // 0x3C >> 1

// HMC5883L Register Map
#define HMC5883L_REG_CONFIG_A   0x00
#define HMC5883L_REG_CONFIG_B   0x01
#define HMC5883L_REG_MODE       0x02
#define HMC5883L_REG_DATA_X_MSB 0x03
#define HMC5883L_REG_DATA_X_LSB 0x04
#define HMC5883L_REG_DATA_Z_MSB 0x05
#define HMC5883L_REG_DATA_Z_LSB 0x06
#define HMC5883L_REG_DATA_Y_MSB 0x07
#define HMC5883L_REG_DATA_Y_LSB 0x08
#define HMC5883L_REG_STATUS     0x09
#define HMC5883L_REG_ID_A       0x0A
#define HMC5883L_REG_ID_B       0x0B
#define HMC5883L_REG_ID_C       0x0C

// Configuration Values
#define HMC5883L_AVERAGING_8    0x60  // 8 samples averaged
#define HMC5883L_AVERAGING_1    0x00  // 8 samples averaged
#define HMC5883L_RATE_15HZ      0x10  // 15 Hz data rate
#define HMC5883L_RATE_30HZ      0x14  // 30 Hz data rate
#define HMC5883L_BIAS_NORMAL    0x00  // Normal measurement mode

#define HMC5883L_GAIN_1370      0x00  // ±0.88 Ga, Gain = 1370 LSb/Gauss
#define HMC5883L_GAIN_1090      0x20  // ±1.3 Ga, Gain = 1090 LSb/Gauss  
#define HMC5883L_GAIN_820       0x40  // ±1.9 Ga, Gain = 820 LSb/Gauss
#define HMC5883L_GAIN_660       0x60  // ±2.5 Ga, Gain = 660 LSb/Gauss
#define HMC5883L_GAIN_440       0x80  // ±4.0 Ga, Gain = 440 LSb/Gauss
#define HMC5883L_GAIN_390       0xA0  // ±4.7 Ga, Gain = 390 LSb/Gauss
#define HMC5883L_GAIN_330       0xC0  // ±5.6 Ga, Gain = 330 LSb/Gauss
#define HMC5883L_GAIN_230       0xE0  // ±8.1 Ga, Gain = 230 LSb/Gauss

#define HMC5883L_MODE_CONTINUOUS 0x00
#define HMC5883L_MODE_SINGLE     0x01
#define HMC5883L_MODE_IDLE       0x02

// Magnetometer data structure
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} hmc5883l_data_t;



// Function to initialize I2C -----------------------------------------------
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = HMC5883L_I2C_SDA_PIN,
        .scl_io_num = HMC5883L_I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = HMC5883L_I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(HMC5883L_I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(HMC5883L_I2C_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C initialized successfully");
    return ESP_OK;
}

// Function to write to HMC5883L register -----------------------------------------------
static esp_err_t hmc5883l_write_register(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    
    esp_err_t ret = i2c_master_write_to_device(HMC5883L_I2C_PORT, HMC5883L_I2C_ADDRESS,  write_buf, sizeof(write_buf),  pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) 
        ESP_LOGE(TAG, "Failed to write register 0x%02X: %s", reg_addr, esp_err_to_name(ret));

    return ret;
}

// Function to read from HMC5883L register -----------------------------------------------
static esp_err_t hmc5883l_read_registers(uint8_t reg_addr, uint8_t *data, size_t len)
{
    esp_err_t ret = i2c_master_write_read_device(HMC5883L_I2C_PORT, HMC5883L_I2C_ADDRESS,  &reg_addr, 1, data, len, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) 
        ESP_LOGE(TAG, "Failed to read registers starting at 0x%02X: %s", reg_addr, esp_err_to_name(ret));

    return ret;
}

// Function to initialize HMC5883L  ------------------------------------------------------
static esp_err_t hmc5883l_init(void)
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
    ret = hmc5883l_write_register(HMC5883L_REG_CONFIG_A, HMC5883L_AVERAGING_1 | HMC5883L_RATE_15HZ | HMC5883L_BIAS_NORMAL);
    if (ret != ESP_OK) 
        return ret;

    // Configure gain
    ret = hmc5883l_write_register(HMC5883L_REG_CONFIG_B, HMC5883L_GAIN_230);
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
static esp_err_t hmc5883l_read_raw(hmc5883l_data_t *data)
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
static esp_err_t hmc5883l_data_ready(bool *ready)
{
    uint8_t status;
    esp_err_t ret = hmc5883l_read_registers(HMC5883L_REG_STATUS, &status, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *ready = (status & 0x01) != 0;
    return ESP_OK;
}


// Main task to read sensor data -------------------------------------------------------------------------
static void hmc5883l_read_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting HMC5883L read task");

    while (1) {
        bool ready;

        if (hmc5883l_data_ready(&ready) == ESP_OK && ready) 
        {
            hmc5883l_data_t raw_data; //, calibrated_data;
            
            if (hmc5883l_read_raw(&raw_data) == ESP_OK)         
                ESP_LOGI(TAG, "Raw: X=%d, Y=%d, Z=%d ", raw_data.x, raw_data.y, raw_data.z);

        } 
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Read every 10ms
    }
}


//----------------------------------------------------------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "Starting HMC5883L application");
    
    // Initialize I2C------------------------------------
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return;
    }
    
    // Initialize HMC5883L--------------------------------
    ret = hmc5883l_init();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to initialize HMC5883L");
        return;
    }
    
    // Create task to read sensor data --------------------
    xTaskCreate(hmc5883l_read_task, "hmc5883l_read_task", 4096, NULL, 5, NULL);

    while(1)
    {

    }

}