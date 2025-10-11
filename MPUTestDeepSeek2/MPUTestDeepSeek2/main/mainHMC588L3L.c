/*
Este código lee, calibra y computa la orientación de la brújula a partir de los datos del HMC588L3L leidos por I2c
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

// Calibration data
typedef struct {
    float x_scale;
    float y_scale;
    float z_scale;
    int16_t x_offset;
    int16_t y_offset;
    int16_t z_offset;
} hmc5883l_calib_t;

static hmc5883l_calib_t calibration = {
    .x_scale = 1.0,
    .y_scale = 1.0,
    .z_scale = 1.0,
    .x_offset = 0,
    .y_offset = 0,
    .z_offset = 0
};

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

//Function to apply calibration ---------------------------------------------------------------------------------------
static void hmc5883l_apply_calibration(hmc5883l_data_t *raw, hmc5883l_data_t *calibrated)
{
    calibrated->x = (raw->x - calibration.x_offset) * calibration.x_scale;
    calibrated->y = (raw->y - calibration.y_offset) * calibration.y_scale;
    calibrated->z = (raw->z - calibration.z_offset) * calibration.z_scale;
}


//Function to convert raw data to Gauss -------------------------------------------------------------------------------
static void hmc5883l_to_gauss(hmc5883l_data_t *raw, float *x_gauss, float *y_gauss, float *z_gauss)
{
    // For gain setting 1090 (±1.3 Ga, Gain = 1090 LSb/Gauss)
    float gain_factor = 1.0 / 1090.0;

    *x_gauss = raw->x * gain_factor;
    *y_gauss = raw->y * gain_factor;
    *z_gauss = raw->z * gain_factor;
}

//Function to calculate heading (in radians) --------------------------------------------------------------------------
static float hmc5883l_calculate_heading(float x_gauss, float y_gauss)
{
    float heading = atan2f(y_gauss, x_gauss);

    // Adjust for declination angle (you need to set this for your location).... Example for London: 0.2 degrees West -> about -0.0035 radians
    float declination_angle = 0.0; // Set this for your location

    heading += declination_angle;

    // Correct for when signs are reversed
    if (heading < 0)
        heading += 2 * M_PI;

    if (heading > 2 * M_PI)
        heading -= 2 * M_PI;

    return heading;
}

//Function to convert radians to degrees ----------------------------------------------------------------------------------
static float radians_to_degrees(float radians)
{
    return radians * (180.0 / M_PI);
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

// Simple calibration routine (run this with sensor in 8-figure pattern) -------------------------------------------------------

static esp_err_t hmc5883l_calibrate(void)
{
    ESP_LOGI(TAG, "Starting calibration - move sensor in 8-figure pattern for 30 seconds");

    int16_t x_min = 32767, x_max = -32768;
    int16_t y_min = 32767, y_max = -32768;
    int16_t z_min = 32767, z_max = -32768;

    int samples = 0;
    uint32_t start_time = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(30000)) {
        hmc5883l_data_t data;

        if (hmc5883l_read_raw(&data) == ESP_OK) {
            if (data.x < x_min) x_min = data.x;
            if (data.x > x_max) x_max = data.x;
            if (data.y < y_min) y_min = data.y;
            if (data.y > y_max) y_max = data.y;
            if (data.z < z_min) z_min = data.z;
            if (data.z > z_max) z_max = data.z;

            samples++;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (samples == 0) {
        ESP_LOGE(TAG, "No samples collected during calibration");
        return ESP_FAIL;
    }

    //Calculate offsets and scale factors
    calibration.x_offset = (x_min + x_max) / 2;
    calibration.y_offset = (y_min + y_max) / 2;
    calibration.z_offset = (z_min + z_max) / 2;

    float x_range = (x_max - x_min) / 2.0;
    float y_range = (y_max - y_min) / 2.0;
    float z_range = (z_max - z_min) / 2.0;

    float avg_range = (x_range + y_range + z_range) / 3.0;

    calibration.x_scale = avg_range / x_range;
    calibration.y_scale = avg_range / y_range;
    calibration.z_scale = avg_range / z_range;

    ESP_LOGI(TAG, "Calibration completed with %d samples", samples);
    ESP_LOGI(TAG, "Offsets: X=%d, Y=%d, Z=%d",
             calibration.x_offset, calibration.y_offset, calibration.z_offset);
    ESP_LOGI(TAG, "Scales: X=%.3f, Y=%.3f, Z=%.3f",
             calibration.x_scale, calibration.y_scale, calibration.z_scale);

    return ESP_OK;
}


// Main task to read sensor data
static void hmc5883l_read_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting HMC5883L read task");

    // Optional: Run calibration (comment out if not needed)
     hmc5883l_calibrate();
    while (1) {
        bool ready;

        if (hmc5883l_data_ready(&ready) == ESP_OK && ready)
        {
            hmc5883l_data_t raw_data = {0};
            hmc5883l_data_t calibrated_data = {0};

            if (hmc5883l_read_raw(&raw_data) == ESP_OK)
            {

                hmc5883l_apply_calibration(&raw_data, &calibrated_data); // Apply calibration

                // Convert to Gauss
                float x_g, y_g, z_g;
                hmc5883l_to_gauss(&calibrated_data, &x_g, &y_g, &z_g);

                // Calculate heading
                float heading_rad = hmc5883l_calculate_heading(x_g, y_g);
                float heading_deg = radians_to_degrees(heading_rad);

                //ESP_LOGI(TAG, "Raw: X=%d, Y=%d, Z=%d ", raw_data.x, raw_data.y, raw_data.z);
                //ESP_LOGI(TAG, "Cal: X=%6d, Y=%6d, Z=%6d", calibrated_data.x, calibrated_data.y, calibrated_data.z);
                //ESP_LOGI(TAG, "Field: X=%7.3fG, Y=%7.3fG, Z=%7.3fG", x_g, y_g, z_g);
                ESP_LOGI(TAG, "Heading: %.1f° (%.3f rad)", heading_deg, heading_rad);
                //ESP_LOGI(TAG, "---");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Read every 10ms
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting HMC5883L application");

    // Initialize I2C
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return;
    }

    // Initialize HMC5883L
    ret = hmc5883l_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize HMC5883L");
        return;
    }

    // Create task to read sensor data
    xTaskCreate(hmc5883l_read_task, "hmc5883l_read_task", 4096, NULL, 5, NULL);
    while(1)
    {

    }

}