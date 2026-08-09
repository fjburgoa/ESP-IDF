/**
 * @file main.c
 * @brief Prueba básica del BNO055 mediante I2C con ESP32-S3.
 *
 * El programa inicializa I2C, detecta el BNO055 en 0x28/0x29, configura
 * unidades SI y modo NDOF, y lee aceleración total, lineal y gravedad.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/i2c.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_SDA_GPIO 8
#define I2C_MASTER_SCL_GPIO 9
#define I2C_MASTER_FREQUENCY_HZ 400000
#define I2C_TIMEOUT_MS 100

#define BNO055_ADDR_LOW 0x28
#define BNO055_ADDR_HIGH 0x29

#define BNO055_REG_CHIP_ID 0x00
#define BNO055_REG_PAGE_ID 0x07
#define BNO055_REG_ACCEL_DATA_X_LSB 0x08
#define BNO055_REG_LINEAR_ACC_X_LSB 0x28
#define BNO055_REG_GRAVITY_X_LSB 0x2E
#define BNO055_REG_CALIB_STAT 0x35
#define BNO055_REG_UNIT_SEL 0x3B
#define BNO055_REG_OPR_MODE 0x3D
#define BNO055_REG_PWR_MODE 0x3E

#define BNO055_CHIP_ID_VALUE 0xA0
#define BNO055_MODE_CONFIG 0x00
#define BNO055_MODE_NDOF 0x0C
#define BNO055_POWER_NORMAL 0x00
#define BNO055_UNIT_SEL_SI 0x00
#define BNO055_ACCEL_LSB_PER_MS2 100.0f
#define BNO055_SAMPLE_PERIOD_MS 100U

static const char *TAG = "BNO055_TEST";
static uint8_t s_bno055_address = BNO055_ADDR_LOW;

typedef struct
{
    float x;
    float y;
    float z;
} vector3f_t;

//---------------------------------------------------------------------------
static esp_err_t i2c_master_init(void)
{
    const i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_GPIO,
        .scl_io_num = I2C_MASTER_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQUENCY_HZ,
        .clk_flags = 0,
    };

    ESP_RETURN_ON_ERROR(i2c_param_config(I2C_MASTER_PORT, &config), TAG, "No se pudo configurar I2C");

    ESP_RETURN_ON_ERROR(i2c_driver_install(I2C_MASTER_PORT, config.mode, 0, 0, 0), TAG, "No se pudo instalar el driver I2C");

    ESP_LOGI(TAG, "I2C inicializado: SDA=%d, SCL=%d, frecuencia=%lu Hz",
             I2C_MASTER_SDA_GPIO, I2C_MASTER_SCL_GPIO,
             (unsigned long)I2C_MASTER_FREQUENCY_HZ);

    return ESP_OK;
}

//---------------------------------------------------------------------------
static esp_err_t bno055_write_u8(uint8_t reg, uint8_t value)
{
    const uint8_t data[2] = {reg, value};
    return i2c_master_write_to_device(I2C_MASTER_PORT, s_bno055_address, data, sizeof(data), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

//---------------------------------------------------------------------------
static esp_err_t bno055_read(uint8_t reg, uint8_t *data, size_t length)
{
    if ((data == NULL) || (length == 0U))
        return ESP_ERR_INVALID_ARG;

    return i2c_master_write_read_device(I2C_MASTER_PORT, s_bno055_address, &reg, 1, data, length, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}
//---------------------------------------------------------------------------
static esp_err_t bno055_read_u8(uint8_t reg, uint8_t *value)
{
    return bno055_read(reg, value, 1);
}
//---------------------------------------------------------------------------
static esp_err_t bno055_detect(void)
{
    const uint8_t addresses[] = {BNO055_ADDR_LOW, BNO055_ADDR_HIGH};

    for (size_t i = 0; i < sizeof(addresses); ++i)
    {
        uint8_t chip_id = 0;
        s_bno055_address = addresses[i];

        esp_err_t err = bno055_read_u8(BNO055_REG_CHIP_ID, &chip_id);

        if ((err == ESP_OK) && (chip_id == BNO055_CHIP_ID_VALUE))
        {
            ESP_LOGI(TAG, "BNO055 detectado en 0x%02X; CHIP_ID=0x%02X", s_bno055_address, chip_id);
            return ESP_OK;
        }

        ESP_LOGW(TAG, "Sin BNO055 válido en 0x%02X: err=%s, CHIP_ID=0x%02X", s_bno055_address, esp_err_to_name(err), chip_id);
    }

    return ESP_ERR_NOT_FOUND;
}
//---------------------------------------------------------------------------
static esp_err_t bno055_set_mode(uint8_t mode)
{
    ESP_RETURN_ON_ERROR(
        bno055_write_u8(BNO055_REG_OPR_MODE, mode),
        TAG,
        "No se pudo cambiar OPR_MODE");

    if (mode == BNO055_MODE_CONFIG)
    {
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    else
    {
        /*
         * Margen deliberadamente conservador para el arranque
         * del sensor y la fusión interna.
         */
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return ESP_OK;
}

//---------------------------------------------------------------------------
static esp_err_t bno055_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(700));

    ESP_RETURN_ON_ERROR(bno055_detect(), TAG, "BNO055 no detectado");
    ESP_RETURN_ON_ERROR(bno055_set_mode(BNO055_MODE_CONFIG), TAG, "No se pudo entrar en CONFIGMODE");
    ESP_RETURN_ON_ERROR(bno055_write_u8(BNO055_REG_PAGE_ID, 0x00), TAG, "No se pudo seleccionar PAGE 0");
    ESP_RETURN_ON_ERROR(bno055_write_u8(BNO055_REG_PWR_MODE, BNO055_POWER_NORMAL), TAG, "No se pudo seleccionar POWER_NORMAL");

    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_RETURN_ON_ERROR(bno055_write_u8(BNO055_REG_UNIT_SEL, BNO055_UNIT_SEL_SI), TAG, "No se pudieron configurar las unidades");
    ESP_RETURN_ON_ERROR(bno055_set_mode(BNO055_MODE_NDOF), TAG, "No se pudo activar NDOF");

    vTaskDelay(pdMS_TO_TICKS(500));

    uint8_t mode = 0;
    uint8_t power = 0;
    uint8_t units = 0;

    ESP_RETURN_ON_ERROR(bno055_read_u8(BNO055_REG_OPR_MODE, &mode), TAG, "No se pudo verificar OPR_MODE");
    ESP_RETURN_ON_ERROR(bno055_read_u8(BNO055_REG_PWR_MODE, &power), TAG, "No se pudo verificar PWR_MODE");
    ESP_RETURN_ON_ERROR(bno055_read_u8(BNO055_REG_UNIT_SEL, &units), TAG, "No se pudo verificar UNIT_SEL");

    ESP_LOGI(TAG, "BNO055 inicializado: OPR_MODE=0x%02X, PWR_MODE=0x%02X, UNIT_SEL=0x%02X", mode, power, units);

    return ESP_OK;
}
//---------------------------------------------------------------------------
static int16_t bno055_make_i16(uint8_t lsb, uint8_t msb)
{
    return (int16_t)(((uint16_t)msb << 8) | lsb);
}
//---------------------------------------------------------------------------
static esp_err_t bno055_read_vector_ms2(uint8_t first_register, vector3f_t *vector)
{
    if (vector == NULL)
        return ESP_ERR_INVALID_ARG;

    uint8_t data[6];
    ESP_RETURN_ON_ERROR(bno055_read(first_register, data, sizeof(data)), TAG, "No se pudo leer el vector");

    vector->x = (float)bno055_make_i16(data[0], data[1]) / BNO055_ACCEL_LSB_PER_MS2;
    vector->y = (float)bno055_make_i16(data[2], data[3]) / BNO055_ACCEL_LSB_PER_MS2;
    vector->z = (float)bno055_make_i16(data[4], data[5]) / BNO055_ACCEL_LSB_PER_MS2;

    return ESP_OK;
}
//---------------------------------------------------------------------------
static esp_err_t bno055_read_calibration(uint8_t *system, uint8_t *gyro, uint8_t *accel, uint8_t *mag)
{
    if ((system == NULL) || (gyro == NULL) || (accel == NULL) || (mag == NULL))
        return ESP_ERR_INVALID_ARG;

    uint8_t status = 0;
    ESP_RETURN_ON_ERROR(bno055_read_u8(BNO055_REG_CALIB_STAT, &status), TAG, "No se pudo leer CALIB_STAT");

    *system = (status >> 6) & 0x03U;
    *gyro = (status >> 4) & 0x03U;
    *accel = (status >> 2) & 0x03U;
    *mag = status & 0x03U;

    return ESP_OK;
}
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
static void bno055_task(void *argument)
{
    (void)argument;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(BNO055_SAMPLE_PERIOD_MS);

    for (;;)
    {
        vector3f_t acceleration;
        vector3f_t linear_acceleration;
        vector3f_t gravity;

        esp_err_t err_acc = bno055_read_vector_ms2(BNO055_REG_ACCEL_DATA_X_LSB, &acceleration);
        esp_err_t err_lin = bno055_read_vector_ms2(BNO055_REG_LINEAR_ACC_X_LSB, &linear_acceleration);
        esp_err_t err_grv = bno055_read_vector_ms2(BNO055_REG_GRAVITY_X_LSB, &gravity);

        uint8_t cal_sys = 0;
        uint8_t cal_gyr = 0;
        uint8_t cal_acc = 0;
        uint8_t cal_mag = 0;

        esp_err_t err_cal = bno055_read_calibration(&cal_sys, &cal_gyr, &cal_acc, &cal_mag);

        if ((err_acc == ESP_OK) && (err_lin == ESP_OK) &&
            (err_grv == ESP_OK))
        {
            const float module = sqrtf(
                acceleration.x * acceleration.x +
                acceleration.y * acceleration.y +
                acceleration.z * acceleration.z);

            ESP_LOGI(TAG,
                     "ACC [m/s2] X=%7.3f Y=%7.3f Z=%7.3f |A|=%7.3f",
                     (double)acceleration.x,
                     (double)acceleration.y,
                     (double)acceleration.z,
                     (double)module);

            ESP_LOGI(TAG,
                     "LIN [m/s2] X=%7.3f Y=%7.3f Z=%7.3f | "
                     "GRV X=%7.3f Y=%7.3f Z=%7.3f",
                     (double)linear_acceleration.x,
                     (double)linear_acceleration.y,
                     (double)linear_acceleration.z,
                     (double)gravity.x,
                     (double)gravity.y,
                     (double)gravity.z);

            if (err_cal == ESP_OK)
                ESP_LOGI(TAG, "CAL SYS=%u GYR=%u ACC=%u MAG=%u", cal_sys, cal_gyr, cal_acc, cal_mag);
        }
        else
        {
            ESP_LOGE(TAG, "Error de lectura: ACC=%s LIN=%s GRV=%s",
                     esp_err_to_name(err_acc),
                     esp_err_to_name(err_lin),
                     esp_err_to_name(err_grv));
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(bno055_init());

    BaseType_t result = xTaskCreate(bno055_task, "bno055_task", 4096, NULL, 5, NULL);

    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "No se pudo crear la tarea BNO055");
        abort();
    }
}
