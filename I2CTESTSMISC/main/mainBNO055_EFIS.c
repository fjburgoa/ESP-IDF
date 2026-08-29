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
#define BNO055_REG_MAG_DATA_X_LSB 0x0E
#define BNO055_REG_GYRO_DATA_X_LSB 0x14
#define BNO055_REG_EULER_H_LSB 0x1A
#define BNO055_REG_QUATERNION_W_LSB 0x20
#define BNO055_REG_LINEAR_ACC_X_LSB 0x28
#define BNO055_REG_GRAVITY_X_LSB 0x2E
#define BNO055_REG_TEMP 0x34
#define BNO055_REG_CALIB_STAT 0x35
#define BNO055_REG_UNIT_SEL 0x3B
#define BNO055_REG_OPR_MODE 0x3D
#define BNO055_REG_PWR_MODE 0x3E

#define BNO055_CHIP_ID_VALUE 0xA0
#define BNO055_MODE_CONFIG 0x00
#define BNO055_MODE_NDOF 0x0C    // NDOF
#define BNO055_MODE_ACCONLY 0x01 // ACC ONLY
#define BNO055_POWER_NORMAL 0x00
#define BNO055_UNIT_SEL_SI 0x00
#define BNO055_ACCEL_LSB_PER_MS2 100.0f
#define BNO055_MAG_LSB_PER_UT 16.0f
#define BNO055_GYRO_LSB_PER_DPS 16.0f
#define BNO055_EULER_LSB_PER_DEG 16.0f
#define BNO055_QUATERNION_LSB 16384.0f
#define BNO055_SAMPLE_PERIOD_MS 100U

static const char *TAG = "BNO055_TEST";
static uint8_t s_bno055_address = BNO055_ADDR_HIGH;

typedef struct
{
    float x;
    float y;
    float z;
} vector3f_t;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} quaternionf_t;

typedef struct
{
    vector3f_t acceleration_ms2;
    vector3f_t magnetic_field_ut;
    vector3f_t gyro_dps;

    float heading_deg;
    float roll_deg;
    float pitch_deg;

    quaternionf_t quaternion;

    vector3f_t linear_acceleration_ms2;
    vector3f_t gravity_ms2;

    int8_t temperature_c;

    uint8_t calibration_system;
    uint8_t calibration_gyro;
    uint8_t calibration_accel;
    uint8_t calibration_mag;

    bool valid;
} bno055_efis_data_t;

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

#define BNO055_REG_SYS_TRIGGER 0x3FU
#define BNO055_SYS_TRIGGER_RST 0x20U

static esp_err_t bno055_software_reset(void)
{
    /*
     * SYS_TRIGGER.RST_SYS = 1
     *
     * Reinicia el BNO055 por software. Después del reset el dispositivo
     * necesita tiempo para arrancar de nuevo antes de aceptar comandos.
     */
    ESP_RETURN_ON_ERROR(
        bno055_write_u8(BNO055_REG_PAGE_ID, 0x00U),
        TAG,
        "No se pudo seleccionar PAGE 0");

    ESP_RETURN_ON_ERROR(
        bno055_write_u8(BNO055_REG_SYS_TRIGGER, BNO055_SYS_TRIGGER_RST),
        TAG,
        "No se pudo realizar el reset del BNO055");

    /*
     * Tras escribir RST_SYS se pierde temporalmente la comunicación con
     * el dispositivo mientras se reinicia.
     */
    vTaskDelay(pdMS_TO_TICKS(700U));

    return ESP_OK;
}

//---------------------------------------------------------------------------
static esp_err_t bno055_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(700));

    ESP_RETURN_ON_ERROR(bno055_detect(), TAG, "BNO055 no detectado");

    /* Reset completo del BNO055. */
  //  ESP_LOGI(TAG, "Realizando software reset");

  //  ESP_RETURN_ON_ERROR(bno055_software_reset(), TAG, "Error durante software reset");

  //  ESP_RETURN_ON_ERROR(bno055_detect(), TAG, "BNO055 no responde despues del reset");

  //  ESP_LOGI(TAG, "BNO055 reiniciado correctamente");

    ESP_RETURN_ON_ERROR(bno055_set_mode(BNO055_MODE_CONFIG), TAG, "No se pudo entrar en CONFIGMODE");

    ESP_RETURN_ON_ERROR(bno055_write_u8(BNO055_REG_PAGE_ID, 0x00), TAG, "No se pudo seleccionar PAGE 0");
    ESP_RETURN_ON_ERROR(bno055_write_u8(BNO055_REG_PWR_MODE, BNO055_POWER_NORMAL), TAG, "No se pudo seleccionar POWER_NORMAL");

    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_RETURN_ON_ERROR(bno055_write_u8(BNO055_REG_UNIT_SEL, BNO055_UNIT_SEL_SI), TAG, "No se pudieron configurar las unidades");

    //    ESP_RETURN_ON_ERROR(bno055_set_mode(BNO055_MODE_NDOF), TAG, "No se pudo activar NDOF");
    ESP_RETURN_ON_ERROR(bno055_set_mode(BNO055_MODE_ACCONLY), TAG, "No se pudo activar ACCONLY");

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
static esp_err_t bno055_read_vector_scaled(uint8_t first_register, float scale, vector3f_t *vector)
{
    if ((vector == NULL) || (scale <= 0.0f))
        return ESP_ERR_INVALID_ARG;

    uint8_t data[6];
    esp_err_t err = bno055_read(first_register, data, sizeof(data));
    if (err != ESP_OK)
        return err;

    vector->x = (float)bno055_make_i16(data[0], data[1]) / scale;
    vector->y = (float)bno055_make_i16(data[2], data[3]) / scale;
    vector->z = (float)bno055_make_i16(data[4], data[5]) / scale;

    return ESP_OK;
}
//---------------------------------------------------------------------------
static esp_err_t bno055_read_euler(float *heading_deg, float *roll_deg, float *pitch_deg)
{
    if ((heading_deg == NULL) || (roll_deg == NULL) || (pitch_deg == NULL))
        return ESP_ERR_INVALID_ARG;

    uint8_t data[6];
    esp_err_t err = bno055_read(BNO055_REG_EULER_H_LSB, data, sizeof(data));
    if (err != ESP_OK)
        return err;

    *heading_deg = (float)bno055_make_i16(data[0], data[1]) / BNO055_EULER_LSB_PER_DEG;
    *roll_deg = (float)bno055_make_i16(data[2], data[3]) / BNO055_EULER_LSB_PER_DEG;
    *pitch_deg = (float)bno055_make_i16(data[4], data[5]) / BNO055_EULER_LSB_PER_DEG;

    return ESP_OK;
}
//---------------------------------------------------------------------------
static esp_err_t bno055_read_quaternion(quaternionf_t *quaternion)
{
    if (quaternion == NULL)
        return ESP_ERR_INVALID_ARG;

    uint8_t data[8];
    esp_err_t err = bno055_read(BNO055_REG_QUATERNION_W_LSB, data, sizeof(data));
    if (err != ESP_OK)
        return err;

    quaternion->w = (float)bno055_make_i16(data[0], data[1]) / BNO055_QUATERNION_LSB;
    quaternion->x = (float)bno055_make_i16(data[2], data[3]) / BNO055_QUATERNION_LSB;
    quaternion->y = (float)bno055_make_i16(data[4], data[5]) / BNO055_QUATERNION_LSB;
    quaternion->z = (float)bno055_make_i16(data[6], data[7]) / BNO055_QUATERNION_LSB;

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
static esp_err_t bno055_read_efis_data(bno055_efis_data_t *data)
{
    if (data == NULL)
        return ESP_ERR_INVALID_ARG;

    bno055_efis_data_t sample = {0};
    esp_err_t err;

    err = bno055_read_vector_ms2(BNO055_REG_ACCEL_DATA_X_LSB, &sample.acceleration_ms2);
    if (err != ESP_OK)
        return err;

    err = bno055_read_vector_scaled(BNO055_REG_MAG_DATA_X_LSB, BNO055_MAG_LSB_PER_UT, &sample.magnetic_field_ut);
    if (err != ESP_OK)
        return err;

    err = bno055_read_vector_scaled(BNO055_REG_GYRO_DATA_X_LSB, BNO055_GYRO_LSB_PER_DPS, &sample.gyro_dps);
    if (err != ESP_OK)
        return err;

    err = bno055_read_euler(&sample.heading_deg, &sample.roll_deg, &sample.pitch_deg);
    if (err != ESP_OK)
        return err;

    err = bno055_read_quaternion(&sample.quaternion);
    if (err != ESP_OK)
        return err;

    err = bno055_read_vector_ms2(BNO055_REG_LINEAR_ACC_X_LSB, &sample.linear_acceleration_ms2);
    if (err != ESP_OK)
        return err;

    err = bno055_read_vector_ms2(BNO055_REG_GRAVITY_X_LSB, &sample.gravity_ms2);
    if (err != ESP_OK)
        return err;

    uint8_t temperature_raw = 0;
    err = bno055_read_u8(BNO055_REG_TEMP, &temperature_raw);
    if (err != ESP_OK)
        return err;
    sample.temperature_c = (int8_t)temperature_raw;

    err = bno055_read_calibration(&sample.calibration_system, &sample.calibration_gyro, &sample.calibration_accel, &sample.calibration_mag);
    if (err != ESP_OK)
        return err;

    sample.valid = true;
    *data = sample;
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
        bno055_efis_data_t efis = {0};
        esp_err_t err = bno055_read_efis_data(&efis);

        if (err == ESP_OK)
        {
            const float acceleration_module = sqrtf(
                efis.acceleration_ms2.x * efis.acceleration_ms2.x +
                efis.acceleration_ms2.y * efis.acceleration_ms2.y +
                efis.acceleration_ms2.z * efis.acceleration_ms2.z);

            ESP_LOGI(TAG, "EUL H=%7.2f R=%7.2f P=%7.2f deg",
                     (double)efis.heading_deg,
                     (double)efis.roll_deg,
                     (double)efis.pitch_deg);

            ESP_LOGI(TAG, "GYR [deg/s] X=%7.2f Y=%7.2f Z=%7.2f",
                     (double)efis.gyro_dps.x,
                     (double)efis.gyro_dps.y,
                     (double)efis.gyro_dps.z);

            ESP_LOGI(TAG, "ACC [m/s2] X=%7.3f Y=%7.3f Z=%7.3f |A|=%7.3f",
                     (double)efis.acceleration_ms2.x,
                     (double)efis.acceleration_ms2.y,
                     (double)efis.acceleration_ms2.z,
                     (double)acceleration_module);

            ESP_LOGI(TAG, "LIN [m/s2] X=%7.3f Y=%7.3f Z=%7.3f",
                     (double)efis.linear_acceleration_ms2.x,
                     (double)efis.linear_acceleration_ms2.y,
                     (double)efis.linear_acceleration_ms2.z);

            ESP_LOGI(TAG, "GRV [m/s2] X=%7.3f Y=%7.3f Z=%7.3f",
                     (double)efis.gravity_ms2.x,
                     (double)efis.gravity_ms2.y,
                     (double)efis.gravity_ms2.z);

            ESP_LOGI(TAG, "MAG [uT] X=%7.2f Y=%7.2f Z=%7.2f",
                     (double)efis.magnetic_field_ut.x,
                     (double)efis.magnetic_field_ut.y,
                     (double)efis.magnetic_field_ut.z);

            ESP_LOGI(TAG, "QUA W=% .5f X=% .5f Y=% .5f Z=% .5f",
                     (double)efis.quaternion.w,
                     (double)efis.quaternion.x,
                     (double)efis.quaternion.y,
                     (double)efis.quaternion.z);

            ESP_LOGI(TAG, "TEMP=%d C | CAL SYS=%u GYR=%u ACC=%u MAG=%u",
                     efis.temperature_c,
                     efis.calibration_system,
                     efis.calibration_gyro,
                     efis.calibration_accel,
                     efis.calibration_mag);

            uint8_t acc_id = 0;
            uint8_t mag_id = 0;
            uint8_t gyr_id = 0;

            ESP_ERROR_CHECK(bno055_write_u8(BNO055_REG_PAGE_ID, 0x00));

            ESP_ERROR_CHECK(bno055_read_u8(0x01, &acc_id));
            ESP_ERROR_CHECK(bno055_read_u8(0x02, &mag_id));
            ESP_ERROR_CHECK(bno055_read_u8(0x03, &gyr_id));

            ESP_LOGI(TAG, "ACC_ID=0x%02X MAG_ID=0x%02X GYR_ID=0x%02X", acc_id, mag_id, gyr_id);

            uint8_t raw_acc[6];

            ESP_ERROR_CHECK(bno055_read(BNO055_REG_ACCEL_DATA_X_LSB, raw_acc, sizeof(raw_acc)));

            ESP_LOGI(TAG, "ACC RAW = %02X %02X %02X %02X %02X %02X", raw_acc[0], raw_acc[1], raw_acc[2], raw_acc[3], raw_acc[4], raw_acc[5]);
        }
        else
        {
            ESP_LOGE(TAG, "Error leyendo datos EFIS: %s", esp_err_to_name(err));
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
