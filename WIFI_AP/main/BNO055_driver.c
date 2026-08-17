/**
 * @file BNO055_driver.c
 * @brief Driver de bajo nivel del BNO055.
 *
 * Responsabilidades:
 *   - Acceso I2C.
 *   - Detección del dispositivo.
 *   - Registros y factores de escala.
 *   - Configuración de modos de operación.
 *   - Remapeo de ejes V/H.
 *   - Composición LSB/MSB.
 *   - Conversión de registros a unidades físicas.
 *
 * No contiene filtros ni algoritmos de estimación de actitud.
 */

#include <stddef.h>
#include <stdint.h>

#include "driver/i2c.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "BNO055_driver.h"
#include "config.h"

/* -------------------------------------------------------------------------- */
/* Bus I2C                                                                    */
/* -------------------------------------------------------------------------- */

/*
 * El bus I2C se inicializa una sola vez en main.c y se comparte con BMP280.
 */
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_TIMEOUT_MS 100U

/* -------------------------------------------------------------------------- */
/* Direcciones y registros BNO055                                             */
/* -------------------------------------------------------------------------- */

#define BNO055_ADDR_LOW 0x28U
#define BNO055_ADDR_HIGH 0x29U

#define BNO055_REG_CHIP_ID 0x00U
#define BNO055_REG_PAGE_ID 0x07U
#define BNO055_REG_ACCEL_DATA_X_LSB 0x08U
#define BNO055_REG_MAG_DATA_X_LSB 0x0EU
#define BNO055_REG_GYRO_DATA_X_LSB 0x14U
#define BNO055_REG_EULER_H_LSB 0x1AU
#define BNO055_REG_QUATERNION_W_LSB 0x20U
#define BNO055_REG_LINEAR_ACC_X_LSB 0x28U
#define BNO055_REG_GRAVITY_X_LSB 0x2EU
#define BNO055_REG_TEMP 0x34U
#define BNO055_REG_CALIB_STAT 0x35U
#define BNO055_REG_UNIT_SEL 0x3BU
#define BNO055_REG_OPR_MODE 0x3DU
#define BNO055_REG_PWR_MODE 0x3EU
#define BNO055_REG_AXIS_MAP_CONFIG 0x41U
#define BNO055_REG_AXIS_MAP_SIGN 0x42U

#define BNO055_CHIP_ID_VALUE 0xA0U

#define BNO055_MODE_CONFIG 0x00U
#define BNO055_MODE_AMG 0x07U
#define BNO055_MODE_NDOF 0x0CU

#define BNO055_POWER_NORMAL 0x00U
#define BNO055_UNIT_SEL_SI 0x00U

/* -------------------------------------------------------------------------- */
/* Axis remap                                                                 */
/* -------------------------------------------------------------------------- */

/*
 * V = P1, orientación original/default.
 *
 * H = P0, giro de 90° en el plano XY:
 *
 *     X_aircraft = -Y_sensor
 *     Y_aircraft =  X_sensor
 *     Z_aircraft =  Z_sensor
 */
#define BNO055_AXIS_CONFIG_VERTICAL 0x24U
#define BNO055_AXIS_SIGN_VERTICAL 0x00U

#define BNO055_AXIS_CONFIG_HORIZONTAL 0x21U
#define BNO055_AXIS_SIGN_HORIZONTAL 0x04U

/* -------------------------------------------------------------------------- */
/* Factores de escala                                                         */
/* -------------------------------------------------------------------------- */

#define BNO055_ACCEL_LSB_PER_MS2 100.0f
#define BNO055_MAG_LSB_PER_UT 16.0f
#define BNO055_GYRO_LSB_PER_DPS 16.0f
#define BNO055_EULER_LSB_PER_DEG 16.0f
#define BNO055_QUATERNION_LSB 16384.0f

/* -------------------------------------------------------------------------- */

static const char *TAG = "BNO055_DRV";

static uint8_t s_bno055_address = BNO055_ADDR_LOW;
static bno055_mount_mode_t s_mount_mode = BNO055_MOUNT_VERTICAL;

/* -------------------------------------------------------------------------- */

static esp_err_t bno055_write_u8(uint8_t reg, uint8_t value)
{
    const uint8_t data[2] = {reg, value};

    return i2c_master_write_to_device(
        I2C_MASTER_PORT,
        s_bno055_address,
        data,
        sizeof(data),
        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

/* -------------------------------------------------------------------------- */

static esp_err_t bno055_read(
    uint8_t reg,
    uint8_t *data,
    size_t length)
{
    if ((data == NULL) || (length == 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_write_read_device(
        I2C_MASTER_PORT,
        s_bno055_address,
        &reg,
        1U,
        data,
        length,
        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

/* -------------------------------------------------------------------------- */

static esp_err_t bno055_read_u8(uint8_t reg, uint8_t *value)
{
    if (value == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return bno055_read(reg, value, 1U);
}

/* -------------------------------------------------------------------------- */

static int16_t bno055_make_i16(uint8_t lsb, uint8_t msb)
{
    return (int16_t)(((uint16_t)msb << 8) | lsb);
}

/* -------------------------------------------------------------------------- */

static esp_err_t bno055_detect(void)
{
    const uint8_t addresses[] = {
        BNO055_ADDR_LOW,
        BNO055_ADDR_HIGH};

    for (size_t i = 0U;
         i < sizeof(addresses) / sizeof(addresses[0]);
         ++i)
    {
        uint8_t chip_id = 0U;

        s_bno055_address = addresses[i];

        const esp_err_t err =
            bno055_read_u8(BNO055_REG_CHIP_ID, &chip_id);

        if ((err == ESP_OK) &&
            (chip_id == BNO055_CHIP_ID_VALUE))
        {
            ESP_LOGI(
                TAG,
                "BNO055 detectado en 0x%02X; CHIP_ID=0x%02X",
                s_bno055_address,
                chip_id);

            return ESP_OK;
        }

        ESP_LOGW(
            TAG,
            "Sin BNO055 válido en 0x%02X: err=%s, CHIP_ID=0x%02X",
            s_bno055_address,
            esp_err_to_name(err),
            chip_id);
    }

    return ESP_ERR_NOT_FOUND;
}

/* -------------------------------------------------------------------------- */

static esp_err_t bno055_set_mode(uint8_t mode)
{
    ESP_RETURN_ON_ERROR(
        bno055_write_u8(BNO055_REG_OPR_MODE, mode),
        TAG,
        "No se pudo cambiar OPR_MODE");

    if (mode == BNO055_MODE_CONFIG)
    {
        vTaskDelay(pdMS_TO_TICKS(25U));
    }
    else
    {
        vTaskDelay(pdMS_TO_TICKS(100U));
    }

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

static uint8_t bno055_selected_operation_mode(void)
{
#if BNO055_USE_INTERNAL_FUSION
    return BNO055_MODE_NDOF;
#else
    return BNO055_MODE_AMG;
#endif
}

/* -------------------------------------------------------------------------- */

static esp_err_t bno055_apply_mount_mode(
    bno055_mount_mode_t mode)
{
    uint8_t map_config;
    uint8_t map_sign;

    switch (mode)
    {
    case BNO055_MOUNT_VERTICAL:
        map_config = BNO055_AXIS_CONFIG_VERTICAL;
        map_sign = BNO055_AXIS_SIGN_VERTICAL;
        break;

    case BNO055_MOUNT_HORIZONTAL:
        map_config = BNO055_AXIS_CONFIG_HORIZONTAL;
        map_sign = BNO055_AXIS_SIGN_HORIZONTAL;
        break;

    default:
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(
        bno055_write_u8(
            BNO055_REG_AXIS_MAP_CONFIG,
            map_config),
        TAG,
        "No se pudo configurar AXIS_MAP_CONFIG");

    ESP_RETURN_ON_ERROR(
        bno055_write_u8(
            BNO055_REG_AXIS_MAP_SIGN,
            map_sign),
        TAG,
        "No se pudo configurar AXIS_MAP_SIGN");

    s_mount_mode = mode;

    ESP_LOGI(
        TAG,
        "Orientacion BNO055: %s (MAP=0x%02X SIGN=0x%02X)",
        mode == BNO055_MOUNT_VERTICAL ? "V" : "H",
        map_config,
        map_sign);

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

static esp_err_t bno055_read_vector_scaled(
    uint8_t first_register,
    float scale,
    bno055_vector3f_t *vector)
{
    if ((vector == NULL) || (scale <= 0.0f))
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[6];

    ESP_RETURN_ON_ERROR(
        bno055_read(
            first_register,
            data,
            sizeof(data)),
        TAG,
        "Error leyendo vector");

    vector->x =
        (float)bno055_make_i16(data[0], data[1]) / scale;

    vector->y =
        (float)bno055_make_i16(data[2], data[3]) / scale;

    vector->z =
        (float)bno055_make_i16(data[4], data[5]) / scale;

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* API interna del driver                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t bno055_driver_init(void)
{
    /*
     * El BNO055 puede necesitar varios cientos de milisegundos después de la
     * alimentación antes de responder de forma estable.
     */
    vTaskDelay(pdMS_TO_TICKS(700U));

    ESP_RETURN_ON_ERROR(
        bno055_detect(),
        TAG,
        "BNO055 no detectado");

    ESP_RETURN_ON_ERROR(
        bno055_set_mode(BNO055_MODE_CONFIG),
        TAG,
        "No se pudo entrar en CONFIGMODE");

    ESP_RETURN_ON_ERROR(
        bno055_write_u8(
            BNO055_REG_PAGE_ID,
            0x00U),
        TAG,
        "No se pudo seleccionar PAGE 0");

    ESP_RETURN_ON_ERROR(
        bno055_write_u8(
            BNO055_REG_PWR_MODE,
            BNO055_POWER_NORMAL),
        TAG,
        "No se pudo seleccionar POWER_NORMAL");

    vTaskDelay(pdMS_TO_TICKS(10U));

    ESP_RETURN_ON_ERROR(
        bno055_write_u8(
            BNO055_REG_UNIT_SEL,
            BNO055_UNIT_SEL_SI),
        TAG,
        "No se pudieron configurar las unidades");

    /*
     * Arranque seguro en V. Si NVS contiene H, websocket.c lo aplicará después.
     */
    ESP_RETURN_ON_ERROR(
        bno055_apply_mount_mode(BNO055_MOUNT_VERTICAL),
        TAG,
        "No se pudo aplicar la orientacion V");

    ESP_RETURN_ON_ERROR(
        bno055_set_mode(
            bno055_selected_operation_mode()),
        TAG,
        "No se pudo activar el modo de operacion");

    vTaskDelay(pdMS_TO_TICKS(500U));

    uint8_t mode = 0U;
    uint8_t power = 0U;
    uint8_t units = 0U;

    ESP_RETURN_ON_ERROR(
        bno055_read_u8(
            BNO055_REG_OPR_MODE,
            &mode),
        TAG,
        "No se pudo verificar OPR_MODE");

    ESP_RETURN_ON_ERROR(
        bno055_read_u8(
            BNO055_REG_PWR_MODE,
            &power),
        TAG,
        "No se pudo verificar PWR_MODE");

    ESP_RETURN_ON_ERROR(
        bno055_read_u8(
            BNO055_REG_UNIT_SEL,
            &units),
        TAG,
        "No se pudo verificar UNIT_SEL");

    ESP_LOGI(
        TAG,
        "Inicializado: OPR_MODE=0x%02X, PWR_MODE=0x%02X, UNIT_SEL=0x%02X",
        mode,
        power,
        units);

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

esp_err_t bno055_driver_set_mount_mode(
    bno055_mount_mode_t mode)
{
    if ((mode != BNO055_MOUNT_VERTICAL) &&
        (mode != BNO055_MOUNT_HORIZONTAL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (mode == s_mount_mode)
    {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(
        bno055_set_mode(BNO055_MODE_CONFIG),
        TAG,
        "No se pudo entrar en CONFIGMODE");

    ESP_RETURN_ON_ERROR(
        bno055_apply_mount_mode(mode),
        TAG,
        "No se pudo aplicar el remapeo");

    ESP_RETURN_ON_ERROR(
        bno055_set_mode(
            bno055_selected_operation_mode()),
        TAG,
        "No se pudo restaurar el modo de operacion");

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

bno055_mount_mode_t bno055_driver_get_mount_mode(void)
{
    return s_mount_mode;
}

/* -------------------------------------------------------------------------- */

esp_err_t bno055_driver_read_acceleration(
    bno055_vector3f_t *acceleration_ms2)
{
    return bno055_read_vector_scaled(
        BNO055_REG_ACCEL_DATA_X_LSB,
        BNO055_ACCEL_LSB_PER_MS2,
        acceleration_ms2);
}

/* -------------------------------------------------------------------------- */

esp_err_t bno055_driver_read_magnetic_field(
    bno055_vector3f_t *magnetic_field_ut)
{
    return bno055_read_vector_scaled(
        BNO055_REG_MAG_DATA_X_LSB,
        BNO055_MAG_LSB_PER_UT,
        magnetic_field_ut);
}

/* -------------------------------------------------------------------------- */

esp_err_t bno055_driver_read_gyro(
    bno055_vector3f_t *gyro_dps)
{
    return bno055_read_vector_scaled(
        BNO055_REG_GYRO_DATA_X_LSB,
        BNO055_GYRO_LSB_PER_DPS,
        gyro_dps);
}

/* -------------------------------------------------------------------------- */

esp_err_t bno055_driver_read_euler(
    float *heading_deg,
    float *roll_deg,
    float *pitch_deg)
{
    if ((heading_deg == NULL) ||
        (roll_deg == NULL) ||
        (pitch_deg == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[6];

    ESP_RETURN_ON_ERROR(
        bno055_read(
            BNO055_REG_EULER_H_LSB,
            data,
            sizeof(data)),
        TAG,
        "Error leyendo Euler");

    *heading_deg =
        (float)bno055_make_i16(data[0], data[1]) /
        BNO055_EULER_LSB_PER_DEG;

    *roll_deg =
        (float)bno055_make_i16(data[2], data[3]) /
        BNO055_EULER_LSB_PER_DEG;

    *pitch_deg =
        (float)bno055_make_i16(data[4], data[5]) /
        BNO055_EULER_LSB_PER_DEG;

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

esp_err_t bno055_driver_read_quaternion(
    bno055_quaternionf_t *quaternion)
{
    if (quaternion == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[8];

    ESP_RETURN_ON_ERROR(
        bno055_read(
            BNO055_REG_QUATERNION_W_LSB,
            data,
            sizeof(data)),
        TAG,
        "Error leyendo cuaternion");

    quaternion->w =
        (float)bno055_make_i16(data[0], data[1]) /
        BNO055_QUATERNION_LSB;

    quaternion->x =
        (float)bno055_make_i16(data[2], data[3]) /
        BNO055_QUATERNION_LSB;

    quaternion->y =
        (float)bno055_make_i16(data[4], data[5]) /
        BNO055_QUATERNION_LSB;

    quaternion->z =
        (float)bno055_make_i16(data[6], data[7]) /
        BNO055_QUATERNION_LSB;

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

esp_err_t bno055_driver_read_linear_acceleration(
    bno055_vector3f_t *linear_acceleration_ms2)
{
    return bno055_read_vector_scaled(
        BNO055_REG_LINEAR_ACC_X_LSB,
        BNO055_ACCEL_LSB_PER_MS2,
        linear_acceleration_ms2);
}

/* -------------------------------------------------------------------------- */

esp_err_t bno055_driver_read_gravity(
    bno055_vector3f_t *gravity_ms2)
{
    return bno055_read_vector_scaled(
        BNO055_REG_GRAVITY_X_LSB,
        BNO055_ACCEL_LSB_PER_MS2,
        gravity_ms2);
}

/* -------------------------------------------------------------------------- */

esp_err_t bno055_driver_read_temperature(
    int8_t *temperature_c)
{
    if (temperature_c == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw = 0U;

    ESP_RETURN_ON_ERROR(
        bno055_read_u8(
            BNO055_REG_TEMP,
            &raw),
        TAG,
        "Error leyendo temperatura");

    *temperature_c = (int8_t)raw;

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

esp_err_t bno055_driver_read_calibration(
    uint8_t *system,
    uint8_t *gyro,
    uint8_t *accel,
    uint8_t *mag)
{
    if ((system == NULL) ||
        (gyro == NULL) ||
        (accel == NULL) ||
        (mag == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t status = 0U;

    ESP_RETURN_ON_ERROR(
        bno055_read_u8(
            BNO055_REG_CALIB_STAT,
            &status),
        TAG,
        "Error leyendo calibracion");

    *system = (status >> 6) & 0x03U;
    *gyro = (status >> 4) & 0x03U;
    *accel = (status >> 2) & 0x03U;
    *mag = status & 0x03U;

    return ESP_OK;
}
