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

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "BNO055_driver.h"
#include "config.h"
#include "EFIS_I2C.h"

/* -------------------------------------------------------------------------- */
/* Bus I2C                                                                    */
/* -------------------------------------------------------------------------- */

/*
 * El bus I2C se inicializa una sola vez en main.c y se comparte con BMP280.
 */

/* -------------------------------------------------------------------------- */
/* Direcciones y registros BNO055                                             */
/* -------------------------------------------------------------------------- */

#define BNO055_ADDR_LOW 0x28U
#define BNO055_ADDR_HIGH 0x29U

#define BNO055_REG_CHIP_ID 0x00U
#define BNO055_REG_PAGE_ID 0x07U
#define BNO055_REG_ACCEL_DATA_X_LSB 0x08U
#define BNO055_REG_GYRO_DATA_X_LSB 0x14U
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
#define BNO055_MODE_NDOF_FMC_OFF 0x0BU
#define BNO055_MODE_IMU 0x08U

#define BNO055_POWER_NORMAL 0x00U
#define BNO055_UNIT_SEL_SI 0x00U

#define BNO055_REG_ACC_CONFIG 0x08U   // Page 1
#define BNO055_REG_GYR_CONFIG_0 0x0AU // Page 1
#define BNO055_REG_GYR_CONFIG_1 0x0BU // Page 1

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
#define BNO055_GYRO_LSB_PER_DPS 16.0f
#define BNO055_QUATERNION_LSB 16384.0f

/* -------------------------------------------------------------------------- */

static const char *TAG = "BNO055_DRV";

static uint8_t s_bno055_address = BNO055_ADDR_LOW;
static i2c_master_dev_handle_t s_bno055_dev = NULL;
static bno055_mount_mode_t s_mount_mode = BNO055_MOUNT_VERTICAL;
/*
 * Modo de operación REAL escrito en OPR_MODE.
 *
 * Para BNOTESTMODE arrancamos deliberadamente en AMG y después se conmuta:
 *
 *   AMG -> IMUPLUS -> NDOF -> NDOF_FMC_OFF -> AMG ...
 *
 * Esto evita esconder el registro real detrás de un enum AMG/NDOF.
 */
#if BNOTESTMODE
static uint8_t s_operation_mode_reg = BNO055_MODE_AMG;
#else
static uint8_t s_operation_mode_reg =
    (BNO055_DEFAULT_OPERATION_MODE == 0U) ? BNO055_MODE_AMG : BNO055_MODE_NDOF;
#endif

/* -------------------------------------------------------------------------- */

static esp_err_t bno055_write_u8(uint8_t reg, uint8_t value)
{
    if (s_bno055_dev == NULL)
        return ESP_ERR_INVALID_STATE;

    const uint8_t data[2] = {reg, value};

    return i2c_master_transmit(
        s_bno055_dev,
        data,
        sizeof(data),
        EFIS_I2C_TIMEOUT_MS);
}

/* -------------------------------------------------------------------------- */

static esp_err_t bno055_read(
    uint8_t reg,
    uint8_t *data,
    size_t length)
{
    if ((s_bno055_dev == NULL) || (data == NULL) || (length == 0U))
        return ESP_ERR_INVALID_ARG;

    return i2c_master_transmit_receive(
        s_bno055_dev,
        &reg,
        1U,
        data,
        length,
        EFIS_I2C_TIMEOUT_MS);
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

static esp_err_t bno055_configure_sensors(void)
{
    /*
     * ACC_CONFIG, GYR_CONFIG_0 y GYR_CONFIG_1 estan situados en PAGE 1.
     * La configuracion solo debe realizarse en CONFIGMODE.
     */
    ESP_RETURN_ON_ERROR(
        bno055_write_u8(BNO055_REG_PAGE_ID, 0x01U),
        TAG,
        "No se pudo seleccionar PAGE 1");

    ESP_RETURN_ON_ERROR(
        bno055_write_u8(BNO055_REG_ACC_CONFIG, BNO055_ACC_CONFIG_EFIS),
        TAG,
        "No se pudo configurar el acelerometro");

    ESP_RETURN_ON_ERROR(
        bno055_write_u8(BNO055_REG_GYR_CONFIG_0, BNO055_GYR_CONFIG_0_EFIS),
        TAG,
        "No se pudo configurar rango/bandwidth del giroscopo");

    ESP_RETURN_ON_ERROR(
        bno055_write_u8(BNO055_REG_GYR_CONFIG_1, BNO055_GYR_CONFIG_1_EFIS),
        TAG,
        "No se pudo configurar power mode del giroscopo");

    uint8_t acc_config = 0U;
    uint8_t gyr_config0 = 0U;
    uint8_t gyr_config1 = 0U;

    ESP_RETURN_ON_ERROR(bno055_read_u8(BNO055_REG_ACC_CONFIG, &acc_config), TAG,
                        "No se pudo verificar ACC_CONFIG");
    ESP_RETURN_ON_ERROR(bno055_read_u8(BNO055_REG_GYR_CONFIG_0, &gyr_config0), TAG,
                        "No se pudo verificar GYR_CONFIG_0");
    ESP_RETURN_ON_ERROR(bno055_read_u8(BNO055_REG_GYR_CONFIG_1, &gyr_config1), TAG,
                        "No se pudo verificar GYR_CONFIG_1");

    ESP_LOGI(TAG,
             "Configuracion IMU: ACC_CONFIG=0x%02X, "
             "GYR_CONFIG_0=0x%02X, "
             "GYR_CONFIG_1=0x%02X (Normal)",
             acc_config, gyr_config0, gyr_config1);

    ESP_RETURN_ON_ERROR(
        bno055_write_u8(BNO055_REG_PAGE_ID, 0x00U),
        TAG,
        "No se pudo volver a PAGE 0");

    if ((acc_config != BNO055_ACC_CONFIG_EFIS) ||
        (gyr_config0 != BNO055_GYR_CONFIG_0_EFIS) ||
        (gyr_config1 != BNO055_GYR_CONFIG_1_EFIS))
    {
        ESP_LOGE(TAG, "La configuracion IMU leida no coincide con la solicitada");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

static int16_t bno055_make_i16(uint8_t lsb, uint8_t msb)
{
    return (int16_t)(((uint16_t)msb << 8) | lsb);
}

/* -------------------------------------------------------------------------- */

static esp_err_t bno055_detect_once(void)
{
    const uint8_t addresses[] = {BNO055_ADDR_LOW, BNO055_ADDR_HIGH};

    for (size_t i = 0U; i < sizeof(addresses) / sizeof(addresses[0]); ++i)
    {
        const uint8_t address = addresses[i];

        esp_err_t err = efis_i2c_probe(address);

        if (err != ESP_OK)
        {
            ESP_LOGW(TAG,
                     "Sin ACK I2C en 0x%02X: %s",
                     address,
                     esp_err_to_name(err));
            continue;
        }

        if (s_bno055_dev != NULL)
        {
            i2c_master_bus_rm_device(s_bno055_dev);
            s_bno055_dev = NULL;
        }

        ESP_RETURN_ON_ERROR(
            efis_i2c_add_device(address, &s_bno055_dev),
            TAG,
            "No se pudo registrar BNO055 en el bus I2C");

        s_bno055_address = address;

        uint8_t chip_id = 0U;
        err = bno055_read_u8(BNO055_REG_CHIP_ID, &chip_id);

        if ((err == ESP_OK) && (chip_id == BNO055_CHIP_ID_VALUE))
        {
            ESP_LOGI(TAG,
                     "BNO055 detectado en 0x%02X; CHIP_ID=0x%02X",
                     s_bno055_address,
                     chip_id);
            return ESP_OK;
        }

        ESP_LOGW(TAG,
                 "Dispositivo en 0x%02X no es BNO055: err=%s CHIP_ID=0x%02X",
                 address,
                 esp_err_to_name(err),
                 chip_id);

        i2c_master_bus_rm_device(s_bno055_dev);
        s_bno055_dev = NULL;
    }

    return ESP_ERR_NOT_FOUND;
}

/* -------------------------------------------------------------------------- */

static esp_err_t bno055_detect(void)
{
    return bno055_detect_once();
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
        vTaskDelay(pdMS_TO_TICKS(BNO055_CONFIG_MODE_DELAY_MS));
    }
    else
    {
        vTaskDelay(pdMS_TO_TICKS(BNO055_OPERATION_MODE_DELAY_MS));
    }

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

static const char *bno055_mode_name(uint8_t mode)
{
    switch (mode)
    {
    case BNO055_MODE_AMG:
        return "AMG";

    case BNO055_MODE_IMU:
        return "IMUPLUS";

    case BNO055_MODE_NDOF:
        return "NDOF";

    case BNO055_MODE_NDOF_FMC_OFF:
        return "NDOF_FMC_OFF";

    case BNO055_MODE_CONFIG:
        return "CONFIG";

    default:
        return "UNKNOWN";
    }
}

/* -------------------------------------------------------------------------- */

static esp_err_t bno055_apply_operation_mode(uint8_t mode)
{
    /*
     * Todo cambio de modo se realiza pasando primero por CONFIGMODE.
     */
    ESP_RETURN_ON_ERROR(
        bno055_set_mode(BNO055_MODE_CONFIG),
        TAG,
        "No se pudo entrar en CONFIGMODE");

    /*
     * En AMG sí queremos imponer nuestra configuración de ACC/GYR.
     * En los modos de fusión dejamos que el BNO055 gestione internamente
     * los sensores.
     */
    if (mode == BNO055_MODE_AMG)
    {
        ESP_RETURN_ON_ERROR(
            bno055_configure_sensors(),
            TAG,
            "No se pudo configurar ACC/GYR para AMG");
    }

    ESP_RETURN_ON_ERROR(
        bno055_set_mode(mode),
        TAG,
        "No se pudo activar OPR_MODE");

    s_operation_mode_reg = mode;

    ESP_LOGI(
        TAG,
        "Modo BNO055 -> %s (OPR_MODE=0x%02X)",
        bno055_mode_name(mode),
        mode);

    return ESP_OK;
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
        return ESP_ERR_INVALID_ARG;

    uint8_t data[6];

    ESP_RETURN_ON_ERROR(bno055_read(first_register, data, sizeof(data)), TAG, "Error leyendo vector");

    vector->x = (float)bno055_make_i16(data[0], data[1]) / scale;
    vector->y = (float)bno055_make_i16(data[2], data[3]) / scale;
    vector->z = (float)bno055_make_i16(data[4], data[5]) / scale;

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* API interna del driver                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t bno055_driver_init(void)
{
    /*
     * El BNO055 puede necesitar varios segundos después del power-on antes de
     * aceptar de forma estable tanto lecturas como escrituras.
     */
    vTaskDelay(pdMS_TO_TICKS(BNO055_STARTUP_DELAY_MS));

    const TickType_t start_tick = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(BNO055_DETECT_TIMEOUT_MS);
    uint32_t attempt = 0U;
    esp_err_t err = ESP_ERR_NOT_FOUND;

    /*
     * No basta con leer CHIP_ID. En el hardware ensayado se ha observado que
     * CHIP_ID=0xA0 puede aparecer unos instantes antes de que OPR_MODE acepte
     * escrituras. Por ello consideramos "BNO listo" únicamente cuando:
     *
     *   1. CHIP_ID se lee correctamente.
     *   2. El dispositivo acepta la entrada en CONFIGMODE.
     */
    for (;;)
    {
        ++attempt;

        err = bno055_detect();

        if (err == ESP_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(BNO055_DETECT_SETTLE_MS));

            err = bno055_set_mode(BNO055_MODE_CONFIG);

            if (err == ESP_OK)
            {
                const uint32_t elapsed_ms = (uint32_t)((xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS);
                ESP_LOGI(TAG, "BNO055 listo tras %lu intento(s) (%lu ms)", (unsigned long)attempt, (unsigned long)elapsed_ms);
                break;
            }

            ESP_LOGW(TAG, "CHIP_ID valido pero CONFIGMODE no disponible: %s", esp_err_to_name(err));
        }

        const TickType_t elapsed_ticks = xTaskGetTickCount() - start_tick;

        if (elapsed_ticks >= timeout_ticks)
        {
            ESP_LOGE(TAG, "BNO055 no listo tras %u ms", (unsigned int)BNO055_DETECT_TIMEOUT_MS);

            return (err == ESP_OK) ? ESP_ERR_TIMEOUT : err;
        }

        ESP_LOGW(TAG, "BNO055 todavia no listo; intento %lu. Reintentando en %u ms...", (unsigned long)attempt, (unsigned int)BNO055_DETECT_RETRY_MS);

        vTaskDelay(pdMS_TO_TICKS(BNO055_DETECT_RETRY_MS));
    }

    /*
     * En AMG controlamos explícitamente los sensores. En NDOF dejamos que la
     * fusión interna gestione los parámetros que necesite.
     */
    if (s_operation_mode_reg == BNO055_MODE_AMG)
        ESP_RETURN_ON_ERROR(bno055_configure_sensors(), TAG, "No se pudo configurar acelerometro/giroscopo");

    ESP_RETURN_ON_ERROR(bno055_write_u8(BNO055_REG_PAGE_ID, 0x00U), TAG, "No se pudo seleccionar PAGE 0");

    ESP_RETURN_ON_ERROR(bno055_write_u8(BNO055_REG_PWR_MODE, BNO055_POWER_NORMAL), TAG, "No se pudo seleccionar POWER_NORMAL");

    vTaskDelay(pdMS_TO_TICKS(BNO055_POWER_MODE_DELAY_MS));

    ESP_RETURN_ON_ERROR(bno055_write_u8(BNO055_REG_UNIT_SEL, BNO055_UNIT_SEL_SI), TAG, "No se pudieron configurar las unidades");

    /* Arranque seguro en V. Si NVS contiene H, websocket.c lo aplicará después. */
    ESP_RETURN_ON_ERROR(bno055_apply_mount_mode(BNO055_MOUNT_VERTICAL), TAG, "No se pudo aplicar la orientacion V");

    ESP_RETURN_ON_ERROR(bno055_set_mode(s_operation_mode_reg), TAG, "No se pudo activar el modo de operacion");

    vTaskDelay(pdMS_TO_TICKS(BNO055_POST_INIT_DELAY_MS));

    uint8_t mode = 0U;
    uint8_t power = 0U;
    uint8_t units = 0U;

    ESP_RETURN_ON_ERROR(bno055_read_u8(BNO055_REG_OPR_MODE, &mode), TAG, "No se pudo verificar OPR_MODE");

    ESP_RETURN_ON_ERROR(bno055_read_u8(BNO055_REG_PWR_MODE, &power), TAG, "No se pudo verificar PWR_MODE");

    ESP_RETURN_ON_ERROR(bno055_read_u8(BNO055_REG_UNIT_SEL, &units), TAG, "No se pudo verificar UNIT_SEL");

    // OPR_MODE=0x07 -> AMG # OPR_MODE=0x0C -> NDOF
    ESP_LOGI(
        TAG,
        "Inicializado: %s | OPR_MODE=0x%02X, PWR_MODE=0x%02X, UNIT_SEL=0x%02X",
        bno055_mode_name(mode),
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
        bno055_set_mode(s_operation_mode_reg),
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

esp_err_t bno055_driver_set_operation_mode(
    bno055_operation_mode_t mode)
{
    uint8_t target_mode;

    if (mode == BNO055_OPERATION_MODE_AMG)
    {
        target_mode = BNO055_MODE_AMG;
    }
    else if (mode == BNO055_OPERATION_MODE_NDOF)
    {
        target_mode = BNO055_MODE_NDOF;
    }
    else
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (target_mode == s_operation_mode_reg)
        return ESP_OK;

    return bno055_apply_operation_mode(target_mode);
}

/* -------------------------------------------------------------------------- */

bno055_operation_mode_t bno055_driver_get_operation_mode(void)
{
    /*
     * API de compatibilidad con el EFIS original:
     * cualquier modo de fusión se considera lógicamente "NDOF".
     */
    return (s_operation_mode_reg == BNO055_MODE_AMG)
               ? BNO055_OPERATION_MODE_AMG
               : BNO055_OPERATION_MODE_NDOF;
}

/* -------------------------------------------------------------------------- */

uint8_t bno055_driver_get_mode_register(void)
{
    return s_operation_mode_reg;
}

/* -------------------------------------------------------------------------- */

const char *bno055_driver_get_mode_name(void)
{
    return bno055_mode_name(s_operation_mode_reg);
}

/* -------------------------------------------------------------------------- */

esp_err_t bno055_driver_cycle_test_mode(void)
{
    uint8_t next_mode;

    switch (s_operation_mode_reg)
    {
    case BNO055_MODE_AMG:
        next_mode = BNO055_MODE_IMU;
        break;

    case BNO055_MODE_IMU:
        next_mode = BNO055_MODE_NDOF;
        break;

    case BNO055_MODE_NDOF:
        next_mode = BNO055_MODE_NDOF_FMC_OFF;
        break;

    case BNO055_MODE_NDOF_FMC_OFF:
    default:
        next_mode = BNO055_MODE_AMG;
        break;
    }

    return bno055_apply_operation_mode(next_mode);
}

/* -------------------------------------------------------------------------- */

esp_err_t bno055_driver_read_acceleration(bno055_vector3f_t *acceleration_ms2)
{
    return bno055_read_vector_scaled(BNO055_REG_ACCEL_DATA_X_LSB, BNO055_ACCEL_LSB_PER_MS2, acceleration_ms2);
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
