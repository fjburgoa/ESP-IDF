#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/i2c.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "BNO055.h"

/* -------------------------------------------------------------------------- */
/* Bus I2C                                                                    */
/* -------------------------------------------------------------------------- */

/*
 * El bus I2C se inicializa una sola vez en main.c y es compartido con BMP280.
 */
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_TIMEOUT_MS 100U

/* -------------------------------------------------------------------------- */
/* BNO055                                                                     */
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
#define BNO055_MODE_NDOF 0x0CU
#define BNO055_POWER_NORMAL 0x00U
#define BNO055_UNIT_SEL_SI 0x00U

/*
 * Posiciones de montaje del BNO055 (datasheet, sección Axis Remap).
 *
 * V = P1, orientación original/default del sensor.
 * H = P0, giro de 90° en el plano XY.
 *
 * P0 equivale a:
 *     X_aircraft = -Y_sensor
 *     Y_aircraft =  X_sensor
 *     Z_aircraft =  Z_sensor
 *
 * Al utilizar el remapeo interno del BNO055, la transformación se aplica
 * también a la solución de fusión NDOF (Euler/cuaternión), no sólo a las
 * lecturas del acelerómetro.
 */
#define BNO055_AXIS_CONFIG_VERTICAL 0x24U /* P1 */
#define BNO055_AXIS_SIGN_VERTICAL 0x00U

#define BNO055_AXIS_CONFIG_HORIZONTAL 0x21U /* P0 */
#define BNO055_AXIS_SIGN_HORIZONTAL 0x04U

#define BNO055_ACCEL_LSB_PER_MS2 100.0f
#define BNO055_MAG_LSB_PER_UT 16.0f
#define BNO055_GYRO_LSB_PER_DPS 16.0f
#define BNO055_EULER_LSB_PER_DEG 16.0f
#define BNO055_QUATERNION_LSB 16384.0f

/*
 * 20 ms = 50 Hz.
 * 40 ms = 25 Hz
 * El BNO055 realiza internamente la fusión NDOF; aquí únicamente se muestrea
 * la solución ya fusionada.
 */
#define BNO055_PERIOD_MS 40U

#define STANDARD_GRAVITY_MS2 9.80665f
#define DEG_TO_RAD 0.01745329251994329577f

/* -------------------------------------------------------------------------- */
/* Coordinador de viraje                                                      */
/* -------------------------------------------------------------------------- */

/*
 * Filtro paso bajo de primer orden del régimen de giro.
 * Se traslada aquí el suavizado que antes realizaba index.html:
 *
 *     alpha = 1 - exp(-dt / tau)
 *     y += alpha * (x - y)
 *
 * Se usa un filtrado lento, acorde con la dinámica de un coordinador de viraje.
 */
#define TURN_RATE_FILTER_TAU_S 0.75f
#define TURN_RATE_DEADBAND_DPS 0.10f

/* -------------------------------------------------------------------------- */
/* Bola del coordinador                                                       */
/* -------------------------------------------------------------------------- */

/*
 * La bola se calcula a partir del ángulo de la resultante de aceleración
 * específica en el plano lateral/vertical (Y-Z) mediante atan2().
 * Se añade una zona muerta y un filtro lento para reproducir una dinámica
 * amortiguada similar a la de un inclinómetro mecánico.
 */
#define SLIP_BALL_FILTER_TAU_S 1.5f
#define SLIP_BALL_LIMIT_DEG 25.0f
#define SLIP_BALL_DEADBAND_DEG 0.8f

/* -------------------------------------------------------------------------- */

static const char *TAG = "BNO055";

static uint8_t s_bno055_address = BNO055_ADDR_LOW;
static TaskHandle_t s_bno055_task = NULL;

static portMUX_TYPE s_data_mux = portMUX_INITIALIZER_UNLOCKED;
static bno055_data_t s_data = {0};

static float s_yaw_rate_filtered_dps = 0.0f;
static float s_slip_ball_filtered_deg = 0.0f;
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
    return bno055_read(reg, value, 1U);
}

/* -------------------------------------------------------------------------- */
static esp_err_t bno055_detect(void)
{
    const uint8_t addresses[] = {BNO055_ADDR_LOW, BNO055_ADDR_HIGH};

    for (size_t i = 0U; i < sizeof(addresses) / sizeof(addresses[0]); ++i)
    {
        uint8_t chip_id = 0U;

        s_bno055_address = addresses[i];

        esp_err_t err = bno055_read_u8(BNO055_REG_CHIP_ID, &chip_id);

        if ((err == ESP_OK) && (chip_id == BNO055_CHIP_ID_VALUE))
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
    ESP_RETURN_ON_ERROR(bno055_write_u8(BNO055_REG_OPR_MODE, mode), TAG, "No se pudo cambiar OPR_MODE");

    if (mode == BNO055_MODE_CONFIG)
    {
        vTaskDelay(pdMS_TO_TICKS(25U));
    }
    else
    {
        /*
         * Margen conservador para que el motor de fusión estabilice
         * la transición al nuevo modo.
         */
        vTaskDelay(pdMS_TO_TICKS(100U));
    }

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
static esp_err_t bno055_apply_mount_mode(bno055_mount_mode_t mode)
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
        bno055_write_u8(BNO055_REG_AXIS_MAP_CONFIG, map_config),
        TAG,
        "No se pudo configurar AXIS_MAP_CONFIG");

    ESP_RETURN_ON_ERROR(
        bno055_write_u8(BNO055_REG_AXIS_MAP_SIGN, map_sign),
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
static esp_err_t bno055_init(void)
{
    /*
     * El BNO055 puede necesitar varios cientos de milisegundos después
     * de la alimentación antes de responder de forma estable.
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
        bno055_write_u8(BNO055_REG_PAGE_ID, 0x00U),
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
     * Arranque seguro en V. Si NVS contiene H, websocket.c aplicará H
     * inmediatamente al recuperar la configuración persistente.
     */
    ESP_RETURN_ON_ERROR(
        bno055_apply_mount_mode(BNO055_MOUNT_VERTICAL),
        TAG,
        "No se pudo aplicar la orientacion V");

    ESP_RETURN_ON_ERROR(
        bno055_set_mode(BNO055_MODE_NDOF),
        TAG,
        "No se pudo activar NDOF");

    /*
     * Tiempo adicional de estabilización antes de iniciar la tarea.
     * Evita el timeout observado en la primera lectura tras cambiar a NDOF.
     */
    vTaskDelay(pdMS_TO_TICKS(500U));

    uint8_t mode = 0U;
    uint8_t power = 0U;
    uint8_t units = 0U;

    ESP_RETURN_ON_ERROR(
        bno055_read_u8(BNO055_REG_OPR_MODE, &mode),
        TAG,
        "No se pudo verificar OPR_MODE");

    ESP_RETURN_ON_ERROR(
        bno055_read_u8(BNO055_REG_PWR_MODE, &power),
        TAG,
        "No se pudo verificar PWR_MODE");

    ESP_RETURN_ON_ERROR(
        bno055_read_u8(BNO055_REG_UNIT_SEL, &units),
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
static int16_t bno055_make_i16(uint8_t lsb, uint8_t msb)
{
    return (int16_t)(((uint16_t)msb << 8) | lsb);
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

    esp_err_t err =
        bno055_read(first_register, data, sizeof(data));

    if (err != ESP_OK)
    {
        return err;
    }

    vector->x =
        (float)bno055_make_i16(data[0], data[1]) / scale;

    vector->y =
        (float)bno055_make_i16(data[2], data[3]) / scale;

    vector->z =
        (float)bno055_make_i16(data[4], data[5]) / scale;

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
static esp_err_t bno055_read_euler(
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

    esp_err_t err =
        bno055_read(
            BNO055_REG_EULER_H_LSB,
            data,
            sizeof(data));

    if (err != ESP_OK)
    {
        return err;
    }

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
static esp_err_t bno055_read_quaternion(
    bno055_quaternionf_t *quaternion)
{
    if (quaternion == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[8];

    esp_err_t err =
        bno055_read(
            BNO055_REG_QUATERNION_W_LSB,
            data,
            sizeof(data));

    if (err != ESP_OK)
    {
        return err;
    }

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
static esp_err_t bno055_read_calibration(
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

    esp_err_t err =
        bno055_read_u8(
            BNO055_REG_CALIB_STAT,
            &status);

    if (err != ESP_OK)
    {
        return err;
    }

    *system = (status >> 6) & 0x03U;
    *gyro = (status >> 4) & 0x03U;
    *accel = (status >> 2) & 0x03U;
    *mag = status & 0x03U;

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
static float bno055_compute_vertical_turn_rate_dps(
    float gyro_x_dps,
    float gyro_y_dps,
    float gyro_z_dps,
    const bno055_vector3f_t *gravity_ms2)
{
    if (gravity_ms2 == NULL)
    {
        return 0.0f;
    }

    const float gx = gravity_ms2->x;
    const float gy = gravity_ms2->y;
    const float gz = gravity_ms2->z;

    const float gravity_norm =
        sqrtf(gx * gx + gy * gy + gz * gz);

    if (!isfinite(gravity_norm) || (gravity_norm < 1.0f))
    {
        return 0.0f;
    }

    /*
     * Proyección de la velocidad angular sobre la vertical local:
     *
     *     omega_vertical = omega dot g_hat
     *
     * Esto evita que cambios puros de pitch o roll generen, idealmente,
     * indicación de régimen de giro.
     */
    float turn_rate_dps =
        (gyro_x_dps * gx +
         gyro_y_dps * gy +
         gyro_z_dps * gz) /
        gravity_norm;

    /*
     * Convención gráfica actual:
     * giro a derechas -> bastón hacia la derecha.
     */
    turn_rate_dps = -turn_rate_dps;

    if (fabsf(turn_rate_dps) < TURN_RATE_DEADBAND_DPS)
    {
        turn_rate_dps = 0.0f;
    }

    return turn_rate_dps;
}

/* -------------------------------------------------------------------------- */
static float bno055_filter_yaw_rate_dps(
    float yaw_rate_dps,
    float dt_s)
{
    float alpha =
        1.0f - expf(-dt_s / TURN_RATE_FILTER_TAU_S);

    if (!isfinite(alpha) || (alpha < 0.0f))
    {
        alpha = 0.0f;
    }
    else if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }

    s_yaw_rate_filtered_dps +=
        alpha *
        (yaw_rate_dps - s_yaw_rate_filtered_dps);

    return s_yaw_rate_filtered_dps;
}

/* -------------------------------------------------------------------------- */
static float bno055_compute_slip_ball_deg(
    float accel_y_g,
    float accel_z_g,
    float dt_s)
{
    /*
     * La bola de un inclinómetro responde a la dirección de la aceleración
     * específica resultante en el plano lateral/vertical del avión.
     *
     * En reposo y nivelado:
     *      accel_y ~= 0 g
     *      accel_z ~= +1 g
     *      ball_angle = 0 deg
     *
     * En un viraje coordinado, la resultante permanece alineada con el eje
     * vertical del avión y la bola debe permanecer centrada.
     *
     * atan2() evita utilizar una ganancia arbitraria y proporciona directamente
     * el ángulo físico de la resultante.
     */
    if (!isfinite(accel_y_g) || !isfinite(accel_z_g))
    {
        return s_slip_ball_filtered_deg;
    }

    const float resultant_g =
        sqrtf(accel_y_g * accel_y_g +
              accel_z_g * accel_z_g);

    if (resultant_g < 0.10f)
    {
        return s_slip_ball_filtered_deg;
    }

    float raw_ball_deg =
        atan2f(accel_y_g, accel_z_g) *
        (180.0f / (float)M_PI);

    /*
     * Convención gráfica.
     * Si durante la prueba física el sentido resulta invertido, basta con
     * cambiar el signo de raw_ball_deg aquí.
     */
    raw_ball_deg = -raw_ball_deg;

    if (fabsf(raw_ball_deg) < SLIP_BALL_DEADBAND_DEG)
    {
        raw_ball_deg = 0.0f;
    }

    /*
     * Dinámica lenta y amortiguada, similar a una bola real en fluido.
     */
    float alpha =
        1.0f - expf(-dt_s / SLIP_BALL_FILTER_TAU_S);

    if (!isfinite(alpha) || (alpha < 0.0f))
    {
        alpha = 0.0f;
    }
    else if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }

    s_slip_ball_filtered_deg +=
        alpha *
        (raw_ball_deg - s_slip_ball_filtered_deg);

    if (s_slip_ball_filtered_deg > SLIP_BALL_LIMIT_DEG)
    {
        s_slip_ball_filtered_deg = SLIP_BALL_LIMIT_DEG;
    }
    else if (s_slip_ball_filtered_deg < -SLIP_BALL_LIMIT_DEG)
    {
        s_slip_ball_filtered_deg = -SLIP_BALL_LIMIT_DEG;
    }

    return s_slip_ball_filtered_deg;
}

/* -------------------------------------------------------------------------- */
static esp_err_t bno055_read_sample(bno055_data_t *sample, float dt_s)
{
    if (sample == NULL)
        return ESP_ERR_INVALID_ARG;

    bno055_data_t data = {0};

    ESP_RETURN_ON_ERROR(
        bno055_read_vector_scaled(
            BNO055_REG_ACCEL_DATA_X_LSB,
            BNO055_ACCEL_LSB_PER_MS2,
            &data.acceleration_ms2),
        TAG,
        "Error leyendo aceleración");

    ESP_RETURN_ON_ERROR(
        bno055_read_vector_scaled(
            BNO055_REG_MAG_DATA_X_LSB,
            BNO055_MAG_LSB_PER_UT,
            &data.magnetic_field_ut),
        TAG,
        "Error leyendo magnetómetro");

    bno055_vector3f_t gyro = {0};

    ESP_RETURN_ON_ERROR(
        bno055_read_vector_scaled(
            BNO055_REG_GYRO_DATA_X_LSB,
            BNO055_GYRO_LSB_PER_DPS,
            &gyro),
        TAG,
        "Error leyendo giróscopo");

    data.gyro_x_dps = gyro.x;
    data.gyro_y_dps = gyro.y;
    data.gyro_z_dps = gyro.z;

    ESP_RETURN_ON_ERROR(
        bno055_read_euler(
            &data.heading_deg,
            &data.roll_deg,
            &data.pitch_deg),
        TAG,
        "Error leyendo Euler");

    ESP_RETURN_ON_ERROR(
        bno055_read_quaternion(
            &data.quaternion),
        TAG,
        "Error leyendo cuaternión");

    ESP_RETURN_ON_ERROR(
        bno055_read_vector_scaled(
            BNO055_REG_LINEAR_ACC_X_LSB,
            BNO055_ACCEL_LSB_PER_MS2,
            &data.linear_acceleration_ms2),
        TAG,
        "Error leyendo aceleración lineal");

    ESP_RETURN_ON_ERROR(
        bno055_read_vector_scaled(
            BNO055_REG_GRAVITY_X_LSB,
            BNO055_ACCEL_LSB_PER_MS2,
            &data.gravity_ms2),
        TAG,
        "Error leyendo gravedad");

    uint8_t temperature_raw = 0U;

    ESP_RETURN_ON_ERROR(
        bno055_read_u8(
            BNO055_REG_TEMP,
            &temperature_raw),
        TAG,
        "Error leyendo temperatura");

    data.temperature_c =
        (int8_t)temperature_raw;

    ESP_RETURN_ON_ERROR(
        bno055_read_calibration(
            &data.calibration_system,
            &data.calibration_gyro,
            &data.calibration_accel,
            &data.calibration_mag),
        TAG,
        "Error leyendo calibración");

    /*
     * Conversión a G para conservar las variables que ya consume
     * websocket.c / index.html.
     */
    data.accel_x_g =
        data.acceleration_ms2.x / STANDARD_GRAVITY_MS2;

    data.accel_y_g =
        data.acceleration_ms2.y / STANDARD_GRAVITY_MS2;

    data.accel_z_g =
        data.acceleration_ms2.z / STANDARD_GRAVITY_MS2;

    data.accel_total_g =
        sqrtf(
            data.accel_x_g * data.accel_x_g +
            data.accel_y_g * data.accel_y_g +
            data.accel_z_g * data.accel_z_g);

    const float yaw_rate_raw_dps =
        bno055_compute_vertical_turn_rate_dps(
            data.gyro_x_dps,
            data.gyro_y_dps,
            data.gyro_z_dps,
            &data.gravity_ms2);

    data.yaw_rate_dps =
        bno055_filter_yaw_rate_dps(
            yaw_rate_raw_dps,
            dt_s);

    data.slip_ball_deg =
        bno055_compute_slip_ball_deg(
            data.accel_y_g,
            data.accel_z_g,
            dt_s);

    /*
     * Magnitud total de aceleración específica expresada en G.
     *
     *     G = sqrt(Gx^2 + Gy^2 + Gz^2)
     *
     * Es independiente de la orientación del equipo. En reposo debe ser
     * aproximadamente 1.00 G.
     */
    data.g_current =
        sqrtf(data.accel_x_g * data.accel_x_g +
              data.accel_y_g * data.accel_y_g +
              data.accel_z_g * data.accel_z_g);

    data.valid = true;

    *sample = data;

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
static void BNO055Task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();

    const TickType_t period = pdMS_TO_TICKS(BNO055_PERIOD_MS);

    const float dt_s = (float)BNO055_PERIOD_MS / 1000.0f;

    for (;;)
    {
        bno055_data_t sample = {0};

        esp_err_t err = bno055_read_sample(&sample, dt_s);

        if (err == ESP_OK)
        {
            portENTER_CRITICAL(&s_data_mux);

            /*
             * Los picos pertenecen al estado persistente del driver.
             * En la primera muestra válida se inicializan ambos al valor actual;
             * después se retienen los extremos absolutos hasta el siguiente reset.
             */
            if (!s_data.valid)
            {
                sample.g_max = sample.g_current;
                sample.g_min = sample.g_current;
            }
            else
            {
                sample.g_max = s_data.g_max;
                sample.g_min = s_data.g_min;

                if (sample.g_current > sample.g_max)
                {
                    sample.g_max = sample.g_current;
                }

                if (sample.g_current < sample.g_min)
                {
                    sample.g_min = sample.g_current;
                }
            }

            s_data = sample;

            portEXIT_CRITICAL(&s_data_mux);

            ESP_LOGD(
                TAG,
                "H=%.2f R=%.2f P=%.2f | "
                "A=[%.3f %.3f %.3f] G | "
                "G=%.3f [%.3f %.3f] | "
                "CAL=%u/%u/%u/%u",
                (double)sample.heading_deg,
                (double)sample.roll_deg,
                (double)sample.pitch_deg,
                (double)sample.accel_x_g,
                (double)sample.accel_y_g,
                (double)sample.accel_z_g,
                (double)sample.g_current,
                (double)sample.g_min,
                (double)sample.g_max,
                sample.calibration_system,
                sample.calibration_gyro,
                sample.calibration_accel,
                sample.calibration_mag);
        }
        else
        {
            ESP_LOGW(TAG, "Lectura BNO055 fallida: %s", esp_err_to_name(err));
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

/* -------------------------------------------------------------------------- */
bno055_data_t BNO055_get_data(void)
{
    bno055_data_t snapshot;

    portENTER_CRITICAL(&s_data_mux);
    snapshot = s_data;
    portEXIT_CRITICAL(&s_data_mux);

    return snapshot;
}

/* -------------------------------------------------------------------------- */
esp_err_t BNO055_set_mount_mode(bno055_mount_mode_t mode)
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

    /*
     * Evita que la tarea periódica acceda por I2C mientras el BNO055 está
     * temporalmente en CONFIGMODE.
     */
    if (s_bno055_task != NULL)
    {
        vTaskSuspend(s_bno055_task);
    }

    esp_err_t err = bno055_set_mode(BNO055_MODE_CONFIG);

    if (err == ESP_OK)
        err = bno055_apply_mount_mode(mode);

    if (err == ESP_OK)
        err = bno055_set_mode(BNO055_MODE_NDOF);

    if (err == ESP_OK)
    {
        /*
         * Al cambiar el frame se invalidan la muestra anterior, los extremos
         * de G y los estados de los filtros para no mezclar ambos sistemas.
         */
        portENTER_CRITICAL(&s_data_mux);
        s_data.valid = false;
        s_data.g_current = 0.0f;
        s_data.g_max = 0.0f;
        s_data.g_min = 0.0f;
        portEXIT_CRITICAL(&s_data_mux);

        s_yaw_rate_filtered_dps = 0.0f;
        s_slip_ball_filtered_deg = 0.0f;

        vTaskDelay(pdMS_TO_TICKS(150U));
    }

    if (s_bno055_task != NULL)
        vTaskResume(s_bno055_task);

    return err;
}

/* -------------------------------------------------------------------------- */
bno055_mount_mode_t BNO055_get_mount_mode(void)
{
    return s_mount_mode;
}

/* -------------------------------------------------------------------------- */
void BNO055_reset_accel_peaks(void)
{
    portENTER_CRITICAL(&s_data_mux);

    /*
     * El reset no altera la medida instantánea. Reinicia la ventana de
     * máximos/mínimos tomando como origen la G actual.
     */
    s_data.g_max = s_data.g_current;
    s_data.g_min = s_data.g_current;

    portEXIT_CRITICAL(&s_data_mux);

    ESP_LOGI(TAG, "G-meter reseteado: min/max = G actual");
}

/* -------------------------------------------------------------------------- */
esp_err_t BNO055_start(void)
{
    if (s_bno055_task != NULL)
        return ESP_ERR_INVALID_STATE;

    ESP_RETURN_ON_ERROR(
        bno055_init(),
        TAG,
        "No se pudo inicializar BNO055");

    /*
     * Estado inicial del G-meter.
     */
    portENTER_CRITICAL(&s_data_mux);
    s_data.g_current = 0.0f;
    s_data.g_max = 0.0f;
    s_data.g_min = 0.0f;
    s_data.valid = false;
    portEXIT_CRITICAL(&s_data_mux);

    BaseType_t ok =
        xTaskCreate(
            BNO055Task,
            "bno055",
            5120,
            NULL,
            5,
            &s_bno055_task);

    if (ok != pdPASS)
    {
        s_bno055_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(
        TAG,
        "BNO055 iniciado en NDOF a %u Hz",
        1000U / BNO055_PERIOD_MS);

    return ESP_OK;
}
