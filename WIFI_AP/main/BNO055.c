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
#include "config.h"

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
#define BNO055_MODE_AMG 0x07U
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
static bool s_attitude_initialized = false;
static float s_pitch_est_deg = 0.0f;
static float s_roll_est_deg = 0.0f;

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
static uint8_t bno055_selected_operation_mode(void)
{
#if BNO055_USE_INTERNAL_FUSION
    return BNO055_MODE_NDOF;
#else
    return BNO055_MODE_AMG;
#endif
}

/* -------------------------------------------------------------------------- */
static void bno055_compute_accel_attitude(float ax, float ay, float az,
                                          float *roll_deg, float *pitch_deg)
{
    const float az2 = az * az;
    *pitch_deg = atan2f(ay, sqrtf(ax * ax + az2)) * (180.0f / (float)M_PI);
    *roll_deg = atan2f(ax, sqrtf(ay * ay + az2)) * (180.0f / (float)M_PI);
}

/* -------------------------------------------------------------------------- */
static void bno055_update_complementary_attitude(
    const bno055_vector3f_t *accel_ms2,
    float gyro_x_dps,
    float gyro_y_dps,
    float dt_s,
    float *roll_deg,
    float *pitch_deg)
{
    float roll_acc_deg = 0.0f;
    float pitch_acc_deg = 0.0f;

    bno055_compute_accel_attitude(
        accel_ms2->x, accel_ms2->y, accel_ms2->z,
        &roll_acc_deg, &pitch_acc_deg);

    if (!s_attitude_initialized)
    {
        s_roll_est_deg = roll_acc_deg;
        s_pitch_est_deg = pitch_acc_deg;
        s_attitude_initialized = true;
    }
    else
    {
        const float pitch_gyro_deg = s_pitch_est_deg + BNO055_PITCH_GYRO_SIGN * gyro_x_dps * dt_s;

        const float roll_gyro_deg = s_roll_est_deg + BNO055_ROLL_GYRO_SIGN * gyro_y_dps * dt_s;

        float alpha = BNO055_ATTITUDE_TAU_S / (BNO055_ATTITUDE_TAU_S + dt_s);
        if (!isfinite(alpha))
            alpha = 0.98f;
        if (alpha < 0.0f)
            alpha = 0.0f;
        else if (alpha > 1.0f)
            alpha = 1.0f;

        s_pitch_est_deg =
            alpha * pitch_gyro_deg + (1.0f - alpha) * pitch_acc_deg;

        s_roll_est_deg =
            alpha * roll_gyro_deg + (1.0f - alpha) * roll_acc_deg;
    }

    *pitch_deg = s_pitch_est_deg;
    *roll_deg = s_roll_est_deg;
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
        bno055_set_mode(bno055_selected_operation_mode()),
        TAG,
        "No se pudo activar el modo de operacion");

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
static float bno055_compute_turn_rate_from_attitude_dps(
    float gyro_x_dps,
    float gyro_y_dps,
    float gyro_z_dps,
    float roll_deg,
    float pitch_deg)
{
    /*
     * En AMG no existe gravity_ms2 fusionada. Reconstruimos la dirección de
     * la vertical local a partir del roll/pitch estimados por nuestro filtro
     * complementario.
     *
     * Las ecuaciones deben ser coherentes con bno055_compute_accel_attitude():
     *
     *     roll  = atan2(ax, sqrt(ay^2 + az^2))
     *     pitch = atan2(ay, sqrt(ax^2 + az^2))
     *
     * Por tanto, para el rango normal de actitud:
     *
     *     g_hat_x = sin(roll)
     *     g_hat_y = sin(pitch)
     *     g_hat_z = +sqrt(1 - g_hat_x^2 - g_hat_y^2)
     *
     * Finalmente:
     *
     *     omega_vertical = omega dot g_hat
     *
     * De este modo, un movimiento puro de pitch o roll no debería aparecer
     * como régimen de giro alrededor de la vertical local.
     */
    if (!isfinite(gyro_x_dps) ||
        !isfinite(gyro_y_dps) ||
        !isfinite(gyro_z_dps) ||
        !isfinite(roll_deg) ||
        !isfinite(pitch_deg))
    {
        return 0.0f;
    }

    const float roll_rad = roll_deg * DEG_TO_RAD;
    const float pitch_rad = pitch_deg * DEG_TO_RAD;

    const float gx = sinf(roll_rad);
    const float gy = sinf(pitch_rad);

    float gz2 = 1.0f - gx * gx - gy * gy;
    if (gz2 < 0.0f)
        gz2 = 0.0f;

    const float gz = sqrtf(gz2);

    float turn_rate_dps =
        gyro_x_dps * gx +
        gyro_y_dps * gy +
        gyro_z_dps * gz;

    /*
     * Misma convención gráfica que la rama NDOF:
     * giro a derechas -> bastón hacia la derecha.
     */
    turn_rate_dps = -turn_rate_dps;

    if (fabsf(turn_rate_dps) < TURN_RATE_DEADBAND_DPS)
        turn_rate_dps = 0.0f;

    return turn_rate_dps;
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

    const float gravity_norm = sqrtf(gx * gx + gy * gy + gz * gz);

    if (!isfinite(gravity_norm) || (gravity_norm < 1.0f))
        return 0.0f;

    /*
     * Proyección de la velocidad angular sobre la vertical local:
     *
     *     omega_vertical = omega dot g_hat
     *
     * Esto evita que cambios puros de pitch o roll generen, idealmente,
     * indicación de régimen de giro.
     */
    float turn_rate_dps = (gyro_x_dps * gx + gyro_y_dps * gy + gyro_z_dps * gz) / gravity_norm;

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
    float accel_x_g,
    float roll_deg,
    float dt_s)
{
    /*
     * Bola de resbale/deslizamiento en modo AMG.
     *
     * El acelerómetro mide aceleración específica total:
     *
     *     a_meas = g + a_linear
     *
     * Para la bola interesa la componente lateral NO gravitatoria. Con nuestro
     * convenio de ejes:
     *
     *     X = transversal (+ izquierda)
     *     Y = longitudinal (+ morro)
     *     Z = vertical (+ arriba)
     *
     * y usando la misma geometría empleada en el estimador de actitud:
     *
     *     g_x / g = sin(roll)
     *
     * Por tanto, en unidades de G:
     *
     *     a_lateral_g = accel_x_g - sin(roll)
     *
     * Esta magnitud equivale a la componente X de una aceleración lineal
     * reconstruida por software, similar a linear_acceleration_ms2.x que
     * proporcionaba el motor de fusión NDOF.
     *
     * Se convierte a un ángulo equivalente de bola mediante:
     *
     *     ball_angle = atan(a_lateral_g)
     *
     * de modo que:
     *     0.0 G ->  0.00 deg
     *     0.1 G ->  5.71 deg
     *     0.2 G -> 11.31 deg
     */
    if (!isfinite(accel_x_g) ||
        !isfinite(roll_deg))
    {
        return s_slip_ball_filtered_deg;
    }

    if (!isfinite(dt_s) || (dt_s <= 0.0f))
    {
        dt_s = (float)BNO055_PERIOD_MS / 1000.0f;
    }

    const float roll_rad =
        roll_deg * DEG_TO_RAD;

    const float gravity_x_g =
        sinf(roll_rad);

    const float lateral_accel_g =
        accel_x_g - gravity_x_g;

    float raw_ball_deg =
        atanf(lateral_accel_g) *
        (180.0f / (float)M_PI);

    /*
     * Convención gráfica actual.
     * Si en la prueba física el sentido queda invertido, basta con eliminar
     * o cambiar este signo.
     */
    raw_ball_deg = -raw_ball_deg;

    if (fabsf(raw_ball_deg) < SLIP_BALL_DEADBAND_DEG)
    {
        raw_ball_deg = 0.0f;
    }

    /*
     * Dinámica amortiguada del indicador.
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
        s_slip_ball_filtered_deg = SLIP_BALL_LIMIT_DEG;
    else if (s_slip_ball_filtered_deg < -SLIP_BALL_LIMIT_DEG)
        s_slip_ball_filtered_deg = -SLIP_BALL_LIMIT_DEG;

    return s_slip_ball_filtered_deg;
}

/* -------------------------------------------------------------------------- */

#define GYRO_FILTER_TAU_S 0.15f
static float s_gyro_x_filtered_dps = 0.0f;
static float s_gyro_y_filtered_dps = 0.0f;
static float s_gyro_z_filtered_dps = 0.0f;

static bool s_gyro_filter_initialized = false;

/* -------------------------------------------------------------------------- */
static void bno055_filter_gyro_dps(
    float gyro_x_dps,
    float gyro_y_dps,
    float gyro_z_dps,
    float dt_s,
    float *gyro_x_filtered_dps,
    float *gyro_y_filtered_dps,
    float *gyro_z_filtered_dps)
{
    if (!isfinite(gyro_x_dps) ||
        !isfinite(gyro_y_dps) ||
        !isfinite(gyro_z_dps))
    {
        return;
    }

    if (!isfinite(dt_s) || dt_s <= 0.0f)
    {
        dt_s = (float)BNO055_PERIOD_MS / 1000.0f;
    }

    /*
     * Primera muestra: inicializamos directamente con la medida
     * para evitar el transitorio de arranque desde cero.
     */
    if (!s_gyro_filter_initialized)
    {
        s_gyro_x_filtered_dps = gyro_x_dps;
        s_gyro_y_filtered_dps = gyro_y_dps;
        s_gyro_z_filtered_dps = gyro_z_dps;

        s_gyro_filter_initialized = true;
    }
    else
    {
        /*
         * Filtro paso bajo de primer orden:
         *
         * y[k] = y[k-1] + alpha * (x[k] - y[k-1])
         *
         * alpha = 1 - exp(-dt/tau)
         */
        const float alpha =
            1.0f - expf(-dt_s / GYRO_FILTER_TAU_S);

        s_gyro_x_filtered_dps +=
            alpha * (gyro_x_dps - s_gyro_x_filtered_dps);

        s_gyro_y_filtered_dps +=
            alpha * (gyro_y_dps - s_gyro_y_filtered_dps);

        s_gyro_z_filtered_dps +=
            alpha * (gyro_z_dps - s_gyro_z_filtered_dps);
    }

    *gyro_x_filtered_dps = s_gyro_x_filtered_dps;
    *gyro_y_filtered_dps = s_gyro_y_filtered_dps;
    *gyro_z_filtered_dps = s_gyro_z_filtered_dps;
}

/* -------------------------------------------------------------------------- */
/*
En el BNO055 tenemos tres magnitudes diferentes:

acceleration_ms2:        lectura del acelerómetro, es decir, aceleración específica total medida, que contiene el efecto de la gravedad
                         y de las aceleraciones debidas al movimiento del dispositivo.

gravity_ms2:             estimación del vector gravedad realizada por la fusión del BNO055.

linear_acceleration_ms2: estimación de la aceleración debida al movimiento, después de eliminar la componente de gravedad.
*/

static esp_err_t bno055_read_sample(bno055_data_t *sample, float dt_s)
{
    if (sample == NULL)
        return ESP_ERR_INVALID_ARG;

    bno055_data_t data = {0};

    ESP_RETURN_ON_ERROR(
        bno055_read_vector_scaled(BNO055_REG_ACCEL_DATA_X_LSB,
                                  BNO055_ACCEL_LSB_PER_MS2,
                                  &data.acceleration_ms2),
        TAG, "Error leyendo acceleration_ms2");

    ESP_RETURN_ON_ERROR(
        bno055_read_vector_scaled(BNO055_REG_MAG_DATA_X_LSB,
                                  BNO055_MAG_LSB_PER_UT,
                                  &data.magnetic_field_ut),
        TAG, "Error leyendo magnetometro");

    bno055_vector3f_t gyro = {0};

    ESP_RETURN_ON_ERROR(
        bno055_read_vector_scaled(BNO055_REG_GYRO_DATA_X_LSB,
                                  BNO055_GYRO_LSB_PER_DPS,
                                  &gyro),
        TAG, "Error leyendo giroscopo");

    data.gyro_x_dps = gyro.x;
    data.gyro_y_dps = gyro.y;
    data.gyro_z_dps = gyro.z;

#if BNO055_USE_INTERNAL_FUSION

    ESP_RETURN_ON_ERROR(
        bno055_read_euler(&data.heading_deg, &data.roll_deg, &data.pitch_deg),
        TAG, "Error leyendo Euler");

    ESP_RETURN_ON_ERROR(
        bno055_read_quaternion(&data.quaternion),
        TAG, "Error leyendo cuaternion");

    ESP_RETURN_ON_ERROR(
        bno055_read_vector_scaled(BNO055_REG_LINEAR_ACC_X_LSB,
                                  BNO055_ACCEL_LSB_PER_MS2,
                                  &data.linear_acceleration_ms2),
        TAG, "Error leyendo aceleracion lineal");

    ESP_RETURN_ON_ERROR(
        bno055_read_vector_scaled(BNO055_REG_GRAVITY_X_LSB,
                                  BNO055_ACCEL_LSB_PER_MS2,
                                  &data.gravity_ms2),
        TAG, "Error leyendo gravedad");

#else

    bno055_update_complementary_attitude(
        &data.acceleration_ms2,
        data.gyro_x_dps,
        data.gyro_y_dps,
        dt_s,
        &data.roll_deg,
        &data.pitch_deg);

    /* Campos conservados por compatibilidad; no son válidos en AMG. */
    data.heading_deg = 0.0f;
    data.quaternion = (bno055_quaternionf_t){0};
    data.linear_acceleration_ms2 = (bno055_vector3f_t){0};
    data.gravity_ms2 = (bno055_vector3f_t){0};

#endif

    uint8_t temperature_raw = 0U;
    ESP_RETURN_ON_ERROR(
        bno055_read_u8(BNO055_REG_TEMP, &temperature_raw),
        TAG, "Error leyendo temperatura");
    data.temperature_c = (int8_t)temperature_raw;

    ESP_RETURN_ON_ERROR(
        bno055_read_calibration(&data.calibration_system,
                                &data.calibration_gyro,
                                &data.calibration_accel,
                                &data.calibration_mag),
        TAG, "Error leyendo calibracion");

    data.accel_x_g = data.acceleration_ms2.x / STANDARD_GRAVITY_MS2;
    data.accel_y_g = data.acceleration_ms2.y / STANDARD_GRAVITY_MS2;
    data.accel_z_g = data.acceleration_ms2.z / STANDARD_GRAVITY_MS2;

    data.accel_total_g =
        sqrtf(data.accel_x_g * data.accel_x_g +
              data.accel_y_g * data.accel_y_g +
              data.accel_z_g * data.accel_z_g);

#if BNO055_USE_INTERNAL_FUSION
    const float yaw_rate_raw_dps =
        bno055_compute_vertical_turn_rate_dps(
            data.gyro_x_dps,
            data.gyro_y_dps,
            data.gyro_z_dps,
            &data.gravity_ms2);
#else
    /*
     * En AMG proyectamos el vector de velocidad angular sobre la vertical
     * local reconstruida con el roll/pitch de nuestro filtro complementario.
     * Así evitamos que cambios de actitud contaminen directamente el
     * indicador de giro.
     */

    float gyro_x_filtered_dps = 0.0f;
    float gyro_y_filtered_dps = 0.0f;
    float gyro_z_filtered_dps = 0.0f;

    bno055_filter_gyro_dps(
        data.gyro_x_dps,
        data.gyro_y_dps,
        data.gyro_z_dps,
        dt_s,
        &gyro_x_filtered_dps,
        &gyro_y_filtered_dps,
        &gyro_z_filtered_dps);

    const float yaw_rate_raw_dps =
        bno055_compute_turn_rate_from_attitude_dps(
            gyro_x_filtered_dps,
            gyro_y_filtered_dps,
            gyro_z_filtered_dps,
            data.roll_deg,
            data.pitch_deg);
#endif

    data.yaw_rate_dps =
        bno055_filter_yaw_rate_dps(yaw_rate_raw_dps, dt_s);

#if !BNO055_USE_INTERNAL_FUSION
    static float s_yaw_demo_deg = 0.0f;

    /* Si lo integramos, obtenemos heading ... esto sólo si no hay fusión*/
    s_yaw_demo_deg += data.yaw_rate_dps * dt_s;

    while (s_yaw_demo_deg >= 360.0f)
        s_yaw_demo_deg -= 360.0f;

    while (s_yaw_demo_deg < 0.0f)
        s_yaw_demo_deg += 360.0f;

    data.heading_deg = s_yaw_demo_deg;
#endif

    data.slip_ball_deg =
        bno055_compute_slip_ball_deg(
            data.accel_x_g,
            data.roll_deg,
            dt_s);

#define SLIP_BALL_GAIN 5.0f
    data.slip_ball_deg = SLIP_BALL_GAIN * data.slip_ball_deg;

    data.g_current = data.accel_total_g;
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
                    sample.g_max = sample.g_current;

                if (sample.g_current < sample.g_min)
                    sample.g_min = sample.g_current;
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
        err = bno055_set_mode(bno055_selected_operation_mode());

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
        s_attitude_initialized = false;
        s_pitch_est_deg = 0.0f;
        s_roll_est_deg = 0.0f;

        /* NUEVO */
        s_gyro_x_filtered_dps = 0.0f;
        s_gyro_y_filtered_dps = 0.0f;
        s_gyro_z_filtered_dps = 0.0f;
        s_gyro_filter_initialized = false;

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

    ESP_RETURN_ON_ERROR(bno055_init(), TAG, "No se pudo inicializar BNO055");

    /*
     * Estado inicial del G-meter.
     */
    portENTER_CRITICAL(&s_data_mux);
    s_data.g_current = 0.0f;
    s_data.g_max = 0.0f;
    s_data.g_min = 0.0f;
    s_data.valid = false;
    portEXIT_CRITICAL(&s_data_mux);

    s_attitude_initialized = false;
    s_pitch_est_deg = 0.0f;
    s_roll_est_deg = 0.0f;

    BaseType_t ok = xTaskCreate(BNO055Task, "bno055", 5120, NULL, 5, &s_bno055_task);

    if (ok != pdPASS)
    {
        s_bno055_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG,
             "BNO055 iniciado en %s a %u Hz",
#if BNO055_USE_INTERNAL_FUSION
             "NDOF",
#else
             "AMG",
#endif
             1000U / BNO055_PERIOD_MS);

    return ESP_OK;
}
