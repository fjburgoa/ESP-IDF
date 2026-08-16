#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/i2c.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "MPU6050.h"

#define MPU6050_ADDR 0x68
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_SMPLRT_DIV 0x19

#define I2C_MASTER_PORT I2C_NUM_0
#define MPU0 0U

/*
 * Para actitud conviene trabajar bastante más rápido que los 80 ms iniciales.
 * 20 ms = 50 Hz. Puede bajarse posteriormente a 10 ms = 100 Hz.
 */
#define MPU6050_PERIOD_MS 20U

#define MPU6050_ACCEL_RANGE_G 2U
#define MPU6050_GYRO_RANGE_DPS 250U

#define ACCEL_LSB_PER_G 16384.0f
#define GYRO_LSB_PER_DPS 131.0f

#define COMPLEMENTARY_ALPHA 0.98f
#define RAD_TO_DEG 57.29577951308232f

#define CALIBRATION_SAMPLES 300U

/*
 * Bola del coordinador de viraje.
 *
 * La aceleración lateral se expresa inicialmente en G, se compensa con la
 * proyección de gravedad estimada a partir de roll/pitch y se filtra antes
 * de convertirla a desplazamiento angular del instrumento.
 */
#define SLIP_BALL_FILTER_TAU_S 0.25f
#define SLIP_BALL_GAIN_DEG_PER_G 120.0f
#define SLIP_BALL_LIMIT_DEG 25.0f
#define SLIP_BALL_DEADBAND_G 0.015f

static const char *TAG = "MPU6050";

static TaskHandle_t s_mpu_task = NULL;
static portMUX_TYPE s_data_mux = portMUX_INITIALIZER_UNLOCKED;

static mpu6050_data_t s_data = {0};

static float s_roll_deg = 0.0f;
static float s_pitch_deg = 0.0f;
static bool s_attitude_initialized = false;

static float s_gyro_bias_x_raw = 0.0f;
static float s_gyro_bias_y_raw = 0.0f;
static float s_gyro_bias_z_raw = 0.0f;

/* Estado interno del filtro paso bajo de la bola. */
static float s_slip_ball_filtered_g = 0.0f;

/*----------------------------------------------------------------------------*/
static esp_err_t mpu6050_register_write(uint8_t mpuid,
                                        uint8_t reg_addr,
                                        uint8_t data)
{
    const uint8_t write_buf[2] = {reg_addr, data};

    return i2c_master_write_to_device(
        I2C_MASTER_PORT,
        MPU6050_ADDR + mpuid,
        write_buf,
        sizeof(write_buf),
        pdMS_TO_TICKS(100));
}

/*----------------------------------------------------------------------------*/
static esp_err_t mpu6050_register_read(uint8_t mpuid,
                                       uint8_t reg_addr,
                                       uint8_t *data,
                                       size_t len)
{
    return i2c_master_write_read_device(
        I2C_MASTER_PORT,
        MPU6050_ADDR + mpuid,
        &reg_addr,
        1,
        data,
        len,
        pdMS_TO_TICKS(100));
}

/*----------------------------------------------------------------------------*/
static esp_err_t mpu6050_read_accel_gyro(uint8_t mpuid,
                                         int16_t *accel_x,
                                         int16_t *accel_y,
                                         int16_t *accel_z,
                                         int16_t *gyro_x,
                                         int16_t *gyro_y,
                                         int16_t *gyro_z)
{
    uint8_t data[14];

    ESP_RETURN_ON_ERROR(
        mpu6050_register_read(
            mpuid,
            MPU6050_ACCEL_XOUT_H,
            data,
            sizeof(data)),
        TAG,
        "Error leyendo MPU6050");

    *accel_x = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    *accel_y = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    *accel_z = (int16_t)(((uint16_t)data[4] << 8) | data[5]);

    *gyro_x = (int16_t)(((uint16_t)data[8] << 8) | data[9]);
    *gyro_y = (int16_t)(((uint16_t)data[10] << 8) | data[11]);
    *gyro_z = (int16_t)(((uint16_t)data[12] << 8) | data[13]);

    return ESP_OK;
}

/*----------------------------------------------------------------------------*/
static esp_err_t mpu6050_set_accel_scale(uint8_t mpuid, uint8_t scale_g)
{
    uint8_t value;

    switch (scale_g)
    {
    case 2:
        value = 0x00;
        break;
    case 4:
        value = 0x08;
        break;
    case 8:
        value = 0x10;
        break;
    case 16:
        value = 0x18;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    return mpu6050_register_write(mpuid, MPU6050_ACCEL_CONFIG, value);
}

/*----------------------------------------------------------------------------*/
static esp_err_t mpu6050_set_gyro_scale(uint8_t mpuid, uint16_t scale_dps)
{
    uint8_t value;

    switch (scale_dps)
    {
    case 250:
        value = 0x00;
        break;
    case 500:
        value = 0x08;
        break;
    case 1000:
        value = 0x10;
        break;
    case 2000:
        value = 0x18;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    return mpu6050_register_write(mpuid, MPU6050_GYRO_CONFIG, value);
}

/*----------------------------------------------------------------------------*/
static esp_err_t mpu6050_set_dlpf(uint8_t mpuid, uint8_t dlpf_cfg)
{
    if (dlpf_cfg > 6U)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return mpu6050_register_write(mpuid, MPU6050_CONFIG, dlpf_cfg);
}

/*----------------------------------------------------------------------------*/
static esp_err_t mpu6050_set_sample_rate(uint8_t mpuid, uint16_t rate_hz)
{
    if ((rate_hz < 4U) || (rate_hz > 1000U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t divider = (uint8_t)((1000U / rate_hz) - 1U);

    return mpu6050_register_write(mpuid, MPU6050_SMPLRT_DIV, divider);
}

/*----------------------------------------------------------------------------*/
static esp_err_t mpu6050_init(void)
{
    ESP_RETURN_ON_ERROR(
        mpu6050_register_write(MPU0, MPU6050_PWR_MGMT_1, 0x00),
        TAG,
        "No se pudo despertar el MPU6050");

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_RETURN_ON_ERROR(
        mpu6050_set_sample_rate(MPU0, 100U),
        TAG,
        "Error configurando sample rate");

    ESP_RETURN_ON_ERROR(
        mpu6050_set_gyro_scale(MPU0, MPU6050_GYRO_RANGE_DPS),
        TAG,
        "Error configurando giroscopio");

    ESP_RETURN_ON_ERROR(
        mpu6050_set_accel_scale(MPU0, MPU6050_ACCEL_RANGE_G),
        TAG,
        "Error configurando acelerómetro");

    /*
     * DLPF_CFG = 3:
     * acelerómetro ~44 Hz, giroscopio ~42 Hz.
     * Es más apropiado para actitud que 5 Hz.
     */
    ESP_RETURN_ON_ERROR(
        mpu6050_set_dlpf(MPU0, 3U),
        TAG,
        "Error configurando DLPF");

    return ESP_OK;
}

/*----------------------------------------------------------------------------*/
static esp_err_t mpu6050_calibrate_gyro(void)
{
    int64_t sum_x = 0;
    int64_t sum_y = 0;
    int64_t sum_z = 0;

    ESP_LOGI(TAG, "Calibrando giroscopio; mantenga el sensor inmóvil");

    for (uint32_t i = 0; i < CALIBRATION_SAMPLES; ++i)
    {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;

        ESP_RETURN_ON_ERROR(
            mpu6050_read_accel_gyro(
                MPU0,
                &ax, &ay, &az,
                &gx, &gy, &gz),
            TAG,
            "Error durante calibración");

        sum_x += gx;
        sum_y += gy;
        sum_z += gz;

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    s_gyro_bias_x_raw = (float)sum_x / (float)CALIBRATION_SAMPLES;
    s_gyro_bias_y_raw = (float)sum_y / (float)CALIBRATION_SAMPLES;
    s_gyro_bias_z_raw = (float)sum_z / (float)CALIBRATION_SAMPLES;

    ESP_LOGI(TAG,
             "Bias gyro raw: X=%.1f, Y=%.1f, Z=%.1f",
             (double)s_gyro_bias_x_raw,
             (double)s_gyro_bias_y_raw,
             (double)s_gyro_bias_z_raw);

    return ESP_OK;
}

/*----------------------------------------------------------------------------*/
void MPU6050_compute_roll_pitch(float ax_g,
                                float ay_g,
                                float az_g,
                                float gx_dps,
                                float gy_dps,
                                float dt_s,
                                float *roll_deg,
                                float *pitch_deg)
{
    if ((roll_deg == NULL) || (pitch_deg == NULL))
    {
        return;
    }

    /*
     * Ángulos calculados exclusivamente con la gravedad.
     * Esta convención supone:
     *   X: hacia delante
     *   Y: hacia la derecha
     *   Z: hacia arriba o abajo según el montaje.
     *
     * Si el módulo está montado con otra orientación habrá que permutar
     * ejes o cambiar signos.
     */
    const float roll_acc_deg =
        atan2f(ay_g, az_g) * RAD_TO_DEG;

    const float pitch_acc_deg =
        atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) *
        RAD_TO_DEG;

    if (!s_attitude_initialized ||
        !isfinite(s_roll_deg) ||
        !isfinite(s_pitch_deg))
    {
        s_roll_deg = roll_acc_deg;
        s_pitch_deg = pitch_acc_deg;
        s_attitude_initialized = true;
    }
    else
    {
        if ((dt_s <= 0.0f) || (dt_s > 0.5f))
        {
            dt_s = (float)MPU6050_PERIOD_MS / 1000.0f;
        }

        const float roll_gyro_deg = s_roll_deg + gx_dps * dt_s;
        const float pitch_gyro_deg = s_pitch_deg + gy_dps * dt_s;

        s_roll_deg = COMPLEMENTARY_ALPHA * roll_gyro_deg + (1.0f - COMPLEMENTARY_ALPHA) * roll_acc_deg;

        s_pitch_deg = COMPLEMENTARY_ALPHA * pitch_gyro_deg + (1.0f - COMPLEMENTARY_ALPHA) * pitch_acc_deg;
    }

    *roll_deg = s_roll_deg;
    *pitch_deg = s_pitch_deg;
}

/*----------------------------------------------------------------------------*/
mpu6050_data_t MPU6050_get_data(void)
{
    // esta función la usa websocket.c
    mpu6050_data_t snapshot;

    portENTER_CRITICAL(&s_data_mux);
    snapshot = s_data;
    portEXIT_CRITICAL(&s_data_mux);

    return snapshot;
}

/*----------------------------------------------------------------------------*/
void MPU6050_reset_accel_peaks(void)
{
    portENTER_CRITICAL(&s_data_mux);

    s_data.g_current = 0.0f;
    s_data.g_max = 0.0f;
    s_data.g_min = 0.0f;

    portEXIT_CRITICAL(&s_data_mux);

    ESP_LOGI(TAG, "G-meter reseteado: current=0, max=0, min=0");
}

/*----------------------------------------------------------------------------*/
static void MPU6050Task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(MPU6050_PERIOD_MS);

    ESP_ERROR_CHECK(mpu6050_calibrate_gyro());

    int64_t previous_time_us = esp_timer_get_time();

    for (;;)
    {
        int16_t ax_raw, ay_raw, az_raw;
        int16_t gx_raw, gy_raw, gz_raw;

        if (mpu6050_read_accel_gyro(
                MPU0,
                &ax_raw, &ay_raw, &az_raw,
                &gx_raw, &gy_raw, &gz_raw) == ESP_OK)
        {
            const int64_t now_us = esp_timer_get_time();
            const float dt_s = (float)(now_us - previous_time_us) / 1000000.0f;
            previous_time_us = now_us;

            const float ax_g = (float)ay_raw / ACCEL_LSB_PER_G;
            const float ay_g = (float)ax_raw / ACCEL_LSB_PER_G;
            const float az_g = (float)az_raw / ACCEL_LSB_PER_G;

            const float gx_dps = -((float)gy_raw - s_gyro_bias_y_raw) / GYRO_LSB_PER_DPS;
            const float gy_dps = -((float)gx_raw - s_gyro_bias_x_raw) / GYRO_LSB_PER_DPS;
            const float gz_dps = -((float)gz_raw - s_gyro_bias_z_raw) / GYRO_LSB_PER_DPS;

            float roll_deg;
            float pitch_deg;

            MPU6050_compute_roll_pitch(
                ax_g,
                ay_g,
                az_g,
                gx_dps,
                gy_dps,
                dt_s,
                &roll_deg,
                &pitch_deg);

            const float accel_total_g = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

            const float roll_rad = roll_deg / RAD_TO_DEG;
            const float pitch_rad = pitch_deg / RAD_TO_DEG;

            const float cos_pitch = cosf(pitch_rad);
            float yaw_rate_dps = 0.0f;

            /*
             * Conversión de velocidades angulares del cuerpo:
             *
             *   p = gx_dps
             *   q = gy_dps
             *   r = gz_dps
             *
             * a velocidad de cambio del ángulo de guiñada:
             *
             *   yaw_rate = (q·sin(roll) + r·cos(roll)) / cos(pitch)
             *
             * Se evita la división cerca de pitch = ±90°, donde la representación
             * mediante ángulos de Euler presenta una singularidad.
             */
            if (fabsf(cos_pitch) > 0.1f)
            {
                yaw_rate_dps =
                    (gy_dps * sinf(roll_rad) +
                     gz_dps * cosf(roll_rad)) /
                    cos_pitch;
            }

            /*
             * Bola del coordinador de viraje.
             *
             * Convención usada:
             *   X: longitudinal
             *   Y: lateral, positivo hacia la derecha
             *   Z: vertical, positivo hacia arriba
             *
             * En una inclinación estática, la gravedad proyectada sobre Y es:
             *
             *   ay_gravity = sin(roll) * cos(pitch)
             *
             * Se resta esa componente para que una inclinación del conjunto,
             * sin aceleración lateral real, no desplace la bola.
             */
            float lateral_g =
                ay_g - sinf(roll_rad) * cosf(pitch_rad);

            /*
             * Pequeña zona muerta antes del filtrado para eliminar bias y
             * vibraciones de muy baja amplitud.
             */
            if (fabsf(lateral_g) < SLIP_BALL_DEADBAND_G)
            {
                lateral_g = 0.0f;
            }

            /*
             * Filtro paso bajo de primer orden:
             *
             *   alpha = dt / (tau + dt)
             *
             * tau = 0,60 s proporciona un movimiento amortiguado similar
             * al de una bola física dentro de un tubo curvo.
             */
            float slip_alpha =
                dt_s / (SLIP_BALL_FILTER_TAU_S + dt_s);

            if ((slip_alpha < 0.0f) || !isfinite(slip_alpha))
            {
                slip_alpha = 0.0f;
            }
            else if (slip_alpha > 1.0f)
            {
                slip_alpha = 1.0f;
            }

            s_slip_ball_filtered_g +=
                slip_alpha *
                (lateral_g - s_slip_ball_filtered_g);

            float slip_ball_deg =
                s_slip_ball_filtered_g * SLIP_BALL_GAIN_DEG_PER_G;

            if (slip_ball_deg > SLIP_BALL_LIMIT_DEG)
            {
                slip_ball_deg = SLIP_BALL_LIMIT_DEG;
            }
            else if (slip_ball_deg < -SLIP_BALL_LIMIT_DEG)
            {
                slip_ball_deg = -SLIP_BALL_LIMIT_DEG;
            }

            portENTER_CRITICAL(&s_data_mux);

            s_data.accel_x_g = ax_g;
            s_data.accel_y_g = ay_g;
            s_data.accel_z_g = az_g;

            s_data.gyro_x_dps = gx_dps;
            s_data.gyro_y_dps = gy_dps;
            s_data.gyro_z_dps = gz_dps;
            s_data.yaw_rate_dps = yaw_rate_dps;

            /* Posición angular de la bola que consume la interfaz web. */
            s_data.slip_ball_deg = slip_ball_deg;

            s_data.roll_deg = roll_deg;
            s_data.pitch_deg = pitch_deg;
            s_data.accel_total_g = accel_total_g;

            /*
             * El G-meter elimina la componente estática de gravedad.
             * Esta expresión supone que, en reposo, el eje Z mide +1 g.
             */
            const float g_current = az_g - 1.0f;

            s_data.g_current = g_current;

            if (g_current > s_data.g_max)
            {
                s_data.g_max = g_current;
            }

            if (g_current < s_data.g_min)
            {
                s_data.g_min = g_current;
            }

            s_data.valid = true;

            portEXIT_CRITICAL(&s_data_mux);

            ESP_LOGD(TAG,
                     "Roll=%.2f Pitch=%.2f | A=[%.3f %.3f %.3f] g | "
                     "G current=%.3f max=%.3f min=%.3f | "
                     "slip=%.3f g / %.2f deg",
                     (double)roll_deg,
                     (double)pitch_deg,
                     (double)ax_g,
                     (double)ay_g,
                     (double)az_g,
                     (double)g_current,
                     (double)s_data.g_max,
                     (double)s_data.g_min,
                     (double)lateral_g,
                     (double)slip_ball_deg);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

/*----------------------------------------------------------------------------*/
esp_err_t MPU6050_start(void)
{
    if (s_mpu_task != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(
        mpu6050_init(),
        TAG,
        "No se pudo inicializar MPU6050");

    BaseType_t ok = xTaskCreate(
        MPU6050Task,
        "mpu6050",
        4096,
        NULL,
        5,
        &s_mpu_task);

    if (ok != pdPASS)
    {
        s_mpu_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "MPU6050 iniciado a %u Hz", 1000U / MPU6050_PERIOD_MS);

    return ESP_OK;
}
