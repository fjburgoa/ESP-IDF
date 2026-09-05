#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/i2c_master.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"

#include "MPU6050.h"
#include "EFIS_I2C.h"

#define MPU6050_ADDR 0x68
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_SMPLRT_DIV 0x19

#define MPU0 0U

/*
 * Para actitud conviene trabajar bastante más rápido que los 80 ms iniciales.
 * 20 ms = 50 Hz. Puede bajarse posteriormente a 10 ms = 100 Hz.
 */
#define MPU6050_PERIOD_MS 40U // esto habrá que pasarlo a config.h

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
// #define SLIP_BALL_FILTER_TAU_S 0.25f
// #define SLIP_BALL_GAIN_DEG_PER_G 120.0f
// #define SLIP_BALL_LIMIT_DEG 25.0f
// #define SLIP_BALL_DEADBAND_G 0.015f

static const char *TAG = "MPU6050";

static TaskHandle_t s_mpu_task = NULL;
static portMUX_TYPE s_data_mux = portMUX_INITIALIZER_UNLOCKED;
static i2c_master_dev_handle_t s_mpu6050_dev = NULL;

static mpu6050_data_t s_data = {0};

static float s_gyro_bias_x_raw = 0.0f;
static float s_gyro_bias_y_raw = 0.0f;
static float s_gyro_bias_z_raw = 0.0f;

static float s_acc_bias_x_raw = 0.0f;
static float s_acc_bias_y_raw = 0.0f;
static float s_acc_bias_z_raw = 0.0f;

/* Estado interno del filtro paso bajo de la bola. */

/*----------------------------------------------------------------------------*/
static esp_err_t mpu6050_register_write(uint8_t mpuid,
                                        uint8_t reg_addr,
                                        uint8_t data)
{
    if ((mpuid != MPU0) || (s_mpu6050_dev == NULL))
        return ESP_ERR_INVALID_STATE;

    const uint8_t write_buf[2] = {reg_addr, data};

    return i2c_master_transmit(
        s_mpu6050_dev,
        write_buf,
        sizeof(write_buf),
        EFIS_I2C_TIMEOUT_MS);
}

/*----------------------------------------------------------------------------*/
static esp_err_t mpu6050_register_read(uint8_t mpuid,
                                       uint8_t reg_addr,
                                       uint8_t *data,
                                       size_t len)
{
    if ((mpuid != MPU0) || (s_mpu6050_dev == NULL))
        return ESP_ERR_INVALID_STATE;

    if ((data == NULL) || (len == 0U))
        return ESP_ERR_INVALID_ARG;

    return i2c_master_transmit_receive(
        s_mpu6050_dev,
        &reg_addr,
        1U,
        data,
        len,
        EFIS_I2C_TIMEOUT_MS);
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
    if (s_mpu6050_dev == NULL)
    {
        ESP_RETURN_ON_ERROR(
            efis_i2c_add_device(MPU6050_ADDR, &s_mpu6050_dev),
            TAG,
            "No se pudo registrar MPU6050 en el bus I2C");
    }

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
static esp_err_t mpu6050_calibrate_accel(void)
{
    int64_t sum_x = 0;
    int64_t sum_y = 0;
    int64_t sum_z = 0;

    ESP_LOGI(TAG,
             "Calibrando acelerometro; mantenga el sensor inmovil y horizontal");

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
            "Error durante calibracion del acelerometro");

        sum_x += ax;
        sum_y += ay;
        sum_z += az;

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    const float mean_x = (float)sum_x / (float)CALIBRATION_SAMPLES;

    const float mean_y = (float)sum_y / (float)CALIBRATION_SAMPLES;

    const float mean_z = (float)sum_z / (float)CALIBRATION_SAMPLES;

    /*
     * Posicion de calibracion:
     *
     *      X = 0 g
     *      Y = 0 g
     *      Z = +1 g
     *
     * Para +/-2 g:
     *      1 g = ACCEL_LSB_PER_G = 16384 LSB
     */
    s_acc_bias_x_raw = mean_x;
    s_acc_bias_y_raw = mean_y;
    s_acc_bias_z_raw = mean_z - ACCEL_LSB_PER_G;

    ESP_LOGI(TAG,
             "Bias accel raw: X=%.1f, Y=%.1f, Z=%.1f",
             (double)s_acc_bias_x_raw,
             (double)s_acc_bias_y_raw,
             (double)s_acc_bias_z_raw);

    return ESP_OK;
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
static void MPU6050Task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(MPU6050_PERIOD_MS);

    ESP_ERROR_CHECK(mpu6050_calibrate_gyro());
    ESP_ERROR_CHECK(mpu6050_calibrate_accel());

    for (;;)
    {
        int16_t ax_raw = 0;
        int16_t ay_raw = 0;
        int16_t az_raw = 0;

        int16_t gx_raw = 0;
        int16_t gy_raw = 0;
        int16_t gz_raw = 0;

        esp_err_t err = mpu6050_read_accel_gyro(
            MPU0,
            &ax_raw, &ay_raw, &az_raw,
            &gx_raw, &gy_raw, &gz_raw);

        if (err == ESP_OK)
        {
            const float ax_g = 9.81 * ((float)ax_raw - s_acc_bias_x_raw) / ACCEL_LSB_PER_G;
            const float ay_g = 9.81 * ((float)ay_raw - s_acc_bias_y_raw) / ACCEL_LSB_PER_G;
            const float az_g = 9.81 * ((float)az_raw - s_acc_bias_z_raw) / ACCEL_LSB_PER_G;

            const float gx_dps = -((float)gy_raw - s_gyro_bias_y_raw) / GYRO_LSB_PER_DPS;
            const float gy_dps = -((float)gx_raw - s_gyro_bias_x_raw) / GYRO_LSB_PER_DPS;
            const float gz_dps = -((float)gz_raw - s_gyro_bias_z_raw) / GYRO_LSB_PER_DPS;

            /*
                        ESP_LOGI(
                            TAG,
                            "ACC=[%+.3f %+.3f %+.3f] ms2 | GYR=[%+.3f %+.3f %+.3f] deg/s",
                            (double)ax_g,
                            (double)ay_g,
                            (double)az_g,
                            (double)gx_dps,
                            (double)gy_dps,
                            (double)gz_dps);
            */

            portENTER_CRITICAL(&s_data_mux);

            s_data.accel_x_g = ax_g;
            s_data.accel_y_g = ay_g;
            s_data.accel_z_g = az_g;

            s_data.gyro_x_dps = gx_dps;
            s_data.gyro_y_dps = gy_dps;
            s_data.gyro_z_dps = gz_dps;

            s_data.valid = true;

            portEXIT_CRITICAL(&s_data_mux);
        }
        else
        {
            ESP_LOGW(TAG,
                     "Error leyendo MPU6050: %s",
                     esp_err_to_name(err));
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

/*----------------------------------------------------------------------------*/
esp_err_t MPU6050_start(void)
{
    if (s_mpu_task != NULL)
        return ESP_ERR_INVALID_STATE;

    ESP_RETURN_ON_ERROR(mpu6050_init(), TAG, "No se pudo inicializar MPU6050");

    BaseType_t ok = xTaskCreatePinnedToCore(MPU6050Task, "mpu6050", 4096, NULL, 5, &s_mpu_task, 1);

    if (ok != pdPASS)
    {
        s_mpu_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "MPU6050 iniciado a %u Hz", 1000U / MPU6050_PERIOD_MS);

    return ESP_OK;
}
