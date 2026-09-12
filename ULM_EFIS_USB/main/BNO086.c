/**
 * @file BNO086.c
 * @brief Procesado de alto nivel del BNO086 para EFIS_USB.
 */

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "BNO086.h"
#include "BNO086_driver.h"
#include "config.h"
#include "settings.h"

#define RAD_TO_DEG 57.29577951308232f

static const char *TAG = "BNO086";

static TaskHandle_t s_task = NULL;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static bno086_data_t s_data;

/* -------------------------------------------------------------------------- */
/* Utilidades                                                                  */
/* -------------------------------------------------------------------------- */

static int16_t read_i16_le(const uint8_t *p)
{
    return (int16_t)(
        (uint16_t)p[0] |
        ((uint16_t)p[1] << 8));
}

/*
 * Transformación de aceleraciones validada en la placa larga:
 *
 * Vertical:
 *     X_avion = -Y_BNO086
 *     Y_avion = -X_BNO086
 *     Z_avion = -Z_BNO086
 *
 * Horizontal:
 *     se mantiene la misma convención que utiliza actualmente el BNO055.
 */
static void acceleration_to_aircraft_axes(
    const bno086_vector3f_t *raw,
    bno086_vector3f_t *aircraft)
{
    if (s_mount_mode == MOUNT_VERTICAL)
    {
        aircraft->x = -raw->y;
        aircraft->y = -raw->x;
        aircraft->z = -raw->z;
    }
    else
    {
        aircraft->x = -raw->x;
        aircraft->y = raw->y;
        aircraft->z = -raw->z;
    }
}

/*
 * Transformación de velocidades angulares validada en la placa larga:
 *
 * Vertical:
 *     p = +Y_BNO086
 *     q = +X_BNO086
 *     r = -Z_BNO086
 *
 * con:
 *     p>0 -> baja el ala derecha
 *     q>0 -> sube el morro
 *     r>0 -> gira el morro a la derecha
 */
static void gyro_to_aircraft_axes(
    const bno086_vector3f_t *raw,
    bno086_vector3f_t *aircraft)
{
    if (s_mount_mode == MOUNT_VERTICAL)
    {
        aircraft->x = raw->y;
        aircraft->y = raw->x;
        aircraft->z = -raw->z;
    }
    else
    {
        aircraft->x = raw->x;
        aircraft->y = -raw->y;
        aircraft->z = -raw->z;
    }
}

/*
 * Para el quaternion se utiliza el cambio de ejes propio de las velocidades
 * angulares. En vertical:
 *
 *     X_avion = +Y_sensor
 *     Y_avion = +X_sensor
 *     Z_avion = -Z_sensor
 *
 * Esta transformación tiene determinante +1, por lo que puede aplicarse a
 * la parte vectorial del quaternion conservando su componente real.
 *
 * Roll/Pitch/Yaw del Game Rotation Vector se validarán ahora en el EFIS.
 */
static void quaternion_to_aircraft_axes(
    const bno086_quaternionf_t *raw,
    bno086_quaternionf_t *aircraft)
{
    aircraft->w = raw->w;

    if (s_mount_mode == MOUNT_VERTICAL)
    {
        aircraft->x = raw->y;
        aircraft->y = raw->x;
        aircraft->z = -raw->z;
    }
    else
    {
        aircraft->x = raw->x;
        aircraft->y = -raw->y;
        aircraft->z = -raw->z;
    }

    const float norm = sqrtf(
        aircraft->w * aircraft->w +
        aircraft->x * aircraft->x +
        aircraft->y * aircraft->y +
        aircraft->z * aircraft->z);

    if (norm > 0.0f)
    {
        aircraft->w /= norm;
        aircraft->x /= norm;
        aircraft->y /= norm;
        aircraft->z /= norm;
    }
}

static void quaternion_to_euler(
    const bno086_quaternionf_t *q,
    float *roll_deg,
    float *pitch_deg,
    float *yaw_deg)
{
    /*
     * Convención Tait-Bryan Z-Y-X:
     * roll  alrededor de X
     * pitch alrededor de Y
     * yaw   alrededor de Z
     */
    const float sin_roll_cos_pitch =
        2.0f * (q->w * q->x + q->y * q->z);

    const float cos_roll_cos_pitch =
        1.0f - 2.0f * (q->x * q->x + q->y * q->y);

    const float roll_rad =
        atan2f(
            sin_roll_cos_pitch,
            cos_roll_cos_pitch);

    float sin_pitch =
        2.0f * (q->w * q->y - q->z * q->x);

    if (sin_pitch > 1.0f)
    {
        sin_pitch = 1.0f;
    }
    else if (sin_pitch < -1.0f)
    {
        sin_pitch = -1.0f;
    }

    const float pitch_rad = asinf(sin_pitch);

    const float sin_yaw_cos_pitch =
        2.0f * (q->w * q->z + q->x * q->y);

    const float cos_yaw_cos_pitch =
        1.0f - 2.0f * (q->y * q->y + q->z * q->z);

    const float yaw_rad =
        atan2f(
            sin_yaw_cos_pitch,
            cos_yaw_cos_pitch);

    *roll_deg = roll_rad * RAD_TO_DEG;
    *pitch_deg = pitch_rad * RAD_TO_DEG;
    *yaw_deg = yaw_rad * RAD_TO_DEG;
}

/* -------------------------------------------------------------------------- */
/* Procesado SH-2                                                              */
/* -------------------------------------------------------------------------- */

static size_t bno086_report_size(uint8_t report_id)
{
    switch (report_id)
    {
        case BNO086_REPORT_ACCELEROMETER:
        case BNO086_REPORT_GYROSCOPE:
        case BNO086_REPORT_LINEAR_ACCELERATION:
        case BNO086_REPORT_GRAVITY:
            return 10U;

        case BNO086_REPORT_GAME_ROTATION_VECTOR:
            return 12U;

        default:
            return 0U;
    }
}

static void process_sensor_report(
    bno086_data_t *sample,
    const uint8_t *report)
{
    const uint8_t report_id = report[0];
    const uint8_t accuracy = report[2] & 0x03U;

    if ((report_id == BNO086_REPORT_ACCELEROMETER) ||
        (report_id == BNO086_REPORT_GYROSCOPE) ||
        (report_id == BNO086_REPORT_LINEAR_ACCELERATION) ||
        (report_id == BNO086_REPORT_GRAVITY))
    {
        const int16_t raw_x = read_i16_le(&report[4]);
        const int16_t raw_y = read_i16_le(&report[6]);
        const int16_t raw_z = read_i16_le(&report[8]);

        if (report_id == BNO086_REPORT_ACCELEROMETER)
        {
            const bno086_vector3f_t scaled = {
                .x = (float)raw_x / 256.0f,
                .y = (float)raw_y / 256.0f,
                .z = (float)raw_z / 256.0f,
            };

            acceleration_to_aircraft_axes(
                &scaled,
                &sample->acceleration_ms2);

            sample->acceleration_accuracy = accuracy;
            sample->acceleration_valid = true;
        }
        else if (report_id == BNO086_REPORT_GYROSCOPE)
        {
            const bno086_vector3f_t scaled = {
                .x = (float)raw_x / 512.0f * RAD_TO_DEG,
                .y = (float)raw_y / 512.0f * RAD_TO_DEG,
                .z = (float)raw_z / 512.0f * RAD_TO_DEG,
            };

            gyro_to_aircraft_axes(
                &scaled,
                &sample->gyro_dps);

            sample->gyro_accuracy = accuracy;
            sample->gyro_valid = true;
        }
        else if (report_id == BNO086_REPORT_LINEAR_ACCELERATION)
        {
            const bno086_vector3f_t scaled = {
                .x = (float)raw_x / 256.0f,
                .y = (float)raw_y / 256.0f,
                .z = (float)raw_z / 256.0f,
            };

            acceleration_to_aircraft_axes(
                &scaled,
                &sample->linear_accel_ms2);

            sample->linear_accel_accuracy = accuracy;
            sample->linear_accel_valid = true;
        }
        else
        {
            const bno086_vector3f_t scaled = {
                .x = (float)raw_x / 256.0f,
                .y = (float)raw_y / 256.0f,
                .z = (float)raw_z / 256.0f,
            };

            acceleration_to_aircraft_axes(
                &scaled,
                &sample->gravity_ms2);

            sample->gravity_accuracy = accuracy;
            sample->gravity_valid = true;
        }
    }
    else if (report_id == BNO086_REPORT_GAME_ROTATION_VECTOR)
    {
        /*
         * Game Rotation Vector:
         * i, j, k, real en Q14.
         * Este report ocupa 12 bytes.
         */
        const bno086_quaternionf_t raw = {
            .x = (float)read_i16_le(&report[4]) / 16384.0f,
            .y = (float)read_i16_le(&report[6]) / 16384.0f,
            .z = (float)read_i16_le(&report[8]) / 16384.0f,
            .w = (float)read_i16_le(&report[10]) / 16384.0f,
        };

        quaternion_to_aircraft_axes(
            &raw,
            &sample->game_rotation);

        quaternion_to_euler(
            &sample->game_rotation,
            &sample->roll_deg,
            &sample->pitch_deg,
            &sample->yaw_deg);

        sample->rotation_accuracy = accuracy;
        sample->rotation_valid = true;
    }
}

static void process_packet(
    bno086_data_t *sample,
    const uint8_t *packet,
    uint16_t packet_len)
{
    if (packet_len < 9U)
    {
        return;
    }

    if (packet[2] != BNO086_SHTP_CHANNEL_REPORTS)
    {
        return;
    }

    const uint8_t *payload = &packet[4];
    const size_t payload_len = packet_len - 4U;

    /*
     * Los sensor reports normales llegan detrás del Base Timestamp 0xFB.
     */
    if (payload[0] != BNO086_SHTP_REPORT_BASE_TIMESTAMP)
    {
        return;
    }

    size_t offset = 5U;

    while (offset < payload_len)
    {
        const uint8_t report_id = payload[offset];
        const size_t report_len = bno086_report_size(report_id);

        if ((report_len == 0U) ||
            ((offset + report_len) > payload_len))
        {
            break;
        }

        process_sensor_report(
            sample,
            &payload[offset]);

        offset += report_len;
    }
}

/* -------------------------------------------------------------------------- */
/* API pública                                                                 */
/* -------------------------------------------------------------------------- */

bno086_data_t BNO086_get_data(void)
{
    bno086_data_t copy;

    portENTER_CRITICAL(&s_mux);
    copy = s_data;
    portEXIT_CRITICAL(&s_mux);

    return copy;
}

static void bno086_task(void *arg)
{
    (void)arg;

    uint8_t packet[BNO086_PACKET_BUFFER_SIZE];

    while (1)
    {
        uint16_t packet_len = 0U;

        const esp_err_t err = bno086_driver_receive_packet(
            packet,
            sizeof(packet),
            &packet_len);

        if (err == ESP_OK)
        {
            bno086_data_t sample = BNO086_get_data();

            process_packet(
                &sample,
                packet,
                packet_len);

            sample.valid =
                sample.acceleration_valid &&
                sample.gyro_valid &&
                sample.rotation_valid;

            portENTER_CRITICAL(&s_mux);
            s_data = sample;
            portEXIT_CRITICAL(&s_mux);
        }
        else if ((err != ESP_ERR_NOT_FOUND) &&
                 (err != ESP_ERR_TIMEOUT))
        {
            ESP_LOGW(
                TAG,
                "Error SHTP: %s",
                esp_err_to_name(err));
        }

        /*
         * El BNO086 se consulta con polling I2C. 2 ms evita ocupar la CPU
         * innecesariamente y es muy inferior al periodo de report de 40 ms.
         */
        vTaskDelay(pdMS_TO_TICKS(BNO086_TASK_PERIOD_MS));
    }
}

esp_err_t BNO086_start(void)
{
    if (s_task != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(
        bno086_driver_init(),
        TAG,
        "No se pudo iniciar BNO086");

    if (xTaskCreate(
            bno086_task,
            "bno086",
            BNO086_TASK_STACK_SIZE,
            NULL,
            BNO086_TASK_PRIORITY,
            &s_task) != pdPASS)
    {
        s_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(
        TAG,
        "BNO086 iniciado: Game RV + Gravity + Linear Acceleration");

    return ESP_OK;
}
