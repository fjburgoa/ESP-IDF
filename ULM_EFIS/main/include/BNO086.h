/**
 * @file BNO086.h
 * @brief Interfaz pública del BNO086 para ULM-EFIS WiFi.
 */

#ifndef BNO086_H
#define BNO086_H

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        /*
         * V: montaje actual validado experimentalmente.
         * H: placa girada 90 grados en el plano horizontal.
         */
        BNO086_MOUNT_VERTICAL = 0,
        BNO086_MOUNT_HORIZONTAL = 1
    } bno086_mount_mode_t;

    typedef struct
    {
        float x;
        float y;
        float z;
    } bno086_vector3f_t;

    typedef struct
    {
        float w;
        float x;
        float y;
        float z;
    } bno086_quaternionf_t;

    typedef struct
    {
        /* Aceleración total en ejes del avión. */
        float accel_x_g;
        float accel_y_g;
        float accel_z_g;
        float accel_total_g;

        bno086_vector3f_t acceleration_ms2;
        bno086_vector3f_t linear_acceleration_ms2;
        bno086_vector3f_t gravity_ms2;
        bno086_vector3f_t gyro_dps;

        /* Game Rotation Vector expresado en ejes del avión. */
        bno086_quaternionf_t quaternion;

        float heading_deg;
        float roll_deg;
        float pitch_deg;

        float yaw_rate_dps;
        float slip_ball_deg;

        float g_current;
        float g_max;
        float g_min;

        uint8_t acceleration_accuracy;
        uint8_t gyro_accuracy;
        uint8_t linear_accel_accuracy;
        uint8_t gravity_accuracy;
        uint8_t rotation_accuracy;

        bool acceleration_valid;
        bool gyro_valid;
        bool linear_accel_valid;
        bool gravity_valid;
        bool rotation_valid;
        bool valid;

    } bno086_data_t;

    esp_err_t BNO086_start(void);
    bno086_data_t BNO086_get_data(void);

    void BNO086_reset_accel_peaks(void);

    esp_err_t BNO086_set_mount_mode(bno086_mount_mode_t mode);
    bno086_mount_mode_t BNO086_get_mount_mode(void);

    void BNO086_set_output_queue(QueueHandle_t queue);

#ifdef __cplusplus
}
#endif

#endif /* BNO086_H */
