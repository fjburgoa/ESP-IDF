#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* Acelerómetro, expresado en unidades G. */
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float accel_total_g;

    /* Velocidades angulares del cuerpo, expresadas en grados/s. */
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    /* Actitud estimada mediante filtro complementario. */
    float roll_deg;
    float pitch_deg;

    /*
     * Derivada del ángulo de guiñada compensada por roll y pitch.
     * Se utiliza para mover el avión del coordinador de giro.
     */
    float yaw_rate_dps;

    /*
     * Posición de la bola del coordinador de giro.
     * Se obtiene de la aceleración lateral compensada y filtrada.
     */
    float slip_ball_deg;

    /*
     * G-meter:
     * g_current = accel_z_g - 1 G
     * g_max y g_min retienen los extremos desde el último reset.
     */
    float g_current;
    float g_max;
    float g_min;

    bool valid;
} mpu6050_data_t;

esp_err_t MPU6050_start(void);

mpu6050_data_t MPU6050_get_data(void);

void MPU6050_reset_accel_peaks(void);

void MPU6050_compute_roll_pitch(float ax_g,
                                float ay_g,
                                float az_g,
                                float gx_dps,
                                float gy_dps,
                                float dt_s,
                                float *roll_deg,
                                float *pitch_deg);

#ifdef __cplusplus
}
#endif

#endif
