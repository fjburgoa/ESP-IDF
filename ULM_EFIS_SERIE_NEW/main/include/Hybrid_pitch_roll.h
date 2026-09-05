#ifndef HYBRID_PITCH_ROLL_H
#define HYBRID_PITCH_ROLL_H

#include <stdbool.h>

typedef struct
{
    float pitch_deg;
    float roll_deg;

    /* Peso final del MPU6050 */
    float accel_weight;

    float accel_norm_ms2;
    float gravity_norm_ms2;
    float accel_error_ms2;

    /* Nuevos datos para el criterio angular */
    float angle_error_deg;
    float accel_mod_weight;
    float accel_angle_weight;

    bool valid;

} hybrid_pitch_roll_data_t;

hybrid_pitch_roll_data_t Hybrid_pitch_roll_update(
    float ax, float ay, float az,
    float gx, float gy, float gz);

#endif
