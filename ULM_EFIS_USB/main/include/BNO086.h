#ifndef BNO086_H
#define BNO086_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

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
    bno086_vector3f_t acceleration_ms2;
    bno086_vector3f_t gyro_dps;

    bno086_vector3f_t linear_accel_ms2;
    bno086_vector3f_t gravity_ms2;

    bno086_quaternionf_t game_rotation;

    float roll_deg;
    float pitch_deg;
    float yaw_deg;

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

#endif /* BNO086_H */
