#ifndef BNO055_H
#define BNO055_H

#include <stdbool.h>
#include "esp_err.h"

typedef enum
{
    BNO055_OPERATION_MODE_AMG = 0,
    BNO055_OPERATION_MODE_NDOF = 1,
    BNO055_OPERATION_MODE_IMUPLUS = 2,
    BNO055_OPERATION_MODE_NDOF_FMC_OFF = 3
} bno055_operation_mode_t;

typedef struct
{
    float x, y, z;
} bno055_vector3f_t;
typedef struct
{
    float w, x, y, z;
} bno055_quaternionf_t;

typedef struct
{
    bno055_vector3f_t acceleration_ms2;
    bno055_vector3f_t gyro_dps;
    bno055_vector3f_t linear_accel;
    bno055_vector3f_t gravity;
    bool linear_accel_valid;
    bool gravity_valid;
    float roll_deg;
    float pitch_deg;
    bno055_operation_mode_t operation_mode;
    bool valid;
} bno055_data_t;

esp_err_t BNO055_start(void);
bno055_data_t BNO055_get_data(void);

#endif
