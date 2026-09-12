#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    float accel_x_ms2;
    float accel_y_ms2;
    float accel_z_ms2;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float roll_deg;
    float pitch_deg;
    bool valid;
} mpu6050_data_t;

esp_err_t MPU6050_start(void);
mpu6050_data_t MPU6050_get_data(void);

#endif
