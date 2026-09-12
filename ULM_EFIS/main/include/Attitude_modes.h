#ifndef ATTITUDE_MODES_H
#define ATTITUDE_MODES_H
#include <stdbool.h>
#include "esp_err.h"
#include "BNO055.h"
#include "MPU6050.h"
typedef enum {ATTITUDE_MODE_BNO_AMG_ACCEL=0,ATTITUDE_MODE_MPU_ACCEL=1,ATTITUDE_MODE_BNO_IMUPLUS_GRAVITY=2,ATTITUDE_MODE_BNO_NDOF_GRAVITY=3,ATTITUDE_MODE_FUSION_NORM=4,ATTITUDE_MODE_FUSION_DOUBLE=5} attitude_mode_t;
typedef struct {float roll_deg,pitch_deg,accel_weight,accel_norm_ms2,gravity_norm_ms2,accel_error_ms2,angle_error_deg,accel_mod_weight,accel_angle_weight;bool valid;} attitude_output_t;
esp_err_t Attitude_set_mode(attitude_mode_t mode);
attitude_mode_t Attitude_get_mode(void);
const char *Attitude_get_mode_name(attitude_mode_t mode);
attitude_output_t Attitude_compute(const bno055_data_t *bno,const mpu6050_data_t *mpu);
#endif
