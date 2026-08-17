#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

/* Opciones generales de compilacion del EFIS. */
#define DATALOGGER_ENABLED 0

#define BNO055_USE_INTERNAL_FUSION 0 // CAMBIA ENTRE NDOF y AMG
#define HEADING_GPS 1                // EL HEADING es del GPS = 1, o El Heading es calculado (IMU o integral de yaw_rate_dps)

#define BNO055_ATTITUDE_TAU_S 0.3f
#define BNO055_PITCH_GYRO_SIGN 1.0f
#define BNO055_ROLL_GYRO_SIGN -1.0f

#endif /* PROJECT_CONFIG_H */
