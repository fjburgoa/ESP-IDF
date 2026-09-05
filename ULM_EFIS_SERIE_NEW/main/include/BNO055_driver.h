/**
 * @file BNO055_driver.h
 * @brief Capa interna de acceso al hardware BNO055.
 */

#ifndef BNO055_DRIVER_H
#define BNO055_DRIVER_H

#include <stdint.h>

#include "esp_err.h"
#include "BNO055.h"

#ifdef __cplusplus
extern "C"
{
#endif

esp_err_t bno055_driver_init(void);

esp_err_t bno055_driver_set_mount_mode(bno055_mount_mode_t mode);
bno055_mount_mode_t bno055_driver_get_mount_mode(void);

/*
 * API compatible con el EFIS original.
 */
esp_err_t bno055_driver_set_operation_mode(bno055_operation_mode_t mode);
bno055_operation_mode_t bno055_driver_get_operation_mode(void);

/*
 * API de prueba: expone el OPR_MODE real y permite recorrer con BOOT:
 *
 *   AMG -> IMUPLUS -> NDOF -> NDOF_FMC_OFF -> AMG ...
 */
uint8_t bno055_driver_get_mode_register(void);
const char *bno055_driver_get_mode_name(void);
esp_err_t bno055_driver_cycle_test_mode(void);

/* Lecturas físicas ya convertidas a unidades de ingeniería. */
esp_err_t bno055_driver_read_acceleration(
    bno055_vector3f_t *acceleration_ms2);

esp_err_t bno055_driver_read_gyro(
    bno055_vector3f_t *gyro_dps);

esp_err_t bno055_driver_read_quaternion(
    bno055_quaternionf_t *quaternion);

esp_err_t bno055_driver_read_linear_acceleration(
    bno055_vector3f_t *linear_acceleration_ms2);

esp_err_t bno055_driver_read_gravity(
    bno055_vector3f_t *gravity_ms2);

esp_err_t bno055_driver_read_temperature(
    int8_t *temperature_c);

esp_err_t bno055_driver_read_calibration(
    uint8_t *system,
    uint8_t *gyro,
    uint8_t *accel,
    uint8_t *mag);

#ifdef __cplusplus
}
#endif

#endif /* BNO055_DRIVER_H */
