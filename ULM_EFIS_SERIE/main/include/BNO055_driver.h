/**
 * @file BNO055_driver.h
 * @brief Capa interna de acceso al hardware BNO055.
 *
 * Este header es privado del módulo BNO055. No debería ser utilizado
 * directamente por el resto de la aplicación.
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

    /**
     * @brief Detecta, configura e inicializa físicamente el BNO055.
     *
     * El arranque se realiza en orientación V y en el modo definido inicialmente
     * por BNO055_DEFAULT_OPERATION_MODE.
     */
    esp_err_t bno055_driver_init(void);

    /**
     * @brief Cambia el remapeo físico V/H del BNO055.
     *
     * La función entra temporalmente en CONFIGMODE, modifica AXIS_MAP y restaura
     * después el modo de operación seleccionado.
     */
    esp_err_t bno055_driver_set_mount_mode(bno055_mount_mode_t mode);

    /**
     * @brief Devuelve el último modo de montaje aplicado al sensor.
     */
    bno055_mount_mode_t bno055_driver_get_mount_mode(void);

    /**
     * @brief Cambia el modo de operación del BNO055 entre AMG y NDOF.
     */
    esp_err_t bno055_driver_set_operation_mode(bno055_operation_mode_t mode);

    /**
     * @brief Devuelve el modo de operación actualmente activo.
     */
    bno055_operation_mode_t bno055_driver_get_operation_mode(void);

    /* Lecturas físicas ya convertidas a unidades de ingeniería. */
    esp_err_t bno055_driver_read_acceleration(bno055_vector3f_t *acceleration_ms2);
    // esp_err_t bno055_driver_read_magnetic_field(bno055_vector3f_t *magnetic_field_ut);
    esp_err_t bno055_driver_read_gyro(bno055_vector3f_t *gyro_dps);

    esp_err_t bno055_driver_read_euler(
        float *heading_deg,
        float *roll_deg,
        float *pitch_deg);

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
