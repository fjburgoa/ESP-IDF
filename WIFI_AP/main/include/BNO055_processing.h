/**
 * @file BNO055_processing.h
 * @brief Procesado interno de señales del módulo BNO055.
 *
 * No realiza ningún acceso I2C ni conoce registros del sensor.
 */

#ifndef BNO055_PROCESSING_H
#define BNO055_PROCESSING_H

#include <stdbool.h>

#include "BNO055.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Estado persistente de los filtros y estimadores.
     *
     * Se mantiene fuera de BNO055_processing.c para permitir que BNO055.c controle
     * explícitamente cuándo se reinicia, por ejemplo al cambiar la orientación V/H.
     */
    typedef struct
    {
        bool attitude_initialized;
        float pitch_est_deg;
        float roll_est_deg;

        bool gyro_filter_initialized;
        float gyro_x_filtered_dps;
        float gyro_y_filtered_dps;
        float gyro_z_filtered_dps;

        float yaw_rate_filtered_dps;
        float slip_ball_filtered_deg;

        /* Yaw integrado utilizado únicamente en AMG como dato de demostración. */
        float yaw_demo_deg;

        /* Extremos del G-meter desde el último RESET. */
        bool g_peaks_initialized;
        float g_min;
        float g_max;

    } bno055_processing_state_t;

    /**
     * @brief Reinicia todos los estados de filtros y estimadores.
     */
    void bno055_processing_reset(bno055_processing_state_t *state);

    /**
     * @brief Reinicia los extremos del G-meter al valor instantáneo indicado.
     */
    void bno055_processing_reset_g_peaks(
        bno055_processing_state_t *state,
        float g_current);

    /**
     * @brief Procesa una muestra ya adquirida y convertida a unidades físicas.
     *
     * @param state Estado persistente de los algoritmos.
     * @param data Muestra BNO055 que se completa in-place.
     * @param dt_s Periodo de muestreo [s].
     * @param use_internal_fusion true=NDOF; false=AMG.
     */
    void bno055_process_sample(
        bno055_processing_state_t *state,
        bno055_data_t *data,
        float dt_s,
        bool use_internal_fusion);

#ifdef __cplusplus
}
#endif

#endif /* BNO055_PROCESSING_H */
