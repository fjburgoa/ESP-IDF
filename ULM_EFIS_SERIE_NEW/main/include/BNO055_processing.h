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
void bno055_processing_reset(
    bno055_processing_state_t *state);

/**
 * @brief Reinicia los extremos del G-meter al valor instantáneo indicado.
 */
void bno055_processing_reset_g_peaks(
    bno055_processing_state_t *state,
    float g_current);

/**
 * @brief Procesa una muestra adquirida en modo AMG.
 *
 * En AMG la actitud se estima en software a partir de acelerómetro y
 * giróscopo. También se calculan régimen de giro, bola y G-meter.
 */
void bno055_process_sample_amg(
    bno055_processing_state_t *state,
    bno055_data_t *data,
    float dt_s);

/**
 * @brief Procesa una muestra adquirida en modo NDOF.
 *
 * En NDOF la actitud y el vector gravedad proceden de la fusión interna del
 * BNO055. El procesado del EFIS calcula las magnitudes derivadas restantes.
 */
void bno055_process_sample_ndof(
    bno055_processing_state_t *state,
    bno055_data_t *data,
    float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* BNO055_PROCESSING_H */
