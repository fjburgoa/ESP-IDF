/**
 * @file DataLogger.h
 * @brief Registro circular en RAM de la telemetría inercial del EFIS.
 */

#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Una línea completa del registro, almacenada internamente en formato binario. */
typedef struct
{
    int64_t utc_time_ms;

    /* Coordenadas GNSS en grados decimales. NAN indica ausencia de FIX. */
    double latitude_deg;
    double longitude_deg;

    float acceleration_x_ms2;
    float acceleration_y_ms2;
    float acceleration_z_ms2;

    float linear_acceleration_x_ms2;
    float linear_acceleration_y_ms2;
    float linear_acceleration_z_ms2;

    float gravity_x_ms2;
    float gravity_y_ms2;
    float gravity_z_ms2;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float pitch_deg;
    float roll_deg;
    float slip_ball_deg;
    float turn_rate_dps;
} datalogger_sample_t;

typedef struct
{
    bool initialized;
    bool recording;
    bool data_available;
    bool wrapped;

    uint32_t samples;
    uint32_t capacity;
    uint32_t total_samples;
    size_t memory_bytes;
} datalogger_status_t;

/** Reserva el buffer circular y crea la tarea de adquisición. */
esp_err_t DataLogger_start(void);

/** Vacía lógicamente el buffer e inicia una grabación nueva. */
esp_err_t DataLogger_begin_recording(void);

/** Detiene la adquisición conservando en RAM las muestras registradas. */
esp_err_t DataLogger_stop_recording(void);

/** Devuelve una instantánea coherente del estado del registrador. */
datalogger_status_t DataLogger_get_status(void);

/**
 * Copia una muestra por índice cronológico: 0 es la más antigua conservada.
 * Solo se permite leer después de detener la grabación.
 */
esp_err_t DataLogger_get_sample(uint32_t chronological_index,
                                datalogger_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* DATALOGGER_H */
