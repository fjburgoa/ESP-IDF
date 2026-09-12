/**
 * @file DataLogger.h
 * @brief Registro SPIFFS de aceleración vertical y tiempo GPS.
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

typedef struct
{
    bool mounted;
    bool recording;
    bool file_available;

    uint32_t samples;
    size_t file_size_bytes;

} datalogger_status_t;

/**
 * @brief Monta la partición SPIFFS "FicheroAcelerac" y crea la tarea.
 *
 * Tras un reset recording siempre comienza en false.
 *
 * La primera vez, si la partición está completamente borrada (0xFF) y no
 * contiene todavía un sistema de archivos, se permite formatearla.
 * Si contiene datos pero falla el montaje, NO se formatea automáticamente.
 */
esp_err_t DataLogger_start(void);

/**
 * @brief Comienza un registro nuevo.
 *
 * Abre /spiffs/aceleracion.csv con "w", por lo que el fichero anterior
 * se trunca. Requiere BNO055 válido y fecha/hora GPS válida.
 */
esp_err_t DataLogger_begin_recording(void);

/**
 * @brief Detiene la grabación y cierra limpiamente el fichero.
 */
esp_err_t DataLogger_stop_recording(void);

/**
 * @brief Devuelve una instantánea del estado del registrador.
 */
datalogger_status_t DataLogger_get_status(void);

/**
 * @brief Ruta VFS del fichero CSV.
 */
const char *DataLogger_get_file_path(void);

#ifdef __cplusplus
}
#endif

#endif /* DATALOGGER_H */
