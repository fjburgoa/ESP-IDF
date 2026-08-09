/**
 * @file GPS.h
 * @brief Driver UART/NMEA/UBX para receptor GNSS del EFIS.
 */

#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* Estado general */
    bool connected;
    bool valid;
    bool fix_valid;

    /* Posición */
    double latitude_deg;
    double longitude_deg;
    float altitude_m;

    /* Movimiento */
    float ground_speed_knots;
    float ground_speed_kmh;
    float ground_speed_mps;
    float ground_track_deg;

    /* Calidad de navegación */
    uint8_t fix_quality;
    uint8_t satellites;
    float hdop;

    /* Tiempo UTC */
    uint8_t utc_hour;
    uint8_t utc_minute;
    uint8_t utc_second;
    uint16_t utc_millisecond;

    /* Fecha UTC */
    uint8_t utc_day;
    uint8_t utc_month;
    uint16_t utc_year;

    /* Diagnóstico del enlace */
    uint32_t baud_rate;
    uint32_t received_sentences;
    uint32_t checksum_errors;
    uint32_t parse_errors;

} gps_data_t;

/**
 * @brief Inicializa el receptor y crea la tarea de recepción.
 *
 * Secuencia:
 *   1. Inicializa UART1 a 9600 baud.
 *   2. Detecta una sentencia NMEA.
 *   3. Intenta configurar navegación a 5 Hz.
 *   4. Intenta dejar únicamente GGA y RMC.
 *   5. Intenta cambiar receptor y ESP32-S3 a 115200 baud.
 *   6. Intenta guardar la configuración mediante UBX-CFG-CFG.
 *   7. Arranca la tarea de recepción y parsing.
 *
 * Si el receptor no acepta UBX, continúa funcionando con su configuración
 * original siempre que se reciban sentencias NMEA válidas.
 */
esp_err_t GPS_start(void);

/**
 * @brief Devuelve una instantánea atómica de los últimos datos GPS.
 */
gps_data_t GPS_get_data(void);

/**
 * @brief Devuelve true si se están recibiendo sentencias NMEA válidas.
 */
bool GPS_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */
