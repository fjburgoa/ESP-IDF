/**
 * @file GPS.h
 * @brief Interfaz pública del driver GNSS del EFIS.
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
    bool connected;
    bool valid;
    bool fix_valid;

    double latitude_deg;
    double longitude_deg;
    float altitude_m;

    float ground_speed_knots;
    float ground_speed_kmh;
    float ground_speed_mps;
    float ground_track_deg;

    uint8_t fix_quality;
    uint8_t satellites;
    float hdop;

    uint8_t utc_hour;
    uint8_t utc_minute;
    uint8_t utc_second;
    uint16_t utc_millisecond;

    uint8_t utc_day;
    uint8_t utc_month;
    uint16_t utc_year;

    uint32_t baud_rate;
    uint32_t received_sentences;
    uint32_t checksum_errors;
    uint32_t parse_errors;
} gps_data_t;

esp_err_t GPS_start(void);
gps_data_t GPS_get_data(void);
bool GPS_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif
