/**
 * @file mainGPS.c
 * @brief Programa mínimo de prueba para GPS.c/GPS.h.
 */

#include <inttypes.h>
#include <stdio.h>

#include "GPS.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN_GPS";

/* -------------------------------------------------------------------------- */

void app_main(void)
{
    ESP_ERROR_CHECK(GPS_start());

    for (;;)
    {
        const gps_data_t gps = GPS_get_data();

        ESP_LOGI(
            TAG,
            "connected=%d valid=%d fix=%d baud=%" PRIu32
            " | lat=%.7f lon=%.7f alt=%.1f m",
            gps.connected,
            gps.valid,
            gps.fix_valid,
            gps.baud_rate,
            gps.latitude_deg,
            gps.longitude_deg,
            (double)gps.altitude_m);

        ESP_LOGI(
            TAG,
            "GS=%.2f kt / %.2f km/h | track=%.1f deg "
            "| sat=%u HDOP=%.2f FIX=%u",
            (double)gps.ground_speed_knots,
            (double)gps.ground_speed_kmh,
            (double)gps.ground_track_deg,
            gps.satellites,
            (double)gps.hdop,
            gps.fix_quality);

        ESP_LOGI(
            TAG,
            "UTC %02u/%02u/%04u %02u:%02u:%02u.%03u "
            "| NMEA=%" PRIu32 " checksum_err=%" PRIu32
            " parse_err=%" PRIu32,
            gps.utc_day,
            gps.utc_month,
            gps.utc_year,
            gps.utc_hour,
            gps.utc_minute,
            gps.utc_second,
            gps.utc_millisecond,
            gps.received_sentences,
            gps.checksum_errors,
            gps.parse_errors);

        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}
