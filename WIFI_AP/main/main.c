/******************************************************************************
 * @file    main.c
 * @brief   ESP32-S3 Electronic Flight Instrument System (EFIS)
 *
 * ============================================================================
 * PROJECT DESCRIPTION
 * ============================================================================
 *
 * This project implements a low-cost Electronic Flight Instrument System
 * (EFIS) based on the ESP32-S3 microcontroller. The ESP32 acquires flight
 * sensor data, computes the required flight parameters and transmits them
 * through a WebSocket connection to a web browser running on a smartphone,
 * tablet or PC.
 *
 * The browser performs all graphics rendering while the ESP32 is dedicated
 * exclusively to sensor acquisition, filtering and flight calculations.
 *
 * ============================================================================
 * SYSTEM ARCHITECTURE
 * ============================================================================
 *
 *                       +----------------------+
 *                       |      BMP280          |
 *                       | Pressure / Temp.     |
 *                       +----------+-----------+
 *                                  |
 *                       +----------v-----------+
 *                       |      MPU6050         |
 *                       | Accel + Gyroscope    |
 *                       +----------+-----------+
 *                                  |
 *                       +----------v-----------+
 *                       |     HMC5883L         |
 *                       |    Magnetometer      |
 *                       +----------+-----------+
 *                                  |
 *                                  v
 *                    +---------------------------+
 *                    |     Flight Computer       |
 *                    |---------------------------|
 *                    | Altitude                  |
 *                    | Vertical Speed            |
 *                    | Attitude (Pitch/Roll)     |
 *                    | Heading                   |
 *                    | Turn Coordinator          |
 *                    | Slip Indicator            |
 *                    | G-Meter                   |
 *                    +-------------+-------------+
 *                                  |
 *                                  |
 *                             JSON Telemetry
 *                                  |
 *                             WebSocket (25 Hz)
 *                                  |
 *                                  v
 *                    +---------------------------+
 *                    | Smartphone Web Browser    |
 *                    | HTML + CSS + JavaScript   |
 *                    | Canvas / SVG Rendering    |
 *                    +---------------------------+
 *
 * ============================================================================
 * CURRENT INSTRUMENTS
 * ============================================================================
 *
 * ---------------------------------------------------------------------------
 * 1. ALTIMETER
 * ---------------------------------------------------------------------------
 * Display:
 *      - Indicated altitude [ft]
 *
 * Inputs:
 *      - BMP280 pressure
 *
 * Calculated variables:
 *      - Altitude MSL
 *      - Feet conversion
 *
 * Configuration (NVS):
 *      - QNH
 *
 * ---------------------------------------------------------------------------
 * 2. VERTICAL SPEED INDICATOR
 * ---------------------------------------------------------------------------
 * Display:
 *      - Vertical speed [ft/min]
 *
 * Inputs:
 *      - Barometric altitude
 *
 * Calculated variables:
 *      - d(Altitude)/dt
 *
 * Future:
 *      - Complementary filter with accelerometer
 *
 * ---------------------------------------------------------------------------
 * 3. ATTITUDE INDICATOR
 * ---------------------------------------------------------------------------
 * Display:
 *      - Pitch
 *      - Roll
 *
 * Inputs:
 *      - Gyroscope
 *      - Accelerometer
 *
 * Future:
 *      - BNO055 attitude engine
 *
 * Configuration (NVS):
 *      - Pitch offset
 *
 * ---------------------------------------------------------------------------
 * 4. HEADING INDICATOR
 * ---------------------------------------------------------------------------
 * Display:
 *      - Magnetic heading
 *
 * Inputs:
 *      - Magnetometer
 *      - Roll/Pitch compensation
 *
 * Configuration (NVS):
 *      - Heading offset
 *
 * ---------------------------------------------------------------------------
 * 5. TURN COORDINATOR
 * ---------------------------------------------------------------------------
 * Display:
 *      - Turn rate
 *      - Slip / Skid ball
 *
 * Inputs:
 *      - Gyroscope Z
 *      - Lateral acceleration
 *
 * ---------------------------------------------------------------------------
 * 6. G-METER
 * ---------------------------------------------------------------------------
 * Display:
 *      - Maximum absolute acceleration [G]
 *
 * Inputs:
 *      - Accelerometer
 *
 * Behaviour:
 *      - Peak hold
 *      - Manual reset
 *
 * ============================================================================
 * MAIN SYSTEM VARIABLES
 * ============================================================================
 *
 * Raw sensor data
 * ----------------
 *      pressure_hPa
 *      temperature_C
 *
 *      accX_g
 *      accY_g
 *      accZ_g
 *
 *      gyroX_dps
 *      gyroY_dps
 *      gyroZ_dps
 *
 *      magX_uT
 *      magY_uT
 *      magZ_uT
 *
 * Flight variables
 * ----------------
 *      pitch_deg
 *      roll_deg
 *      yaw_deg
 *
 *      altitude_m
 *      altitude_ft
 *
 *      verticalSpeed_ms
 *      verticalSpeed_fpm
 *
 *      heading_deg
 *
 *      turnRate_dps
 *      slip_deg
 *
 *      gCurrent
 *      gPeak
 *
 * Configuration stored in NVS
 * ---------------------------
 *      qnh_hPa
 *      pitchOffset_deg
 *      headingOffset_deg
 *
 * ============================================================================
 * SOFTWARE MODULES
 * ============================================================================
 *
 * Drivers
 *      bmp280.c
 *      mpu6050.c
 *      hmc5883l.c
 *
 * Flight Computer
 *      attitude.c
 *      altimeter.c
 *      variometer.c
 *      compass.c
 *      turn_coordinator.c
 *      gmeter.c
 *      calibration.c
 *
 * Communication
 *      wifi_ap.c
 *      webserver.c
 *      websocket.c
 *
 * User Interface
 *      index.html
 *
 * Configuration
 *      settings.c
 *
 * ============================================================================
 * FUTURE EXTENSIONS
 * ============================================================================
 *
 * - BNO055 support
 * - GPS receiver
 * - Airspeed indicator
 * - Density altitude
 * - Flight recorder
 * - Audible alarms
 * - AHRS calibration page
 * - SD card logging
 * - Terrain awareness
 *
 ******************************************************************************/

#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_ap.h"
#include "webserver.h"
#include "websocket.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND))
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    else
    {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(wifi_ap_start()); // arranca el Access point

    httpd_handle_t server = webserver_start(); // arranca el servidor web http
    if (server == NULL)
    {
        ESP_LOGE(TAG, "No se ha podido iniciar el servidor web");
        return;
    }

    ESP_ERROR_CHECK(websocket_start_dummy_stream(server)); // Register WebSocket endpoint

    ESP_LOGI(TAG, "Sistema iniciado correctamente");
}
