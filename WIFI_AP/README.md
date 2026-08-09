/******************\*\*******************\*\*\*******************\*\*******************

- @file main.c
- @brief ESP32-S3 Electronic Flight Instrument System (EFIS)
-
- =============================================================================
- PROJECT DESCRIPTION
- =============================================================================
-
- This project implements a low-cost Electronic Flight Instrument System
- (EFIS) based on the ESP32-S3 microcontroller.
-
- The ESP32-S3 acquires data from the flight sensors, performs the required
- signal processing, filtering and flight-parameter calculations, and sends
- the resulting telemetry through a WebSocket connection to a web browser
- running on a smartphone, tablet or PC.
-
- The browser is mainly responsible for graphical representation of the
- instruments. Sensor processing and flight calculations are performed in
- the ESP32-S3 so that the values transmitted through the WebSocket already
- represent the processed flight variables.
-
-
- =============================================================================
- SYSTEM ARCHITECTURE
- =============================================================================
-
-
- ```
                 +----------------------+
  ```
- ```
                 |       BMP280         |
  ```
- ```
                 | Pressure / Temp.     |
  ```
- ```
                 +----------+-----------+
  ```
- ```
                            |
  ```
- ```
                            |
  ```
- ```
                 +----------v-----------+
  ```
- ```
                 |       BNO055         |
  ```
- ```
                 |----------------------|
  ```
- ```
                 | Accelerometer        |
  ```
- ```
                 | Gyroscope            |
  ```
- ```
                 | Magnetometer         |
  ```
- ```
                 | NDOF sensor fusion   |
  ```
- ```
                 +----------+-----------+
  ```
- ```
                            |
  ```
- ```
                            |
  ```
- ```
                 +----------v-----------+
  ```
- ```
                 |     GPS / GNSS       |
  ```
- ```
                 |----------------------|
  ```
- ```
                 | Position             |
  ```
- ```
                 | Altitude             |
  ```
- ```
                 | UTC date / time      |
  ```
- ```
                 +----------+-----------+
  ```
- ```
                            |
  ```
- ```
                            v
  ```
-
- ```
              +---------------------------+
  ```
- ```
              |     ESP32-S3 EFIS         |
  ```
- ```
              |---------------------------|
  ```
- ```
              | Barometric altitude       |
  ```
- ```
              | Vertical speed            |
  ```
- ```
              | Pitch / Roll              |
  ```
- ```
              | Heading                   |
  ```
- ```
              | Turn rate                 |
  ```
- ```
              | Slip / Skid               |
  ```
- ```
              | G-meter                   |
  ```
- ```
              | GPS position / altitude   |
  ```
- ```
              | UTC date / time           |
  ```
- ```
              +-------------+-------------+
  ```
- ```
                            |
  ```
- ```
                            |
  ```
- ```
                      JSON Telemetry
  ```
- ```
                            |
  ```
- ```
                            |
  ```
- ```
                     WebSocket (80 ms)
  ```
- ```
                      ~12.5 messages/s
  ```
- ```
                            |
  ```
- ```
                            v
  ```
-
- ```
              +---------------------------+
  ```
- ```
              | Smartphone Web Browser    |
  ```
- ```
              |---------------------------|
  ```
- ```
              | HTML + CSS + JavaScript   |
  ```
- ```
              | Canvas / SVG Rendering    |
  ```
- ```
              +---------------------------+
  ```
-
-
- =============================================================================
- SENSOR PROCESSING
- =============================================================================
-
- BMP280
- ***
-
- The BMP280 provides:
-
- - Atmospheric pressure
- - Temperature
-
- Barometric altitude is calculated from pressure and the currently selected
- QNH:
-
- ```
   altitude = f(pressure, QNH)
  ```
-
- The Vertical Speed Indicator (VSI) is calculated in BMP280.c from the
- variation of barometric altitude:
-
- ```
   vertical_speed = d(altitude) / dt
  ```
-
- The resulting vertical speed is filtered with a first-order low-pass filter
- before being published to the rest of the system.
-
-
- BNO055
- ***
-
- The BNO055 operates in NDOF mode and internally performs sensor fusion using:
-
- - Accelerometer
- - Gyroscope
- - Magnetometer
-
- The driver provides:
-
- - Pitch
- - Roll
- - Magnetic heading
- - Angular rates
- - Linear acceleration
- - Gravity vector
- - Magnetic field
- - Quaternion
-
- The turn rate used by the Turn Coordinator is calculated from the gyroscope
- and compensated using the current roll and pitch angles.
-
- A first-order low-pass filter is applied to the calculated turn rate inside
- BNO055.c.
-
- The slip/skid ball is calculated from lateral acceleration. A deadband,
- gain, saturation and low-pass filtering are applied in BNO055.c before the
- value is transmitted to the browser.
-
- Consequently, the web interface does not apply additional filtering to the
- Turn Coordinator.
-
-
- GPS / GNSS
- ***
-
- The GPS receiver is connected through UART1.
-
- The driver processes NMEA GGA and RMC sentences and provides:
-
- - Latitude
- - Longitude
- - GPS altitude
- - Ground speed
- - Ground track
- - Fix quality
- - Number of satellites
- - HDOP
- - UTC time
- - UTC date
-
- GPS communication validity and navigation FIX validity are treated
- independently.
-
- Therefore, UTC date and time may be available and displayed even when the
- receiver does not yet have a valid navigation FIX.
-
- Latitude, longitude and GPS altitude are only considered valid when a FIX
- is available.
-
- For diagnostic purposes, GPS.c prints the received UTC time to the ESP-IDF
- serial monitor whenever a valid GGA or RMC sentence is processed.
-
-
- =============================================================================
- CURRENT INSTRUMENTS
- =============================================================================
-
-
- ***
- 1. ALTIMETER
- ***
-
- Display:
-
- - Indicated altitude [ft]
- - Indicated altitude [m]
- - GPS altitude [m]
-
- Inputs:
-
- - BMP280 pressure
- - User QNH setting
- - GPS altitude
-
- Calculated variables:
-
- - Barometric altitude MSL
- - Feet conversion
-
- Configuration stored in NVS:
-
- - QNH
-
- GPS altitude is permanently shown in the interface. If no valid GPS FIX is
- available, "--" is displayed instead of an altitude value.
-
-
- ***
- 2. VERTICAL SPEED INDICATOR
- ***
-
- Display:
-
- - Analog vertical-speed needle [ft/min]
-
- No separate numerical vertical-speed indication is displayed in the web
- interface.
-
- Inputs:
-
- - Barometric altitude calculated by BMP280.c
-
- Calculated variables:
-
- - Raw vertical speed:
-
- ```
      d(Altitude) / dt
  ```
-
- - Filtered vertical speed
-
- The VSI calculation and filtering are performed in BMP280.c rather than in
- the browser.
-
-
- ***
- 3. ATTITUDE INDICATOR
- ***
-
- Display:
-
- - Pitch
- - Roll
-
- Inputs:
-
- - BNO055 NDOF attitude solution
-
- Configuration stored in NVS:
-
- - Pitch offset
-
-
- ***
- 4. HEADING INDICATOR
- ***
-
- Display:
-
- - Magnetic heading
-
- Inputs:
-
- - BNO055 NDOF heading
-
- The BNO055 internally combines gyroscope, accelerometer and magnetometer
- information to obtain the heading solution.
-
- Configuration stored in NVS:
-
- - Heading offset
-
-
- ***
- 5. TURN COORDINATOR
- ***
-
- Display:
-
- - Turn rate
- - Slip / Skid ball
-
- Inputs:
-
- - BNO055 gyroscope
- - BNO055 attitude
- - Lateral acceleration
-
- Processing:
-
- - Roll/Pitch compensated turn-rate calculation
- - Turn-rate low-pass filtering
- - Slip-ball deadband
- - Slip-ball low-pass filtering
- - Slip-ball gain and saturation
-
- All dynamic filtering is implemented in BNO055.c.
-
- The HTML interface directly represents the already processed values received
- through the WebSocket.
-
-
- ***
- 6. G-METER
- ***
-
- Display:
-
- - Instantaneous G
- - Maximum retained G
- - Minimum retained G
-
- Inputs:
-
- - BNO055 accelerometer
-
- Behaviour:
-
- - Instantaneous indication
- - Maximum peak hold
- - Minimum peak hold
- - Manual reset from the web interface
-
-
- ***
- 7. GPS / GNSS INFORMATION
- ***
-
- Display:
-
- - UTC date
- - UTC time
- - Latitude
- - Longitude
- - GPS altitude
-
- Date and time may be displayed without a valid navigation FIX.
-
- Latitude, longitude and GPS altitude display "--" when the GPS does not have
- a valid FIX.
-
-
- =============================================================================
- VERTICAL SPEED TEST MODE
- =============================================================================
-
- Since testing a barometric Vertical Speed Indicator while the system remains
- stationary is difficult, main.c includes a test altitude generator.
-
- The test generator produces a triangular altitude waveform with:
-
- ```
   Climb rate   = +500 ft/min
  ```
- ```
   Descent rate = -500 ft/min
  ```
-
- Equivalent vertical speed:
-
- ```
   500 ft/min = 2.54 m/s
  ```
-
- The generated fake altitude is passed to the BMP280 vertical-speed
- processing instead of directly generating a vertical-speed value.
-
- Therefore the complete VSI processing chain is tested:
-
- ```
   Fake altitude
  ```
- ```
        |
  ```
- ```
        v
  ```
- ```
   Altitude derivative
  ```
- ```
        |
  ```
- ```
        v
  ```
- ```
   VSI low-pass filter
  ```
- ```
        |
  ```
- ```
        v
  ```
- ```
   JSON telemetry
  ```
- ```
        |
  ```
- ```
        v
  ```
- ```
   WebSocket
  ```
- ```
        |
  ```
- ```
        v
  ```
- ```
   Analog VSI needle
  ```
-
- This allows the complete vertical-speed algorithm to be verified without
- physically changing the altitude of the instrument.
-
-
- =============================================================================
- MAIN SYSTEM VARIABLES
- =============================================================================
-
- BMP280 data
- ***
-
- pressure_hPa
- temperature_C
- altitude_m
- verticalSpeed_ms
-
-
- BNO055 data
- ***
-
- accelX_g
- accelY_g
- accelZ_g
- accelTotal_g
-
- gyroX_dps
- gyroY_dps
- gyroZ_dps
-
- magX_uT
- magY_uT
- magZ_uT
-
- pitch_deg
- roll_deg
- heading_deg
-
- turnRate_dps
- slip_deg
-
- gCurrent
- gMax
- gMin
-
-
- GPS / GNSS data
- ***
-
- gpsConnected
- gpsFixValid
-
- gpsLatitude_deg
- gpsLongitude_deg
- gpsAltitude_m
-
- gpsGroundSpeed_knots
- gpsGroundTrack_deg
-
- gpsUtcHour
- gpsUtcMinute
- gpsUtcSecond
-
- gpsUtcDay
- gpsUtcMonth
- gpsUtcYear
-
-
- Display / derived variables
- ***
-
- altitude_ft
- verticalSpeed_fpm
-
-
- Configuration stored in NVS
- ***
-
- qnh_hPa
- pitchOffset_deg
- headingOffset_deg
-
-
- =============================================================================
- SOFTWARE MODULES
- =============================================================================
-
- Sensor drivers
- ***
-
- BMP280.c
- ```
   Pressure, temperature, barometric altitude and VSI calculation.
  ```
-
- BNO055.c
- ```
   NDOF attitude, heading, turn-rate processing, slip/skid calculation
  ```
- ```
   and G-meter processing.
  ```
-
- GPS.c
- ```
   UART GNSS interface, NMEA GGA/RMC parser, position and UTC data.
  ```
-
-
- Communication
- ***
-
- wifi_ap.c
- ```
   ESP32-S3 Wi-Fi Access Point.
  ```
-
- webserver.c
- ```
   HTTP server used to provide the EFIS web interface.
  ```
-
- websocket.c
- ```
   Periodic JSON telemetry transmission and reception of user commands.
  ```
-
-
- User interface
- ***
-
- index.html
- ```
   Complete graphical EFIS interface using HTML, CSS, Canvas and SVG.
  ```
-
-
- Main application
- ***
-
- main.c
- ```
   System initialization and test-signal generation.
  ```
-
-
- =============================================================================
- COMMUNICATION AND USER SETTINGS
- =============================================================================
-
- The ESP32-S3 operates as a Wi-Fi Access Point.
-
- A smartphone, tablet or PC connects directly to the ESP32-S3 and loads the
- EFIS web interface from the integrated HTTP server.
-
- Telemetry is periodically transmitted as JSON through a WebSocket
- connection.
-
- The web interface can also send commands to the ESP32-S3 for:
-
- - QNH increase / decrease
- - Pitch-offset adjustment
- - Heading-offset adjustment
- - G-meter reset
-
- Persistent user settings are stored in NVS.
-
-
- =============================================================================
- FUTURE EXTENSIONS
- =============================================================================
-
- - Airspeed indicator
- - Pitot / static pressure sensor
- - Density altitude
- - GPS ground-speed indication
- - GPS ground-track integration
- - Flight recorder
- - SD card logging
- - Audible alarms
- - AHRS calibration page
- - Sensor diagnostics page
- - Terrain awareness
- - METAR / QNH acquisition through an Internet-connected device
- ******************\*\*******************\*\*******************\*\*******************/

https://thelastoutpostworkshop.github.io/ESPConnect/
