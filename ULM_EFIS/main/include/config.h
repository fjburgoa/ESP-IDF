#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

/*
 * ============================================================================
 * ULM-EFIS - Configuracion general
 * ============================================================================
 *
 * Este fichero concentra los parametros ajustables del sistema.
 *
 * Criterio:
 *   - Aqui: periodos, filtros, rangos, modos, pines, velocidades, limites,
 *     prioridades, buffers y parametros de interfaz.
 *   - En cada driver: direcciones de registros, identificadores de chip,
 *     factores de escala y constantes propias del protocolo.
 */

/* ============================================================================
 * Opciones generales
 * ========================================================================== */

#define DATALOGGER_ENABLED 1

/*
 * Registro inercial circular en PSRAM.
 * 30000 muestras / 25 Hz = 1200 s = 20 minutos conservados.
 */
#define DATALOGGER_PERIOD_MS 40U
#define DATALOGGER_MAX_SAMPLES 30000U
#define DATALOGGER_TASK_STACK_SIZE 4096U
#define DATALOGGER_TASK_PRIORITY 4U

/*
 * Heading mostrado:
 *   1 = Ground Track del GPS
 *   0 = heading procedente de la IMU / integracion
 */
#define HEADING_GPS 1

/* ============================================================================
 * Bus I2C general
 * ========================================================================== */

#define EFIS_I2C_PORT I2C_NUM_0
#define EFIS_I2C_SCL_GPIO GPIO_NUM_9
#define EFIS_I2C_SDA_GPIO GPIO_NUM_8
#define EFIS_I2C_FREQ_HZ 400000U
#define EFIS_I2C_TIMEOUT_MS 100U
#define EFIS_I2C_MUTEX_TIMEOUT_MS 150U

/* ============================================================================
 * BNO086
 * ========================================================================== */

#define BNO086_I2C_ADDRESS 0x4BU

/* Pines cableados al BNO086. H_INTN y RESET_N son activos a nivel bajo. */
#define BNO086_INT_GPIO GPIO_NUM_4
#define BNO086_RESET_GPIO GPIO_NUM_6

/*
 * El BNO086 se deja inicialmente a 100 kHz para maximizar margen eléctrico
 * y temporal. Puede volver a 400 kHz cuando el bus esté validado.
 */
#define BNO086_I2C_FREQ_HZ 400000U
#define BNO086_SCL_WAIT_US 20000U

/* 25 Hz. */
#define BNO086_REPORT_INTERVAL_US 40000U

#define BNO086_PACKET_BUFFER_SIZE 128U
#define BNO086_I2C_READ_TIMEOUT_MS 30U
#define BNO086_STARTUP_FLUSH_PACKETS 32U

/* Reset hardware y espera de arranque SH-2. */
#define BNO086_RESET_LOW_MS 20U
#define BNO086_BOOT_TIMEOUT_MS 1500U
#define BNO086_STARTUP_PACKET_GAP_MS 2U

/* Recuperación automática tras errores consecutivos de SHTP/I2C. */
#define BNO086_MAX_CONSECUTIVE_ERRORS 3U
#define BNO086_RECOVERY_DELAY_MS 50U

/* La tarea se despierta por H_INTN; ya no realiza polling periódico I2C. */
#define BNO086_TASK_STACK_SIZE 5120U
#define BNO086_TASK_PRIORITY 5U

/* Régimen de giro. */
#define TURN_RATE_FILTER_TAU_S 2.0f
#define TURN_RATE_DEADBAND_DPS 0.10f

/*
 * Bola de resbale:
 * el BNO086 proporciona directamente Linear Acceleration, con la gravedad
 * eliminada por SH-2.
 */
#define SLIP_BALL_LIMIT_DEG 25.0f
#define SLIP_BALL_DEADBAND_DEG 0.8f
#define SLIP_BALL_GAIN 1.0f

#define SLIP_BALL_FILTER_TAU_S 0.5

/* ============================================================================
 * BMP280 / BME280
 * ========================================================================== */

#define BMP280_ADDR 0x76U
#define BMP280_PERIOD_MS 100U /* 10 Hz */
#define BMP280_TASK_STACK_SIZE 4096U
#define BMP280_TASK_PRIORITY 5U

/*
 * REG_CONFIG:
 *   t_sb   = 001 -> 62.5 ms
 *   filter = 100 -> IIR x16
 *   spi3w  = 0
 */
#define BMP280_CONFIG_VALUE 0x30U

/*
 * REG_CTRL_MEAS:
 *   temperatura x1
 *   presion x4
 *   modo normal
 */
#define BMP280_CTRL_MEAS_VALUE 0x2FU

#define BMP280_RESET_VALUE 0xB6U
#define BMP280_RESET_DELAY_MS 100U

/* Variometro. */
#define VSI_FILTER_TAU_S 1.0f
#define VSI_LIMIT_FPM 4000.0f

/* ============================================================================
 * GPS / GNSS
 * ========================================================================== */

#define GPS_UART_PORT UART_NUM_1
#define GPS_UART_TX_GPIO 5
#define GPS_UART_RX_GPIO 7

/*
 * 1: el receptor arranca ya a 115200 baud.
 * 0: se intenta arrancar a 9600 y despues cambiar a 115200.
 */
#define GPS_115200 1

#if GPS_115200
#define GPS_INITIAL_BAUD_RATE 115200U
#else
#define GPS_INITIAL_BAUD_RATE 9600U
#endif

#define GPS_TARGET_BAUD_RATE 115200U

/* 200 ms = 5 Hz; 100 ms = 10 Hz. */
#define GPS_TARGET_RATE_MS 200U

#define GPS_UART_RX_BUFFER_SIZE 2048U
#define GPS_UART_TX_BUFFER_SIZE 256U
#define GPS_LINE_BUFFER_SIZE 192U
#define GPS_READ_BUFFER_SIZE 128U

#define GPS_TASK_STACK_SIZE 5120U
#define GPS_TASK_PRIORITY 5U

#define GPS_DETECT_TIMEOUT_MS 4000U
#define GPS_VERIFY_TIMEOUT_MS 2500U
#define GPS_ACK_TIMEOUT_MS 800U

/* Tiempos internos de configuracion UART/UBX. */
#define GPS_UART_TX_TIMEOUT_MS 250U
#define GPS_UART_READ_TIMEOUT_MS 20U
#define GPS_BAUD_CHANGE_PRE_DELAY_MS 100U
#define GPS_BAUD_CHANGE_POST_DELAY_MS 150U
#define GPS_NMEA_READ_TIMEOUT_MS 100U

/* Ground Track solo se considera fiable por encima de esta velocidad. */
#define GPS_HEADING_MIN_SPEED_KT 4.0f

/* ============================================================================
 * Arranque general
 * ========================================================================== */

#define STARTUP_GPS_TO_I2C_DELAY_MS 1500U
#define STARTUP_I2C_TO_BNO_DELAY_MS 500U
#define STARTUP_BNO_TO_BMP_DELAY_MS 500U

/* Potencia WiFi en unidades de 0.25 dBm: 32 -> 8 dBm. */
#define WIFI_TX_POWER_2_DBM 8
#define WIFI_TX_POWER_5_DBM 20
#define WIFI_TX_POWER_7_DBM 28
#define WIFI_TX_POWER_8_5_DBM 34
#define WIFI_TX_POWER_11_DBM 44
#define WIFI_TX_POWER_13_DBM 52
#define WIFI_TX_POWER_14_DBM 56
#define WIFI_TX_POWER_15_DBM 60
#define WIFI_TX_POWER_16_5_DBM 66
#define WIFI_TX_POWER_18_DBM 72
#define WIFI_TX_POWER_20_DBM 80

#define WIFI_TX_POWER_DBM WIFI_TX_POWER_7_DBM

/* ============================================================================
 * Modo de prueba de altitud / VSI
 * ========================================================================== */

#define VSI_TEST_MODE 0
#define ALTITUDE_TEST_PERIOD_MS 80U
#define ALTITUDE_TEST_RATE_FPM 500.0f
#define ALTITUDE_TEST_MIN_M 700.0f
#define ALTITUDE_TEST_MAX_M 760.0f
#define ALTITUDE_TEST_TASK_STACK_SIZE 3072U
#define ALTITUDE_TEST_TASK_PRIORITY 4U

/* ============================================================================
 * WiFi Access Point
 * ========================================================================== */

#define WIFI_AP_SSID "ESP32-FlightDisplay"
#define WIFI_AP_PASSWORD "esp32s3test"
#define WIFI_AP_CHANNEL 6
#define WIFI_AP_MAX_CLIENTS 4

/* ============================================================================
 * Servidor HTTP / WebSocket
 * ========================================================================== */

#define WEBSERVER_PORT 80U
#define WEBSERVER_MAX_OPEN_SOCKETS 4U

/* Telemetría WebSocket multinivel. */
#define TELEMETRY_FAST_PERIOD_MS 40U /* F: actitud/coordinador/G = 25 Hz */
#define TELEMETRY_NAV_DIVIDER 10U    /* N: 20 Hz / 10 = 2 Hz */
#define TELEMETRY_TASK_STACK_SIZE 4096U
#define TELEMETRY_TASK_PRIORITY 5U
#define MAX_WS_CLIENTS 4U
#define JSON_BUFFER_SIZE 192U

/* Sensor interno de temperatura ESP32-S3. */
#define INTERNAL_TEMP_MIN_C 10
#define INTERNAL_TEMP_MAX_C 80

/* ============================================================================
 * Ajustes de usuario / interfaz
 * ========================================================================== */

#define QNH_STEP_HPA 0.1f
#define QNH_MIN_HPA 970.00f
#define QNH_MAX_HPA 1150.00f
#define QNH_DEFAULT_HPA 1013.2f

#define PITCH_OFFSET_DEFAULT_DEG 0
#define PITCH_OFFSET_STEP_DEG 1
#define PITCH_OFFSET_MIN_DEG -90
#define PITCH_OFFSET_MAX_DEG 90

#define HEADING_OFFSET_DEFAULT_DEG 0U
#define HEADING_OFFSET_STEP_DEG 1U
#define HEADING_OFFSET_MAX_DEG 359U

/* ============================================================================
 * NVS
 * ========================================================================== */

#define NVS_NAMESPACE "flight_cfg"
#define NVS_KEY_QNH_X100 "qnh_x100"
#define NVS_KEY_PITCH_OFFSET "pitch_off"
#define NVS_KEY_HEAD_OFFSET "head_off"
#define NVS_KEY_MOUNT_MODE "mount_mode"
#define NVS_KEY_IMU_MODE "imu_mode"
#define NVS_KEY_ATTITUDE_MODE "att_mode"

/* Selector experimental de pitch/roll */
#define ATTITUDE_DEFAULT_MODE 0U
#define ATTITUDE_ACCEL_ERROR_LOW_MS2 0.20f
#define ATTITUDE_ACCEL_ERROR_HIGH_MS2 1.50f
#define ATTITUDE_ANGLE_ERROR_LOW_DEG 3.0f
#define ATTITUDE_ANGLE_ERROR_HIGH_DEG 12.0f
#define ATTITUDE_GRAVITY_VALID_MIN_MS2 8.0f
#define ATTITUDE_GRAVITY_VALID_MAX_MS2 11.5f
#endif /* PROJECT_CONFIG_H */
