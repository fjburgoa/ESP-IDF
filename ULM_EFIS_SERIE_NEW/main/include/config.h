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

#define DATALOGGER_ENABLED 0

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

/* ============================================================================
 * BNO055 - tarea y modo de operacion
 * ========================================================================== */

/*
 * Modo IMU por defecto si NVS no contiene una seleccion previa:
 *   0 = BRUTO
 *   1 = PROCESADO
 *
 * Despues del arranque puede cambiarse desde la interfaz web.
 */
#define BNO055_DEFAULT_OPERATION_MODE 1U

#define BNO055_PERIOD_MS 40U /* 25 Hz */
// #define BNO055_PERIOD_MS 20U /* 50 Hz */
#define BNO055_TASK_STACK_SIZE 5120U
#define BNO055_TASK_PRIORITY 5U

/* Tiempos de inicializacion/cambio de modo. */
#define BNO055_STARTUP_DELAY_MS 1500U

/*
 * Deteccion robusta durante el arranque.
 * El BNO055 de algunos modulos puede tardar varios segundos en responder
 * correctamente despues del power-on. Durante este intervalo el ESP32 no se
 * reinicia: se repite exclusivamente la deteccion del BNO055.
 */
#define BNO055_DETECT_RETRY_MS 500U
#define BNO055_DETECT_TIMEOUT_MS 30000U

/*
 * CHIP_ID puede empezar a responder antes de que el dispositivo acepte de
 * forma estable escrituras en OPR_MODE. Se deja un pequeño margen entre la
 * deteccion y el primer acceso de configuracion.
 */
#define BNO055_DETECT_SETTLE_MS 100U
#define BNO055_CONFIG_MODE_DELAY_MS 25U
#define BNO055_OPERATION_MODE_DELAY_MS 100U
#define BNO055_POWER_MODE_DELAY_MS 10U
#define BNO055_POST_INIT_DELAY_MS 500U
#define BNO055_MOUNT_CHANGE_SETTLE_MS 150U

#define BNOTESTMODE 1

/* Botón BOOT usado por BNO055_test.c para recorrer los modos de operación. */
#define BNO055_TEST_BOOT_GPIO GPIO_NUM_0
#define BNO055_TEST_BOOT_DEBOUNCE_MS 150U

/* ============================================================================
 * BNO055 - configuracion del acelerometro y giroscopo en AMG
 * ========================================================================== */

/*
 *
 * Valores utiles para ensayo:
 *   0x0C -> +/-2 g, 62.5 Hz
 *   0x0D -> +/-4 g, 62.5 Hz
 *   0x0E -> +/-8 g, 62.5 Hz
 */
#define BNO055_ACC_CONFIG_EFIS 0x0DU

/*
 * GYR_CONFIG_0 = 0x39 -> +/-1000 deg/s, 32 Hz
 * GYR_CONFIG_0 = 0x3A -> +/-500 deg/s, 32 Hz
 */
#define BNO055_GYR_CONFIG_0_EFIS 0x39U

/* GYR_CONFIG_1: Normal power mode. */
#define BNO055_GYR_CONFIG_1_EFIS 0x00U

/* ============================================================================
 * BNO055 - estimacion de actitud
 * ========================================================================== */

#define BNO055_ATTITUDE_TAU_S 0.3f
#define BNO055_PITCH_GYRO_SIGN 1.0f
#define BNO055_ROLL_GYRO_SIGN -1.0f

/* ============================================================================
 * BNO055 - filtros de procesado
 * ========================================================================== */

/* LPF previo de gyro X/Y/Z. */
#define GYRO_FILTER_TAU_S 0.6f

/* Regimen de giro. */
#define TURN_RATE_FILTER_TAU_S 0.9f
#define TURN_RATE_DEADBAND_DPS 0.10f

/* Bola de resbale/deslizamiento. */
#define SLIP_BALL_FILTER_TAU_S 1.5f
#define SLIP_BALL_LIMIT_DEG 25.0f
#define SLIP_BALL_DEADBAND_DEG 0.8f
#define SLIP_BALL_GAIN 5.0f

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
#define GPS_HEADING_MIN_SPEED_KT 5.0f

/* ============================================================================
 * Arranque general
 * ========================================================================== */

#define STARTUP_GPS_TO_I2C_DELAY_MS 1500U
#define STARTUP_I2C_TO_BNO_DELAY_MS 500U
#define STARTUP_BNO_TO_BMP_DELAY_MS 500U

/* Potencia WiFi en unidades de 0.25 dBm: 32 -> 8 dBm. */
#define WIFI_TX_POWER_QDBM 32

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

#define TELEMETRY_PERIOD_MS 66U /* ~15 Hz */
#define TELEMETRY_TASK_STACK_SIZE 4096U
#define TELEMETRY_TASK_PRIORITY 5U
#define MAX_WS_CLIENTS 4U
#define JSON_BUFFER_SIZE 1600U

/* Sensor interno de temperatura ESP32-S3. */
#define INTERNAL_TEMP_MIN_C 10
#define INTERNAL_TEMP_MAX_C 80

/* ============================================================================
 * Ajustes de usuario / interfaz
 * ========================================================================== */

#define QNH_STEP_HPA 0.05f
#define QNH_MIN_HPA 970.00f
#define QNH_MAX_HPA 1150.00f
#define QNH_DEFAULT_HPA 1013.25f

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

extern SemaphoreHandle_t xMutex;

#endif /* PROJECT_CONFIG_H */
