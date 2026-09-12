#ifndef EFIS_USB_CONFIG_H
#define EFIS_USB_CONFIG_H

#include "driver/i2c_types.h"

/* -------------------------------------------------------------------------- */
/* Selección de placa                                                         */
/* -------------------------------------------------------------------------- */

/*
 * PlacaLarga: BNO055 + BNO086
 * PlacaCorta: BNO055 + MPU6050
 *
 * Dejar definida exactamente una de las dos.
 */
#define PlacaLarga
// #define PlacaCorta

#if defined(PlacaLarga) && defined(PlacaCorta)
#error "Defina solo una placa: PlacaLarga o PlacaCorta"
#endif

#if !defined(PlacaLarga) && !defined(PlacaCorta)
#error "Debe definir PlacaLarga o PlacaCorta"
#endif


#define EFIS_I2C_PORT I2C_NUM_0
#define EFIS_I2C_SDA_GPIO 8
#define EFIS_I2C_SCL_GPIO 9
#define EFIS_I2C_FREQ_HZ 400000U
#define EFIS_I2C_TIMEOUT_MS 1000

#define MPU6050_PERIOD_MS 40U

/* -------------------------------------------------------------------------- */
/* BNO086                                                                      */
/* -------------------------------------------------------------------------- */

#define BNO086_I2C_ADDRESS 0x4BU

/* 25 Hz -> 40 000 us */
#define BNO086_REPORT_INTERVAL_US 40000U

/*
 * Los reports de sensores son pequeños. El código de recepción consume
 * también correctamente los paquetes SHTP grandes de arranque.
 */
#define BNO086_PACKET_BUFFER_SIZE 128U
#define BNO086_I2C_READ_TIMEOUT_MS 50
#define BNO086_RESET_DELAY_MS 300U
#define BNO086_STARTUP_FLUSH_PACKETS 20U

#define BNO086_TASK_PERIOD_MS 2U
#define BNO086_TASK_STACK_SIZE 4096U
#define BNO086_TASK_PRIORITY 5U

#define BNO055_PERIOD_MS 40U
#define BNO055_TASK_STACK_SIZE 4096U
#define BNO055_TASK_PRIORITY 5U

#define BNO055_STARTUP_DELAY_MS 1500U
#define BNO055_DETECT_RETRY_MS 500U
#define BNO055_DETECT_TIMEOUT_MS 30000U
#define BNO055_DETECT_SETTLE_MS 100U
#define BNO055_CONFIG_MODE_DELAY_MS 25U
#define BNO055_OPERATION_MODE_DELAY_MS 100U
#define BNO055_POWER_MODE_DELAY_MS 10U
#define BNO055_POST_INIT_DELAY_MS 500U

/*
 * ACC_CONFIG = 0x0D = 0000 1101
 *
 * Bits [1:0] ACC_Range     = 01  -> ±4 g
 * Bits [4:2] ACC_Bandwidth = 011 -> 62,5 Hz
 * Bits [7:5] ACC_PowerMode = 000 -> Normal
 */
#define BNO055_ACC_CONFIG_EFIS 0x0DU

/*
 * GYR_CONFIG_0 = 0x39 = 0011 1001
 * Bits [2:0] GYR_Range     = 001 -> ±1000 °/s
 * Bits [5:3] GYR_Bandwidth = 111 -> 32 Hz
 *
 * GYR_CONFIG_0 = 0x3B
 * Bits [5:3] = 111 -> 32 Hz
 * Bits [2:0] = 011 -> ±250 °/s
 *
 */
#define BNO055_GYR_CONFIG_0_EFIS 0x39U

// #define BNO055_GYR_CONFIG_0_EFIS 0x3BU

/*
 * GYR_CONFIG_1 = 0x00
 *
 * Bits [2:0] GYR_PowerMode = 000 -> Normal
 */
#define BNO055_GYR_CONFIG_1_EFIS 0x00U

#define NVS_NAMESPACE "flight_cfg"
#define NVS_KEY_PITCH_OFFSET "pitch_off"
#define PITCH_OFFSET_DEFAULT_DEG 0
#define PITCH_OFFSET_MIN_DEG -30
#define PITCH_OFFSET_MAX_DEG 30

#define USB_TELEMETRY_PERIOD_MS 20U

#endif
