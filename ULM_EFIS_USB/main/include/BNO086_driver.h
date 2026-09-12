#ifndef BNO086_DRIVER_H
#define BNO086_DRIVER_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BNO086_REPORT_ACCELEROMETER         0x01U
#define BNO086_REPORT_GYROSCOPE             0x02U
#define BNO086_REPORT_LINEAR_ACCELERATION   0x04U
#define BNO086_REPORT_GRAVITY               0x06U
#define BNO086_REPORT_GAME_ROTATION_VECTOR  0x08U

#define BNO086_SHTP_CHANNEL_REPORTS         3U
#define BNO086_SHTP_REPORT_BASE_TIMESTAMP   0xFBU

esp_err_t bno086_driver_init(void);

esp_err_t bno086_driver_receive_packet(
    uint8_t *packet,
    size_t capacity,
    uint16_t *packet_len);

#ifdef __cplusplus
}
#endif

#endif /* BNO086_DRIVER_H */
