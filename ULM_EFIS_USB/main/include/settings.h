#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>
#include "esp_err.h"

esp_err_t settings_init(void);
int32_t settings_get_pitch_offset_deg(void);
esp_err_t settings_set_pitch_offset_deg(int32_t value);

typedef enum
{
    MOUNT_VERTICAL = 0,
    MOUNT_HORIZONTAL = 1
} mount_mode_t;

extern volatile mount_mode_t s_mount_mode;

#endif
