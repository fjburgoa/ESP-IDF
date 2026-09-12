#include "settings.h"

#include "config.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "BNO055.h"

/* Única variable global de orientación. Siempre arranca en vertical. */
volatile mount_mode_t s_mount_mode = MOUNT_VERTICAL;

static int32_t s_pitch_offset_deg = PITCH_OFFSET_DEFAULT_DEG;

static int32_t clamp_offset(int32_t value)
{
    if (value < PITCH_OFFSET_MIN_DEG)
        return PITCH_OFFSET_MIN_DEG;
    if (value > PITCH_OFFSET_MAX_DEG)
        return PITCH_OFFSET_MAX_DEG;
    return value;
}

esp_err_t settings_init(void)
{
    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) ||
        (err == ESP_ERR_NVS_NEW_VERSION_FOUND))
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK)
        return err;

    nvs_handle_t handle;
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND)
        return ESP_OK;
    if (err != ESP_OK)
        return err;

    int32_t stored = PITCH_OFFSET_DEFAULT_DEG;
    if (nvs_get_i32(handle, NVS_KEY_PITCH_OFFSET, &stored) == ESP_OK)
    {
        s_pitch_offset_deg = clamp_offset(stored);
    }
    nvs_close(handle);
    return ESP_OK;
}

int32_t settings_get_pitch_offset_deg(void)
{
    return s_pitch_offset_deg;
}

esp_err_t settings_set_pitch_offset_deg(int32_t value)
{
    value = clamp_offset(value);
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;
    err = nvs_set_i32(handle, NVS_KEY_PITCH_OFFSET, value);
    if (err == ESP_OK)
        err = nvs_commit(handle);
    nvs_close(handle);
    if (err == ESP_OK)
        s_pitch_offset_deg = value;
    return err;
}
