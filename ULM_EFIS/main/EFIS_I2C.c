/**
 * @file EFIS_I2C.c
 * @brief Bus I2C maestro compartido del ULM-EFIS para ESP-IDF 6.1.
 */

#include <stddef.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "EFIS_I2C.h"
#include "config.h"

static const char *TAG = "EFIS_I2C";
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static SemaphoreHandle_t s_i2c_mutex = NULL;

esp_err_t efis_i2c_init(void)
{
    if (s_i2c_bus != NULL)
    {
        return ESP_OK;
    }

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = EFIS_I2C_PORT,
        .sda_io_num = EFIS_I2C_SDA_GPIO,
        .scl_io_num = EFIS_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_RETURN_ON_ERROR(
        i2c_new_master_bus(&bus_config, &s_i2c_bus),
        TAG,
        "No se pudo crear el bus I2C");

    s_i2c_mutex = xSemaphoreCreateMutex();
    if (s_i2c_mutex == NULL)
    {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG,
             "I2C inicializado: puerto=%d SDA=%d SCL=%d",
             (int)EFIS_I2C_PORT,
             (int)EFIS_I2C_SDA_GPIO,
             (int)EFIS_I2C_SCL_GPIO);

    return ESP_OK;
}

i2c_master_bus_handle_t efis_i2c_get_bus(void)
{
    return s_i2c_bus;
}

esp_err_t efis_i2c_lock(uint32_t timeout_ms)
{
    if (s_i2c_mutex == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const TickType_t timeout_ticks =
        (timeout_ms == 0U) ? 0U : pdMS_TO_TICKS(timeout_ms);

    if (xSemaphoreTake(s_i2c_mutex, timeout_ticks) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

void efis_i2c_unlock(void)
{
    if (s_i2c_mutex != NULL)
    {
        xSemaphoreGive(s_i2c_mutex);
    }
}

esp_err_t efis_i2c_add_device_ex(
    uint16_t address,
    uint32_t scl_speed_hz,
    uint32_t scl_wait_us,
    i2c_master_dev_handle_t *device_handle)
{
    if ((s_i2c_bus == NULL) || (device_handle == NULL))
    {
        return ESP_ERR_INVALID_STATE;
    }

    const i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = scl_speed_hz,
        .scl_wait_us = scl_wait_us,
    };

    return i2c_master_bus_add_device(
        s_i2c_bus,
        &dev_config,
        device_handle);
}

esp_err_t efis_i2c_add_device(
    uint16_t address,
    i2c_master_dev_handle_t *device_handle)
{
    return efis_i2c_add_device_ex(
        address,
        EFIS_I2C_FREQ_HZ,
        0U,
        device_handle);
}

esp_err_t efis_i2c_probe(uint16_t address)
{
    if (s_i2c_bus == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(
        efis_i2c_lock(EFIS_I2C_MUTEX_TIMEOUT_MS),
        TAG,
        "Timeout esperando mutex I2C");

    const esp_err_t err = i2c_master_probe(
        s_i2c_bus,
        address,
        EFIS_I2C_TIMEOUT_MS);

    efis_i2c_unlock();

    return err;
}

esp_err_t efis_i2c_reset_bus(void)
{
    if (s_i2c_bus == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(
        efis_i2c_lock(EFIS_I2C_MUTEX_TIMEOUT_MS),
        TAG,
        "Timeout esperando mutex I2C para reset");

    const esp_err_t err = i2c_master_bus_reset(s_i2c_bus);

    efis_i2c_unlock();

    if (err == ESP_OK)
    {
        ESP_LOGW(TAG, "Bus I2C recuperado mediante i2c_master_bus_reset()");
    }

    return err;
}
