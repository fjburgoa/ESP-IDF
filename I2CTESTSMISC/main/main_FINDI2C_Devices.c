#include <stdio.h>

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* -------------------------------------------------------------------------- */
/* I2C                                                                        */
/* -------------------------------------------------------------------------- */

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_8
#define I2C_SCL_GPIO GPIO_NUM_9

/*
 * Los u-blox M10 utilizan I2C Fast-mode.
 */
#define I2C_FREQ_HZ 400000

static const char *TAG = "I2C_SCAN";

/* -------------------------------------------------------------------------- */

static esp_err_t i2c_master_init(void)
{
    const i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &config));

    return i2c_driver_install(
        I2C_PORT,
        config.mode,
        0,
        0,
        0);
}

/* -------------------------------------------------------------------------- */

static esp_err_t i2c_probe(uint8_t address)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);

    i2c_master_write_byte(
        cmd,
        (address << 1) | I2C_MASTER_WRITE,
        true);

    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(
        I2C_PORT,
        cmd,
        pdMS_TO_TICKS(50));

    i2c_cmd_link_delete(cmd);

    return err;
}

/* -------------------------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando I2C a 400 kHz...");

    ESP_ERROR_CHECK(i2c_master_init());

    ESP_LOGI(TAG, "Escaneando bus I2C...");

    int devices = 0;

    for (uint8_t address = 0x08; address <= 0x77; address++)
    {
        if (i2c_probe(address) == ESP_OK)
        {
            ESP_LOGI(TAG, "Dispositivo encontrado en 0x%02X", address);
            devices++;
        }
    }

    ESP_LOGI(TAG, "Fin del escaneo. Dispositivos encontrados: %d", devices);

    while (1)
        vTaskDelay(pdMS_TO_TICKS(1000));
}