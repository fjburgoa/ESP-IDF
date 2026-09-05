#include <stdio.h>

#include "driver/i2c_master.h"
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
 * Frecuencia del bus I2C.
 */
#define I2C_FREQ_HZ 400000

/*
 * Tiempo máximo de espera durante el sondeo de una dirección I2C.
 */
#define I2C_TIMEOUT_MS 50

static const char *TAG = "I2C_SCAN";

/*
 * Handle del bus I2C master.
 */
static i2c_master_bus_handle_t i2c_bus_handle;

/* -------------------------------------------------------------------------- */
/* Inicialización del bus I2C                                                 */
/* -------------------------------------------------------------------------- */

static esp_err_t i2c_master_init(void)
{
    i2c_master_bus_config_t bus_config = {

        /* Selección automática de la fuente de reloj */
        .clk_source = I2C_CLK_SRC_DEFAULT,

        /* Puerto I2C utilizado */
        .i2c_port = I2C_PORT,

        /* Pines SDA y SCL */
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,

        /*
         * Filtrado de pequeños glitches de la señal.
         * Valor típico recomendado por Espressif.
         */
        .glitch_ignore_cnt = 7,

        /*
         * Habilita las resistencias pull-up internas.
         *
         * Para un montaje real es preferible disponer además
         * de resistencias pull-up externas.
         */
        .flags.enable_internal_pullup = true,
    };

    return i2c_new_master_bus(
        &bus_config,
        &i2c_bus_handle);
}

/* -------------------------------------------------------------------------- */
/* Comprobación de una dirección I2C                                          */
/* -------------------------------------------------------------------------- */

static esp_err_t i2c_probe(uint8_t address)
{
    return i2c_master_probe(
        i2c_bus_handle,
        address,
        I2C_TIMEOUT_MS);
}

/* -------------------------------------------------------------------------- */
/* Programa principal                                                         */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando bus I2C...");

    ESP_ERROR_CHECK(i2c_master_init());

    ESP_LOGI(TAG,
             "SDA = GPIO%d, SCL = GPIO%d",
             I2C_SDA_GPIO,
             I2C_SCL_GPIO);

    ESP_LOGI(TAG, "Escaneando bus I2C...");

    int devices = 0;

    /*
     * Se recorren las direcciones I2C de 7 bits utilizables.
     *
     * 0x00...0x07 y 0x78...0x7F están reservadas.
     */
    for (uint8_t address = 0x08; address <= 0x77; address++)
    {
        esp_err_t err = i2c_probe(address);

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG,
                     "Dispositivo encontrado en 0x%02X",
                     address);

            devices++;
        }
    }

    ESP_LOGI(TAG,
             "Fin del escaneo. Dispositivos encontrados: %d",
             devices);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}