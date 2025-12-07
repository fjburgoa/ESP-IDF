 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "led_strip.h"
//#include "sdkconfig.h"

#define LED_GPIO 48

void app_main(void)
{

    gpio_reset_pin(LED_GPIO);                       // Borra configuración previa Pin GPIO 48 conectado al LED
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT); // Pin GPIO 48 conectado al LED como salida

    led_strip_handle_t led_strip = NULL;         // Handler

    led_strip_config_t strip_config =
    {
        .strip_gpio_num = LED_GPIO,              // Pin GPIO conectado al LED
        .max_leds       = 1,                     // LEDS en la tira (1),
        .led_model      = LED_MODEL_WS2812,      // Modelo de LED
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // Formato RGB 
        .flags = {
            .invert_out = false, // no se invierte la señal
        }
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,    // Fuente del reloj
        .resolution_hz     = 10 * 1000 * 1000,       // Frecuencia: 10MHz
        .mem_block_symbols = 64,                     // tamaño de la memoria de cada canal RMT
        .flags = {
            .with_dma = false, 	                     // DMA no es necesario
        }
    };

    //registra RTM, configuración y handler
    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);

    while(1)
    {
        led_strip_set_pixel(led_strip, 0, 255, 0, 0); //red
        led_strip_refresh(led_strip);

        vTaskDelay(pdMS_TO_TICKS(10));      
    }
}

