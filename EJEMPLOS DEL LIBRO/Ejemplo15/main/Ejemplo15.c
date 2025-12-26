#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "led_strip.h"

#define LED_GPIO 48

void app_main(void)
{
    gpio_reset_pin(LED_GPIO);                       // Borra config. previa LED_GPIO
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT); // GPIO 48 conectado al LED 

    led_strip_handle_t led_strip = NULL;            // Handler

    // Configura el LED 
    led_strip_config_t strip_config =
    {
        .strip_gpio_num         = LED_GPIO,               
        .max_leds               = 1,                                 // 1 LED   
        .led_model              = LED_MODEL_WS2812,                  // Modelo LED
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // Formato RGB 
        .flags = {
            .invert_out = false, // no se invierte la señal
        }
    };

    // Configura el driver RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,    // Fuente del reloj
        .resolution_hz     = 10 * 1000 * 1000,       // Frecuencia: 10 MHz
        .mem_block_symbols = 64,                     // tamaño de la memoria de 
                                                     // cada canal RMT
        .flags = {
            .with_dma = false,                       // DMA no es necesario
        }
    };

    //registra RMT, configuración y handler
    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);

    while(1)
    {
        led_strip_set_pixel(led_strip, 0, 255, 0, 0); //rojo
        led_strip_refresh(led_strip);

        vTaskDelay(pdMS_TO_TICKS (1000));      

        led_strip_set_pixel(led_strip, 0, 0, 255, 0); //verde
        led_strip_refresh(led_strip);

        vTaskDelay(pdMS_TO_TICKS (1000));      

        led_strip_set_pixel(led_strip, 0, 0, 0, 255); //azul
        led_strip_refresh(led_strip);

        vTaskDelay(pdMS_TO_TICKS (1000));      
    }
}
