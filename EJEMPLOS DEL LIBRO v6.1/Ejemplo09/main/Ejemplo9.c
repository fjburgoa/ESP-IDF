#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define LED6 4

#define LEDC_MODE           LEDC_LOW_SPEED_MODE 

void app_main(void) 
{
    // configura salida digital
    gpio_set_direction(LED6,GPIO_MODE_OUTPUT);

    // configura y habilita el canal PWM
    ledc_channel_config_t ledc_channel = {
        .speed_mode       = LEDC_MODE,  
        .channel          = LEDC_CHANNEL_0,
        .timer_sel        = LEDC_TIMER_0,
        .intr_type        = LEDC_INTR_DISABLE,
        .gpio_num         = LED6,   
        .duty             = 0,   
    };
    ledc_channel_config(&ledc_channel);

   // configura y habilita el timer asociado al canal
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_TIMER_10_BIT, // 10 bits de resolución 0..1023
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 8000,              // 8 kHz de frecuencia
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
  
    uint32_t dutycycle = 0;
  
    while (1) 
    {
        // actualiza el ancho de pulso
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, dutycycle);   
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);       

        dutycycle += 32;          // Incremento gradual en 32 escalones
        if (dutycycle>1023)       // 10 bits de resolución -> max = 1023   
            dutycycle = 0;

        // delay de 200ms
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
