
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
 
#define BUTTON 0
#define SLEEP_TIME_SEC 10        //10 segundos para despertar de deep-sleep

RTC_SLOW_ATTR int32_t mi_variable_no_volatil;  //mantener en memoria caliente.
 
void app_main(void)
{
    //CONFIGURA GPIO0 (BOOT) COMO ENTRADA
    gpio_config_t io_conf = 
    {
        .pin_bit_mask = 1ULL << BUTTON,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf); 

    //configura timer RTC
    esp_sleep_enable_timer_wakeup(SLEEP_TIME_SEC * 1000000);

    int btn  = 0;
    int btn1 = 0;

    while(1)
    {        
        btn = gpio_get_level(BUTTON);

        //detecta flanco de bajada en BOOT
        if ((btn!=btn1)&&(btn==0))    
        {
            printf("Entrando en modo Sleep\n");

            //entra en modo Deep-Sleep 
            esp_deep_sleep_start();
        }
        btn1 = btn;

        printf("Décimas de segundo  %lu\n", mi_variable_no_volatil++); 

        //delay de 100ms   
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

