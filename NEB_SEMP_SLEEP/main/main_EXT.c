#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
 
#define BUTTON 0
#define SLEEP_TIME_SEC 10
#define EXT_WAKE_UP_PIN 7

RTC_SLOW_ATTR int32_t mi_variable_no_volatil;
 
//---------------------------------------------------------------------------------
void app_main(void)
{
    //CONFIGURA GPIO0 COMO ENTRADA
    gpio_config_t io_conf = 
    {
        .pin_bit_mask = 1ULL << BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf); 


    printf("Enabling EXT0 wakeup on pin GPIO%d\n", EXT_WAKE_UP_PIN);
    esp_sleep_enable_ext0_wakeup(EXT_WAKE_UP_PIN , 1);
    rtc_gpio_pullup_dis(EXT_WAKE_UP_PIN);
    rtc_gpio_pulldown_en(EXT_WAKE_UP_PIN);
    
    int btn  = 0;
    int btn1 = 0;

    while(1)
    {        
        btn1 = btn;
        btn = gpio_get_level(BUTTON);
        if ((btn!=btn1)&&(btn==0))    
        {
            printf("Entrando en modo Sleep\n");
            esp_deep_sleep_start();
        }

        printf("Décimas de segundo  %lu\n", mi_variable_no_volatil++);    
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
