#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "esp_timer.h"
 


#define BUTTON 0
#define SLEEP_TIME_SEC 7

RTC_SLOW_ATTR int32_t mi_variable_no_volatil;
 
//---------------------------------------------------------------------------------
void app_main(void)
{

    
    int64_t sleep_time = esp_timer_get_time(); // µs
    printf("Tiempo de deep sleep: %lld us\n", sleep_time);

    //CONFIGURA GPIO0 COMO ENTRADA
    gpio_config_t io_conf = 
    {
        .pin_bit_mask = 1ULL << BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf); 

    esp_sleep_enable_timer_wakeup(SLEEP_TIME_SEC * 1000000);


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
