#include <stdio.h>
#include "driver/gpio.h"
#include "esp_timer.h"

#define LED4 4

int state_led_1 = 0;
int cont = 0;

esp_timer_handle_t myESP_Timer;

void ESP_TimerCallback (void* arg);

//----------app_main ---------------------------------------------------------
void app_main(void)
{    
    gpio_set_direction(LED4, GPIO_MODE_OUTPUT);
    
    const esp_timer_create_args_t My_ESP_Timer_Configuration = 
    {
        .callback = &ESP_TimerCallback,              //función callback            
        .name     = "ISR ESP_Timer "                 //nombre    
    };

    //configura y arranca el callback asociado al System Timer
    esp_timer_create(&My_ESP_Timer_Configuration, &myESP_Timer);
    esp_timer_start_periodic(myESP_Timer, 500000);   //500000 µs = 0,5s

    while(1)
    {
         //Bucle principal
    }
}
//----------callback asociado al System Timer---------------------------------
void ESP_TimerCallback (void* arg)
{
    printf("ESP Timer: %d\n",cont++);
    
    gpio_set_level(LED4,state_led_1);
    state_led_1 = !state_led_1;
}
