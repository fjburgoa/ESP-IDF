/*
Programa Realizado por Javier Burgoa
Manejo del timer de rango extendido
*/

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_timer.h"

#define LED1 4
#define LED2 5
#define PULS 0

int state_led_1 = 0;
int state_led_2 = 0;
int cont  = 0;

esp_timer_handle_t myESP_Timer;

void ESP_TimerCallback (void* arg);

void app_main(void)
{    
    gpio_set_direction(PULS,GPIO_PULLUP_ENABLE);
    gpio_set_direction(LED1,GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2,GPIO_MODE_OUTPUT);
    
    const esp_timer_create_args_t My_ESP_Timer_Configuration = 
    {
        .callback = &ESP_TimerCallback,                //callback            
        .name     = "ISR ESP_Timer "                   //name    
    };

    //configure and start timer
    esp_timer_create(&My_ESP_Timer_Configuration, &myESP_Timer);
    esp_timer_start_periodic(myESP_Timer, 500000);   //useg

    print_config_timers();

    while(1)
    {
        int my_puls = gpio_get_level(PULS);
        if (my_puls)
            gpio_set_level(LED2,0);             
        else
            gpio_set_level(LED2,1);                 
    }
}

void ESP_TimerCallback (void* arg)
{
    printf("ESP Timer: %d\n",cont++);
    gpio_set_level(LED1,state_led_1);
    state_led_1 = !state_led_1;
}