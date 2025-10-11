/*
06.09.2025
este programa incluye:
- control de 10 Leds que van cambiando cada poco tiempo
*/

#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PULS 0
#define LED0 47
#define LED1 48
#define LED2 45
#define LED3 35
#define LED4 36
#define LED5 37
#define LED6 38
#define LED7 7
#define LED8 6
#define LED9 4
#define N 10

#define OFF 1
#define ON  0


int DO[N] = {LED0,LED1,LED2,LED3,LED4,LED5,LED6,LED7,LED8,LED9};


void app_main(void)
{
    
    gpio_set_direction(PULS,GPIO_PULLUP_ENABLE);
    for (int i=0;i<N;i++)
    {
        gpio_set_direction(DO[i],GPIO_MODE_OUTPUT);
    }
    
    int k = 0;
    int state = 0;

    while(1)
    {

        vTaskDelay(250/portTICK_PERIOD_MS);
        
        gpio_set_level(DO[k],state);
        printf("Led a %d\n",k);
        k++;

        if (k>(N-1))
        {            
            k = 0; 
            state = !state;    
        }
    }
}
