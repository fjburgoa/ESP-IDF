#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED4 4

int state = 0;

void app_main(void)
{
    //establece Entrada Salida
    gpio_set_direction(LED4, GPIO_MODE_OUTPUT);  

    while(1)
    {
        //establece Nivel
        gpio_set_level(LED4, state);
        state = !state;

        //Delay
        vTaskDelay(1000/portTICK_PERIOD_MS); 
    }
}


#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED4 4
#define PULS 0

int state     = 0;
int pulsador  = 0;
int pulsador1 = 0;

void app_main(void)
{
    //establece Entrada Salida 
    gpio_set_direction(PULS,  GPIO_PULLUP_ENABLE);
    gpio_set_direction(LED4,  GPIO_MODE_OUTPUT);

    while(1)
    {       
        int pulsador  = gpio_get_level(PULS);      //Lee input
        
        //compara actual y anterior 
        if ((pulsador != pulsador1)&&(pulsador == 0)){
            gpio_set_level(LED4, gpio_get_level(LED4));
        }
        //actualiza valor con el actual
        pulsador1 = pulsador;                       
        vTaskDelay(1000/portTICK_PERIOD_MS); 
    }
}



/*
Programa Realizado por Javier Burgoa
Manejo de entradas por interrupción y salidas digitales.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED1     4
#define LED2 	 5
#define PULS_ISR 0

Volatile int flag, flag2   = 0;

//-------------ISR---------------------
static void IRAM_ATTR ExtPinO_ISR_handler(void *args)
{
     flag2 = !flag2;
     gpio_set_level(LED1, flag2);             
}

//-------------main--------------------- 
void app_main(void)
{        
     gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
     gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
     gpio_set_direction(PULS, GPIO_PULLUP_ENABLE);
        
     gpio_config_t  myGPIOconfig;                //por medio de estructuras

     //configuramos la estructura gpio_config_t
     myGPIOconfig.pin_bit_mask = 1ULL<< PULS_ISR;  //GPIO PULS_ISR asociado a la interrup-ción
     myGPIOconfig.mode         = GPIO_MODE_INPUT;  //input
     myGPIOconfig.pull_up_en   = true;             //pull-up enabled
     myGPIOconfig.pull_down_en = false;            //pull-down disabled
     myGPIOconfig.intr_type    = GPIO_INTR_NEGEDGE;//Falling edge

     gpio_config(&myGPIOconfig);		      //registramos el pin.

     //registramos la rutina de atención a la interrupción:
     gpio_install_isr_service(0);
     gpio_isr_handler_add(PULS_ISR, ExtPinO_ISR_handler, NULL);

     while(1)
     {            
        //GPIO4 parpadea cada 2000ms
         gpio_set_level(LED2,flag);
         flag = !flag;
         vTaskDelay(2000/portTICK_PERIOD_MS);
     }
}
