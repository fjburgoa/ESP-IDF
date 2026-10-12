#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#define LED4 4     //GPIO4  
#define PULS 0     //GPIO0 -> BOOT

int pulsador, pulsador1  = 0;

void app_main(void)
{
    //establece Entradas y Salidas 
    gpio_config_t io_conf = {
     .pin_bit_mask = 1ULL << PULS,
     .mode         = GPIO_MODE_INPUT,
     .pull_up_en   = true,             //pull-up habilitada
     .pull_down_en = false,            //pull-down deshabilitada
    };
    gpio_config(&io_conf);
    gpio_set_direction(LED4, GPIO_MODE_INPUT_OUTPUT);

    while(1)
    {       
        pulsador  = gpio_get_level(PULS);      //Lee la entrada
        
        //compara actual y anterior, si ha habido cambio es que hay un flanco 
        if ((pulsador != pulsador1)&&(pulsador == 0))
        {
            gpio_set_level(LED4, !gpio_get_level(LED4));
        }               
        pulsador1 = pulsador;                  // valor anterior  actual

        vTaskDelay(pdMS_TO_TICKS(10));         //Delay 10 ms   
    }
}
