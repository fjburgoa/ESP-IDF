#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

#define LED4     4
#define PULS_ISR 0

//-------------ISR---------------------
static void IRAM_ATTR ExtPinO_ISR_handler(void *args)
{
gpio_set_level(LED4, !gpio_get_level(LED4)); //conmuta estado del LED              
}

//------------------------------------- 
void app_main(void)
{      
    //GPIO4 como salida digital (GPIO_MODE_OUTPUT o GPIO_MODE_INPUT_OUTPUT)
    gpio_set_direction(LED4, GPIO_MODE_INPUT_OUTPUT);  

    gpio_config_t  myGPIOconfig;                    //estructura de configuración input 

    //Se configura la estructura gpio_config_t
    myGPIOconfig.pin_bit_mask = 1ULL<< PULS_ISR;    //GPIO PULS_ISR asociado a la 
                                                    //interrupción
    myGPIOconfig.mode         = GPIO_MODE_INPUT;    //GPIO es entrada
    myGPIOconfig.pull_up_en   = true;               //pull-up habilitada
    myGPIOconfig.pull_down_en = false;              //pull-down deshabilitada
    myGPIOconfig.intr_type    = GPIO_INTR_NEGEDGE;  //Flanco de bajada

    gpio_config(&myGPIOconfig);		                //registra el pin y su configuración

    gpio_install_isr_service(0);		            //registra la rutina la ISR	

    gpio_isr_handler_add(PULS_ISR, ExtPinO_ISR_handler, NULL);

    while(1)
    {            
       //no hay que hacer nada en el bucle principal.
       vTaskDelay(pdMS_TO_TICKS(1));
    }
}
