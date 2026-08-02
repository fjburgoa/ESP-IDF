#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"


#define LED4 4

int state = 0; //variable que almacena el estado del LED en GPIO4

void app_main(void)
{
    //configura GPIO4 como salida digital
    gpio_set_direction(LED4, GPIO_MODE_OUTPUT);  

    while(1)
    {
        //establece nivel
        gpio_set_level(LED4, state);
        state = !state;

        //Delay 1 segundo = 1000ms, (frecuencia = 0,5Hz)
	    vTaskDelay(pdMS_TO_TICKS(500));
    }
}
