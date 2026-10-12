 
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
 
#define GPIO_OUTPUT_IO_0    4     //GPIO 4  - salida 1
#define GPIO_OUTPUT_IO_1    6     //GPIO 6  - salida 2
#define PULSADOR            0     //GPIO 0  - entrada digital

#define STACK_SIZE 2*1024      //1kByte es el tamaño mínimo de la pila   

volatile int state = 0;

/************************************************************************* */
void myTask1( void * pvParameters )
{
     int state_led = 0;
     const TickType_t xDelayTicks = 100/portTICK_PERIOD_MS;  //period. = 100ms.
          
     while(1)
     {
       //contenido de la tarea 
       gpio_set_level(GPIO_OUTPUT_IO_1,state_led); 
       state_led = !state_led;

       //pasamos la tarea a bloqueo
       vTaskDelay(xDelayTicks);
       //Tarea Despierta...
     }  
}
//---------------------------------------------------------
void myTask2( void * pvParameters )
{
     int state_led = 0;
     const TickType_t xDelayTicks = 250/portTICK_PERIOD_MS;  //period. = 250ms 
     
     while(1)
     {
       //ejecución de la tarea
       gpio_set_level(GPIO_OUTPUT_IO_0,state_led); 
       state_led = !state_led;

       //pasamos la tarea a bloqueo     
       vTaskDelay (xDelayTicks);
       //Tarea Despierta...     
     }  
}

//--------------------------------------------------------- 
void app_main(void) 
{   
    /************* Config GPIO ***************/    
    gpio_set_direction(GPIO_OUTPUT_IO_0,GPIO_MODE_OUTPUT)  ;  //GPIO_OUTPUT_IO_0 
    gpio_set_direction(GPIO_OUTPUT_IO_1,GPIO_MODE_OUTPUT)  ;  //GPIO_OUTPUT_IO_1

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << PULSADOR,
        .mode         = GPIO_MODE_INPUT,
     .pull_up_en   = true,             //pull-up habilitada
     .pull_down_en = false,            //pull-down deshabilitada
    };
    gpio_config(&io_conf);

    //Handlers a tareas      
    TaskHandle_t xHandle1 = NULL;    //Handler a la tarea1
    TaskHandle_t xHandle2 = NULL;    //Handler a la tarea2

    int ucParamToPass = 0;           //dummy  
 
    //Crear la tarea  
    xTaskCreate(myTask1, "MyTask_1", STACK_SIZE, &ucParamToPass, 1, &xHandle1);
    xTaskCreate(myTask2, "MyTask_2", STACK_SIZE, &ucParamToPass, 1, &xHandle2);
 
    int my_puls     = 1;
    int my_puls_old = 0;

    while (1) 
    {        
        my_puls = gpio_get_level(PULSADOR);         //coge estado BOOT

        int estado_tarea = eTaskGetState(xHandle2); 
        printf("Estado tarea = %d\n",estado_tarea);

        if ((my_puls!=my_puls_old)&&(my_puls==0))   //solo en los flancos de bajada
        {
           if (estado_tarea==2)
           {
              vTaskSuspend(xHandle2);                //suspende la tarea 
              printf("Tarea 2 suspendida\n");
           }

           if (estado_tarea==3)
           {
              vTaskDelay(300/portTICK_PERIOD_MS);             
              vTaskResume(xHandle2);                 //reanuda la tarea 
              printf("Tarea 2 reanudada\n");
           }
        }
        my_puls_old = my_puls; 
 
        //Tarea bloqueada
        vTaskDelay(100/portTICK_PERIOD_MS); 
        //Tarea despierta...
 
    }
}

