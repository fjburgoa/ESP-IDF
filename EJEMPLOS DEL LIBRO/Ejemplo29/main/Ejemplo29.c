
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/queue.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
 
#define STACK_SIZE  2*1024    //tamaño de la pila: 2kBytes

#define ADC1_CHANN  ADC_CHANNEL_5 //<-> GPIO6
#define N  	    10            //tamaño de la cola

volatile int state1 = 0;
volatile int state2 = 0;
QueueHandle_t HandlerCola = NULL;
adc_oneshot_unit_handle_t adc1_handle;  //handler ADC  

void configura_adc(void);

//------------------------------------------------------------------------------
void vTaskCode1( void * pvParameters )
{
    while(1)
    {
       vTaskDelay(200/portTICK_PERIOD_MS);  

       int medida_adc=0;
       adc_oneshot_read (adc1_handle, ADC1_CHANN , &medida_adc);

       if (!xQueueSend(HandlerCola,&medida_adc,pdMS_TO_TICKS(10)))
          printf("Error en el envío de la cola\n");
    }   
}
//---------------------------------------------------------------------------------
void vTaskCode2( void * pvParameters )
{
    int buff[N] = {0};
    int k = 0;

    while(1)
    {
       vTaskDelay(100/portTICK_PERIOD_MS);  

       int valor_recibido,valor_promediado   = 0;
       
       while (xQueueReceive(HandlerCola, &valor_recibido,0))
       {
            buff[k] = valor_recibido;    //almacena el dato en buffer interno a tarea2
            k++;                         //actualiza el índice del buffer de 
     
            if (k>N-1)                   //forma cíclica
               k= 0;

            valor_promediado = 0;
            for (int j=0; j<N; j++)
                 valor_promediado += buff[j];   //suma todos los datos del buffer

            printf("Se ha recibido: %d. Promedio: %d\n",valor_recibido, 
                                                valor_promediado/N);
       }       
    }   
}

//---------------------------------------------------------------------------------
int ucParToPass = 0;    //dummy
void app_main(void) 
{   
    configura_adc();

    TaskHandle_t xHandle1 = NULL;                        //Handler a la tarea
    TaskHandle_t xHandle2 = NULL;                        //Handler a la tarea
    
    //Crea las tareas
    xTaskCreate(vTaskCode1,"Task1", STACK_SIZE, &ucParToPass, 1, &xHandle1);   
    xTaskCreate(vTaskCode2,"Task2", STACK_SIZE, &ucParToPass, 2, &xHandle2); 
    
    HandlerCola = xQueueCreate ( N, sizeof(uint32_t));   //Crea la cola

    while (1) 
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Tarea main ociosa  
    }
}

//------Rutina de configuración del ADC----------------------------------------
void configura_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_config = 
    {
        .unit_id = ADC_UNIT_1,                                                 
    };
    adc_oneshot_new_unit (&init_config, &adc1_handle);
    adc_oneshot_chan_cfg_t config = 
    {
        .atten    = ADC_ATTEN_DB_12,                                               
        .bitwidth = ADC_BITWIDTH_DEFAULT,                                           
    };
    adc_oneshot_config_channel(adc1_handle, ADC1_CHANN , &config);              
}

