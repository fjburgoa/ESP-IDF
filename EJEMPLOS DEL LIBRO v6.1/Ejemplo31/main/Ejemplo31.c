 
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"           //Semáforo              
#include "esp_adc/adc_oneshot.h"       //ADC
#include "esp_adc/adc_cali.h"          //ADC
#include "esp_adc/adc_cali_scheme.h"   //ADC 
 
#define STACK_SIZE   2*1024    //n x 1kByte es el tamaño de la pila   
#define ADC1_CHANN   ADC_CHANNEL_5 //<-> GPIO6
#define NUM_MEDIDAS  1000
#define T1           200    //ms
#define T2           200    //ms
 

int  min_over_time = 30000;
int  max_over_time = 0;
unsigned long mean_over_time = 0;

SemaphoreHandle_t xSemaphore;

void configura_adc(void);
adc_oneshot_unit_handle_t adc1_handle;  //handler ADC  

//---------------------------------------------------------------------------------
void vTaskMinOverTime( void * pvParameters )
{
    TickType_t           xLastWakeTime; 
    const TickType_t     xDelayTicks = T1/portTICK_PERIOD_MS;  
    
    xLastWakeTime = xTaskGetTickCount ();          

    int medida_adc = 0; 

    while(1)
    {
       xTaskDelayUntil( &xLastWakeTime, xDelayTicks );   //Bloqueo
       //Tarea despierta

       for (int i=0;i<NUM_MEDIDAS;i++)
       {
            adc_oneshot_read (adc1_handle, ADC1_CHANN, &medida_adc);
            if(min_over_time > medida_adc)
                min_over_time = medida_adc;
       }
       xSemaphoreGive(xSemaphore);
    }   
}

//--------------------------------------------------------------------------------
void vTaskMaxOverTime( void * pvParameters )
{
    int medida_adc = 0; 
    TickType_t           xLastWakeTime; 
    const TickType_t     xDelayTicks = 1/portTICK_PERIOD_MS;  
    
    xLastWakeTime = xTaskGetTickCount (); 

    while(1)
    {
       xTaskDelayUntil( &xLastWakeTime, xDelayTicks );    //Bloqueo
       //Tarea despierta

       if (xSemaphoreTake(xSemaphore, portMAX_DELAY))     //A la espera de semáforo
       {
           for (int i=0;i<NUM_MEDIDAS;i++)
           {
              adc_oneshot_read (adc1_handle, ADC1_CHANN, &medida_adc);
              if(max_over_time < medida_adc)
                 max_over_time = medida_adc;
           }           
       }
    }   
}

//--------------------------------------------------------------------------------
int ucParamToPass = 0;    //dummy
void app_main(void) 
{   
    xSemaphore = xSemaphoreCreateBinary();                   //semáforo

    configura_adc();

    TaskHandle_t xHandle1,xHandle2 = NULL;                   //Handler a la tarea
    
    xTaskCreate(vTaskMinOverTime,"MinTask",STACK_SIZE, &ucParamToPass,2,&xHandle1);   
    xTaskCreate(vTaskMaxOverTime,"MaxTask",STACK_SIZE, &ucParamToPass,1,&xHandle2); 

    while (1)
    {
       vTaskDelay(1000 / portTICK_PERIOD_MS);//Tarea main imprime
          
       printf("\t\t\t\t\t\tMax|Min|Mean Over Time: %d, %d, %lu\n",
                max_over_time, 
                min_over_time, 
                mean_over_time); 
    }
}

//-----------------------------------------------------------
void configura_adc(void)
{
    /************* Config ADC ****************/
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

