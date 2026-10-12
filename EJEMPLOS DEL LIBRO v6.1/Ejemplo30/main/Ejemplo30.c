 
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "freertos/semphr.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
 
#define STACK_SIZE   2*1024    //n x 1kByte es el tamaño de la pila   
#define ADC1_CHANN  ADC_CHANNEL_5 //<-> GPIO6
#define NUM_MEDIDAS  1000

#define T1           200    //ms
#define T2           200    //ms
#define T3           200    //ms

int  min_over_time = 30000;    //El mutex protege tanto el acceso al ADC como a las 
int  max_over_time = 0;        //variables compartidas.
unsigned long mean_over_time = 0;

SemaphoreHandle_t xMutex;

void configura_adc(void);
adc_oneshot_unit_handle_t adc1_handle;  //handler ADC  

//-----Tarea 1---------------------------------------------------------------------
void vTaskMinOverTime( void * pvParameters )
{
    int medida_adc = 0; 

    while(1)
    {
       vTaskDelay (T1/portTICK_PERIOD_MS );          //Tarea bloqueada
       if (xSemaphoreTake(xMutex, portMAX_DELAY))    //¿Recurso libre?
       {                                
            for (int i=0;i<NUM_MEDIDAS;i++)          //Opera     
            {
                adc_oneshot_read (adc1_handle, ADC1_CHANN , &medida_adc);
                if(min_over_time > medida_adc)
                    min_over_time = medida_adc;
                
            }
            //El mutex se mantiene durante NUM_MEDIDAS lecturas-> sección larga  
            xSemaphoreGive(xMutex);                  //Libera el recurso
       }                
       //Continua tarea
    }   
}
//------Tarea 2--------------------------------------------------------------------
void vTaskMaxOverTime( void * pvParameters )
{
    int medida_adc = 0; 

    while(1)
    {
       vTaskDelay (T2/portTICK_PERIOD_MS );          //Tarea bloqueada
       if (xSemaphoreTake(xMutex, portMAX_DELAY))    //¿Recurso libre?
       {  
           for (int i=0;i<NUM_MEDIDAS;i++)           //Opera
           {
              adc_oneshot_read (adc1_handle, ADC1_CHANN  , &medida_adc);
              if(max_over_time < medida_adc)
                 max_over_time = medida_adc;
           }
           //El mutex se mantiene durante NUM_MEDIDAS lecturas-> sección larga
           xSemaphoreGive(xMutex);                    //Libera el recurso 
           //Continua tarea
       }
    }   
}

//-----Tarea 3 --------------------------------------------------------------------
void vTaskMeanOverTime( void * pvParameters )
{
   int medida_adc = 0; 

    while(1)
    {
       mean_over_time = 0;
       vTaskDelay (T3/portTICK_PERIOD_MS );          //Tarea bloqueada
       if (xSemaphoreTake(xMutex, portMAX_DELAY))    //¿Recurso libre?
       {  
           for (int i=0;i<NUM_MEDIDAS;i++)           //Opera 
           {
              adc_oneshot_read (adc1_handle, ADC1_CHANN  , &medida_adc);
              mean_over_time += medida_adc;
           }       
           mean_over_time = mean_over_time/NUM_MEDIDAS;
           //El mutex se mantiene durante NUM_MEDIDAS lecturas-> sección larga
           xSemaphoreGive(xMutex);                    //Libera el recurso
           //Continua tarea
       }                
    }   
}

//-----main_app ------------------------------------------------------------------
int ucParToPass = 0;    //dummy
void app_main(void) 
{   
    xMutex = xSemaphoreCreateMutex();                      //mutex

    configura_adc();

    TaskHandle_t xHandle1,xHandle2,xHandle3 = NULL;        //Handler a las tareas
    
    xTaskCreate(vTaskMinOverTime,"MinTask",STACK_SIZE,  &ucParToPass,2,&xHandle1);
    xTaskCreate(vTaskMaxOverTime,"MaxTask",STACK_SIZE,  &ucParToPass,2,&xHandle2);   
    xTaskCreate(vTaskMeanOverTime,"MeanTask",STACK_SIZE,&ucParToPass,1,&xHandle3);   

    while (1){
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Tarea main ociosa          
        printf("\t\t\t\t\t\tMax|Min|Mean Over Time: %d, %d, %lu\n",max_over_time, 
                                                                   min_over_time, 
                                                                   mean_over_time) ; 
    }
}


//---Configuración del ADC---------------------------------------------------
void configura_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_config = 
    {
        .unit_id = ADC_UNIT_1,                                                 
    };
    adc_oneshot_new_unit (&init_config, &adc1_handle);
    adc_oneshot_chan_cfg_t config = {
        .atten    = ADC_ATTEN_DB_12,                                               
        .bitwidth = ADC_BITWIDTH_DEFAULT,                                           
    };
    adc_oneshot_config_channel(adc1_handle, ADC1_CHANN  , &config);              
}

