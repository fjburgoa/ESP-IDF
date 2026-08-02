/*
5 tareas se ejecutan de forma concurrente.

Se registra el acceso a cada una de ellas

Al pulsar BOOT, las tareas se suspenden y se muestra por el terminal serie 
La secuencia de eventos.

*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_clk_tree.h"
#include "trace_recorder_jtag.h"

//PERIODOS TAREAS en ms

#define TASK1_T 50
#define TASK2_T 50
#define TASK3_T 50
#define TASK4_T 50
#define TASK5_T 50

//CONTADORES
#define ITERATE_1 60000
#define ITERATE_2 60000
#define ITERATE_3 60000
#define ITERATE_4 60000
#define ITERATE_5 60000


//GPIO SALIDAS Y ENTRADAS DIGITALES
#define PULSADOR 0

#define STACK_SIZE	4*1024	     //N x 1kByte es el tamaño de la piLa  

bool flag_stats = false;
void imprime_estadisticas(void);


//----------------------------------------------------------------
//--------------Task1--------------------------------------------- 
//----------------------------------------------------------------
void vTaskCode1( void * pvParameters )           
{
    TickType_t 		    xLastWakeTime; 
    const TickType_t 	xDelayTicks = TASK1_T/portTICK_PERIOD_MS;
      
    xLastWakeTime = xTaskGetTickCount ();         // Initialise the xLastWakeTime variable with the current time. 
    while(1)
    {
       if (!flag_stats) 
            printf("\tI\t\t\t\t\t%llu\n", esp_timer_get_time());            

       //Consume CPU cycles
       for (long i = 0; i < ITERATE_1; i++) {
           __asm__ __volatile__("NOP");
       }

       if (!flag_stats) 
            printf("\tO\t\t\t\t\t%llu\n", esp_timer_get_time());            

       //bloquea y espera hasta TASK1_T ms        
       xTaskDelayUntil( &xLastWakeTime, xDelayTicks );        
    }	
}
//----------------------------------------------------------------
//-----------------Task2 -----------------------------------------
//----------------------------------------------------------------
void vTaskCode2( void * pvParameters )           //
{
    TickType_t 		    xLastWakeTime; 
    const TickType_t 	xDelayTicks = TASK2_T/portTICK_PERIOD_MS;

    xLastWakeTime = xTaskGetTickCount ();         // Initialise the xLastWakeTime variable with the current time. 
    while(1)
    {
       if (!flag_stats)       
            printf("\t\tI\t\t\t\t%llu\n", esp_timer_get_time());  
 
       //Consume CPU cycles
       for (long i = 0; i < ITERATE_2; i++) {
           __asm__ __volatile__("NOP");
       }
       if (!flag_stats) 
            printf("\t\tO\t\t\t\t%llu\n", esp_timer_get_time());
 
       //bloquea y espera hasta TASK2_T ms               
       xTaskDelayUntil( &xLastWakeTime, xDelayTicks );         
    }	      
}
//----------------------------------------------------------------
//-----------------Task3: ----------------------------------------
//----------------------------------------------------------------
void vTaskCode3( void * pvParameters )           
{
    TickType_t 		    xLastWakeTime; 
    const TickType_t 	xDelayTicks = TASK3_T/portTICK_PERIOD_MS;
      
    xLastWakeTime = xTaskGetTickCount ();         // Initialise the xLastWakeTime variable with the current time. 

    while(1)
    {
       if (!flag_stats) 
            printf("\t\t\tI\t\t\t%llu\n", esp_timer_get_time());       

       //Consume CPU cycles
       for (long i = 0; i < ITERATE_3; i++) {
           __asm__ __volatile__("NOP");
       }
       if (!flag_stats) 
            printf("\t\t\tO\t\t\t%llu\n", esp_timer_get_time()); 

       //bloquea y espera hasta TASK3_T ms        
       xTaskDelayUntil( &xLastWakeTime, xDelayTicks );             
    }	
}

//----------------------------------------------------------------
//-----------------Task4:   --------------------------------------
//----------------------------------------------------------------
void vTaskCode4( void * pvParameters )             
{
     TickType_t 		xLastWakeTime; 
    const TickType_t 	xDelayTicks = TASK4_T/portTICK_PERIOD_MS;

    xLastWakeTime = xTaskGetTickCount ();         // Initialise the xLastWakeTime variable with the current time. 
    while(1)
    {
       if (!flag_stats)       
            printf("\t\t\t\tI\t\t%llu\n", esp_timer_get_time());

       //Consume CPU cycles
       for (long i = 0; i < ITERATE_4; i++) {
           __asm__ __volatile__("NOP");
       }
       if (!flag_stats) 
            printf("\t\t\t\tO\t\t%llu\n", esp_timer_get_time());

       //bloquea y espera hasta TASK4_T ms               
       xTaskDelayUntil( &xLastWakeTime, xDelayTicks );             
    }
}
//----------------------------------------------------------------
//-----------------Task5:   --- ----------------------------------
//----------------------------------------------------------------
void vTaskCode5( void * pvParameters )             
{
     TickType_t 		xLastWakeTime; 
    const TickType_t 	xDelayTicks = TASK5_T/portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount ();         // Initialise the xLastWakeTime variable with the current time. 
    while(1)
    {
       if (!flag_stats) 
            printf("\t\t\t\t\tI\t%llu\n", esp_timer_get_time());                         

       //Consume CPU cycles
       for (long i = 0; i < ITERATE_5; i++) {
           __asm__ __volatile__("NOP");
       }

       if (!flag_stats) 
            printf("\t\t\t\t\tO\t%llu\n", esp_timer_get_time());  
 
       //bloquea y espera hasta TASK5_T ms
       xTaskDelayUntil( &xLastWakeTime, xDelayTicks );             
    }  
}


//----------------------------------------------------------------
//-----------------Main loop    ----------------------------------
//----------------------------------------------------------------

#define GPIO_OUTPUT_IO_0    4     //GPIO 4  - salida 1 - 3Hz
#define GPIO_OUTPUT_IO_1    5     //GPIO 5  - salida 2 - 6Hz
#define PULSADOR            0     //GPIO 0  - entrada digital

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

    //Crea Handlers a las taraeas
    TaskHandle_t xHandle1, xHandle2, xHandle3, xHandle4, xHandle5 = NULL;  //Handler a las tareas 1..5
    TaskHandle_t hTrace = NULL;

    //Crea tareas
    xTaskCreatePinnedToCore( vTaskCode1, "TASK1", STACK_SIZE, NULL, 1, &xHandle1,0 );  //Prioridad 1
    xTaskCreatePinnedToCore( vTaskCode2, "TASK2", STACK_SIZE, NULL, 1, &xHandle2,0 );  //Prioridad 1 
    xTaskCreatePinnedToCore( vTaskCode3, "TASK3", STACK_SIZE, NULL, 1, &xHandle3,0 );  //Prioridad 1   
    xTaskCreatePinnedToCore( vTaskCode4, "TASK4", STACK_SIZE, NULL, 1, &xHandle4,0 );  //Prioridad 1   
    xTaskCreatePinnedToCore( vTaskCode5, "TASK5", STACK_SIZE, NULL, 1, &xHandle5,0 );  //Prioridad 1   
 
    trace_init();

    trace_register_task(xHandle1, "TASK1", 1);
    trace_register_task(xHandle2, "TASK2", 2);
    trace_register_task(xHandle3, "TASK3", 3);
    trace_register_task(xHandle4, "TASK4", 4);
    trace_register_task(xHandle5, "TASK5", 5);

     xTaskCreatePinnedToCore(trace_flush_task, "TraceFlush", 4096, NULL, 1, &hTrace,1);
    

    while (1) 
    {
       vTaskDelay(pdMS_TO_TICKS(500)); // debounce + evitar doble trigger
    }

}



void imprime_estadisticas(void)
{
    char Buff[512]      = {0};
    char task_list[512] = {0};
    uint32_t cpu_freq_hz = 0;
    
    //------------- Statistics ----------------------

    //Consulta el stado de todas las tareas
    vTaskList(task_list);
    vTaskGetRunTimeStats(Buff);    
    esp_clk_tree_src_get_freq_hz(SOC_CPU_CLK_SRC_PLL, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &cpu_freq_hz);

    //imprime por el terminal.
    printf("**************************************\n");
    printf("Estado de las tareas:\n%s\n", task_list);    

    //Imprime la carga de la CPU.
    printf("%s ", Buff);   

    //CPU Freq                
    printf("La CPU se ha configurado a : %lu MHz\n", cpu_freq_hz / 1000000);

    //ticks rate
    printf("TIME SLICE: %d Hz, %.1f ms\n", configTICK_RATE_HZ, (float)(1000.0/configTICK_RATE_HZ));    

    printf("**************************************\n"); 
}
