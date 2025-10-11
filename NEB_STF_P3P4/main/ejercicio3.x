#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "driver/uart.h"

//INDICE TAREAS
#define TASK1 0
#define TASK2 1
#define TASK3 2
#define TASK4 3
#define TASK5 4
#define NTASKS TASK4+1

//PERIODOS TAREAS en ms
#define TASK1_T 10
#define TASK2_T 10
#define TASK3_T 10
#define TASK4_T 10
#define TASK5_T 10

//CONTADORES
#define ITER1 50000
#define ITER2 30000
#define ITER3 20000
#define ITER4 10000
#define ITER5  5000




//GPIO SALIDAS Y ENTRADAS DIGITALES
#define LED4 4
#define LED6 6


#define STACK_SIZE	2*1024	     //N x 1kByte es el tamaño de la piLa  

#define BUF_SIZE 512
char Buff1[BUF_SIZE] = {0};
char Buff2[BUF_SIZE] = {0};

unsigned long exec_time[NTASKS] = {0};

//-----TIMER ESP------------------------
//esp_timer_handle_t myTimerESP;   //handler a timer

//----------------------------------------------------------------
//--------------Task1: 
//----------------------------------------------------------------
void vTaskCode1( void * pvParameters )           
{
    TickType_t 		    xLastWakeTime; 
    const TickType_t 	xDelayTicks = TASK1_T/portTICK_PERIOD_MS;
      
    unsigned long us1 = 0;
    unsigned long us2 = 0;
    TickType_t  us3 = 0;
    TickType_t  us4 = 0;

    xLastWakeTime = xTaskGetTickCount ();         // Initialise the xLastWakeTime variable with the current time. 
    while(1)
    {
       //toma tiempo
       us3 = us4;
       us4 = xTaskGetTickCount ();

       //Consume CPU cycles
       for (int i = 0; i < ITER1; i++) {
           __asm__ __volatile__("NOP");
       }

       //calcula dT
       //us2          = xTaskGetTickCount ();;
       exec_time[TASK1] = (us4-us3)*portTICK_PERIOD_MS;
       //printf("T1: %lu\n",exec_time[TASK1]); 
       
       //printf("%d", xPortGetCoreID());        //para saber en qué core está

       // printf("%lu\n",ulTaskGetIdleRunTimePercent());  //Tarea IDLE


       //Suspende y espera hasta 1ms  
       xTaskDelayUntil( &xLastWakeTime, xDelayTicks );     
        
    }	
}
//----------------------------------------------------------------
//-----------------Task2: FFT, inicia con semáforo  --------------
//----------------------------------------------------------------
void vTaskCode2( void * pvParameters )           //
{
    TickType_t 		    xLastWakeTime; 
    const TickType_t 	xDelayTicks = TASK2_T/portTICK_PERIOD_MS;
      
    unsigned long us1,us2 = 0;

    xLastWakeTime = xTaskGetTickCount ();         // Initialise the xLastWakeTime variable with the current time. 
    while(1)
    {
       //toma tiempo
       us1 = esp_timer_get_time();  

        //Consume CPU cycles
        for (int i = 0; i < ITER2; i++) {
            __asm__ __volatile__("NOP");
        }
       

       //calcula dT
       us2          = esp_timer_get_time();
       exec_time[TASK2] = us2-us1;

       //printf("T2: %lu ",exec_time[TASK2]);  
       //printf("%d", xPortGetCoreID());

       //Suspende y espera hasta 1ms  
       xTaskDelayUntil( &xLastWakeTime, xDelayTicks );     
    }	
}
//----------------------------------------------------------------
//-----------------Task3:                      -------------------
//----------------------------------------------------------------
void vTaskCode3( void * pvParameters )           
{
    TickType_t 		    xLastWakeTime; 
    const TickType_t 	xDelayTicks = TASK3_T/portTICK_PERIOD_MS;
      
    unsigned long us1,us2 = 0;

    xLastWakeTime = xTaskGetTickCount ();         // Initialise the xLastWakeTime variable with the current time. 
    while(1)
    {
       //toma tiempo
       us1 = esp_timer_get_time();  

       //Consume CPU cycles
       for (int i = 0; i < ITER3; i++) {
           __asm__ __volatile__("NOP");
       }

       //calcula dT
       us2          = esp_timer_get_time();
       exec_time[TASK3] = us2-us1;

       //printf("T3: %lu",exec_time[TASK3]);  
       //printf("%d", xPortGetCoreID());

       //Suspende y espera hasta 1ms  
       xTaskDelayUntil( &xLastWakeTime, xDelayTicks );     
    }	
}

//----------------------------------------------------------------
//-----------------Task4:   --------------------------------------
//----------------------------------------------------------------
void vTaskCode4( void * pvParameters )             
{
     TickType_t 		    xLastWakeTime; 
    const TickType_t 	xDelayTicks = TASK4_T/portTICK_PERIOD_MS;
      
    unsigned long us1,us2 = 0;

    xLastWakeTime = xTaskGetTickCount ();         // Initialise the xLastWakeTime variable with the current time. 
    while(1)
    {
       //toma tiempo
       us1 = esp_timer_get_time();  
      
       //Consume CPU cycles
       for (int i = 0; i < ITER4; i++) {
           __asm__ __volatile__("NOP");
       }

       //calcula dT
       us2          = esp_timer_get_time();
       exec_time[TASK4] = us2-us1;


       //printf("T4: %lu",exec_time[TASK4]);  
       //printf("%d", xPortGetCoreID());

       //Suspende y espera hasta 1ms  
       xTaskDelayUntil( &xLastWakeTime, xDelayTicks );     
    }	
}
//----------------------------------------------------------------
//-----------------Task5:   --- ----------------------------------
//----------------------------------------------------------------
void vTaskCode5( void * pvParameters )             
{
     TickType_t 		    xLastWakeTime; 
    const TickType_t 	xDelayTicks = TASK4_T/portTICK_PERIOD_MS;
      
    unsigned long us1,us2 = 0;

    xLastWakeTime = xTaskGetTickCount ();         // Initialise the xLastWakeTime variable with the current time. 
    while(1)
    {
       //toma tiempo
       us1 = esp_timer_get_time();  
      
       //Consume CPU cycles
       for (int i = 0; i < ITER5; i++) {
           __asm__ __volatile__("NOP");
       }

       //printf("%d", xPortGetCoreID());

       //calcula dT
       us2          = esp_timer_get_time();
       exec_time[TASK5] = us2-us1;



       //Suspende y espera hasta 1ms  
       xTaskDelayUntil( &xLastWakeTime, xDelayTicks );     
    }	
}


//----------------------------------------------------------------
//-----------------Timer ESP  ------------------------------------
//----------------------------------------------------------------
void ESP_TimerCallback (void)
{
    //nothing to do 
}

//----------------------------------------------------------------
//-----------------Task3: Main  ----------------------------------
//----------------------------------------------------------------

//----INT-------------------------------
gpio_config_t myGPIOconfig;
void ExtPinO_ISR_handler(void *args);

#define PULS_ISR 0


void app_main(void) 
{   
    //configura el Timer    
    /*
    const esp_timer_create_args_t MyTimerESPConfig = 
    {
        .callback = &ESP_TimerCallback,                //callback            
        .name = "My_Periodic"                          //name    
    };

    esp_timer_create(&MyTimerESPConfig, &myTimerESP);
    esp_timer_start_periodic(myTimerESP, 1000000);   //1 s

    */

    gpio_set_direction(LED4,GPIO_MODE_OUTPUT);
    gpio_set_direction(LED6,GPIO_MODE_OUTPUT);

    //Interrupciones IO EXT

    //configuramos la estructura gpio_config_t
    myGPIOconfig.pin_bit_mask = 1ULL<< PULS_ISR;  //entrada asociada a la interrupción
    myGPIOconfig.mode = GPIO_MODE_INPUT;          //input
    myGPIOconfig.pull_up_en = true;               //pull-up enabled
    myGPIOconfig.pull_down_en = false;            //pull-down disabled
    myGPIOconfig.intr_type  = GPIO_INTR_NEGEDGE;  //Falling edge
   
    //registramos el pin.
    gpio_config(&myGPIOconfig);
    //registramos el pin.
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PULS_ISR, ExtPinO_ISR_handler, NULL);


    //Crea Handlers
    TaskHandle_t xHandle1 = NULL;  //Handler a la tarea
    TaskHandle_t xHandle2 = NULL;  //Handler a la tarea
    TaskHandle_t xHandle3 = NULL;  //Handler a la tarea
    TaskHandle_t xHandle4 = NULL;  //Handler a la tarea
    TaskHandle_t xHandle5 = NULL;  //Handler a la tarea

    int ucParameterToPass = 0;     //dummy 

    xTaskCreate( vTaskCode1, "TASK1", STACK_SIZE, &ucParameterToPass, 1, &xHandle1 );  //Prioridad máxima
    xTaskCreate( vTaskCode2, "TASK2", STACK_SIZE, &ucParameterToPass, 1, &xHandle2 );  //Prioridad baja 
    xTaskCreate( vTaskCode3, "TASK3", STACK_SIZE, &ucParameterToPass, 1, &xHandle3 );  //Prioridad media   
    xTaskCreate( vTaskCode4, "TASK4", STACK_SIZE, &ucParameterToPass, 1, &xHandle4 );  //Sin prioridad 
    xTaskCreate( vTaskCode5, "TASK5", STACK_SIZE, &ucParameterToPass, 1, &xHandle5 );  //Sin prioridad 

    while (1) 
    {
       

       
       vTaskGetRunTimeStats(Buff1);
       printf("%s", Buff1); 
       
       //vTaskDelay(2000/portTICK_PERIOD_MS);

       //vTaskList(Buff2);
       /*
       printf("**********************************\n");
       printf("Task  State   Prio    Stack    Num\n"); 
       printf("**********************************\n");
       printf("%s \n %s",Buff2, Buff1);
       printf("**********************************\n");
       memset(Buff1,BUF_SIZE,sizeof(char));
       memset(Buff2,BUF_SIZE,sizeof(char));
        */
              
       

       vTaskDelay(4000/portTICK_PERIOD_MS);
    }
}

//-------------ISR---------------------
void ExtPinO_ISR_handler(void *args)
{
 static long my_puls = 0;
 
 my_puls = !my_puls;
 gpio_set_level(LED6,my_puls);          

 //Consume CPU cycles
 for (int i = 0; i < 10000; i++) 
 {
     __asm__ __volatile__("NOP");
 }

}



 /*
    'B' - Blocked
    'R' - Ready
    'D' - Deleted (waiting clean up)
    'S' - Suspended, or Blocked without a timeout

*/