
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"


//PERIODOS TAREAS en ms
#define TASK1_T 200
#define TASK2_T 200

//GPIO SALIDAS Y ENTRADAS DIGITALES
#define PULSADOR 0

#define STACK_SIZE  4*1024       //N x 1kByte es el tamaño de la piLa  

bool flag_stats = false;

static QueueHandle_t gpio_evt_queue = NULL;

//----------------------------------------------------------------
static void ExtPin0_ISR_handler(void *args);
void configura_GPIO_e_INT(void);
static void TareaAsociadaInt (void* arg);

//--------------Task1--------------------------------------------- 
void Tarea1( void * pvParameters )           
{
    while(1)
    {
       //Trabajo dentro de la tarea

       //Suspende y espera hasta TASK1_T ms        
       vTaskDelay (TASK1_T / portTICK_PERIOD_MS);        
    }   
}
//-----------------Task2 -----------------------------------------
void Tarea2( void * pvParameters )           
{
    while(1)
    {
       //Trabajo dentro de la tarea

       //Suspende y espera hasta TASK2_T ms        
       vTaskDelay (TASK2_T / portTICK_PERIOD_MS);        
    }   
}
//-----------------App Main----------------------------------
gpio_config_t myGPIOconfig;

void app_main(void) 
{   
    //Crea una cola para comunicar la acción de la ISR
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    //Configura BOOT para generar una interrupción digital
    configura_GPIO_e_INT();

    //Crea handler de las tareas
    TaskHandle_t xHandle1 = NULL;  //Handler a la tarea 1
    TaskHandle_t xHandle2 = NULL;  //Handler a la tarea 2

    xTaskCreate( Tarea1,"TASK1", STACK_SIZE, NULL, 1, &xHandle1);     //Prioridad 1
    xTaskCreate( Tarea2,"TASK2", STACK_SIZE, NULL, 1, &xHandle2);     //Prioridad 1 

    //Crea handler de las tarea asociada a la ISR
    TaskHandle_t xGPIOint = NULL;  //Handler a la Tarea de la ISR 
    xTaskCreate(TareaAsociadaInt,"ISR",STACK_SIZE,NULL,10,&xGPIOint); //Prioridad 10 

    while (1) 
    {
	//tarea main_app ociosa	
        vTaskDelay(1000/portTICK_PERIOD_MS);   
    }
}
//------------Configura el GPIO para generar una interrupción-----------
void configura_GPIO_e_INT(void)
{
    //se configura la estructura gpio_config_t
    myGPIOconfig.pin_bit_mask = 1ULL<< PULSADOR;    //entrada 
    myGPIOconfig.mode         = GPIO_MODE_INPUT;    //input
    myGPIOconfig.pull_up_en   = true;               //pull-up enabled
    myGPIOconfig.pull_down_en = false;              //pull-down disabled
    myGPIOconfig.intr_type    = GPIO_INTR_NEGEDGE;  //Falling edge
    
    //registra el pin.
    gpio_config(&myGPIOconfig);
    
    //registra el pin y la ISR asociada a la interrupción:
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PULSADOR, ExtPin0_ISR_handler, (void *)PULSADOR);
}

//-------------ISR----------------------------------------------------
static void IRAM_ATTR ExtPin0_ISR_handler(void *args)
{
    uint32_t gpio_num = (uint32_t) args;       

    /* La ISR pone en la cola un aviso indicando que se ha producido una *
    * interrupción en GPIO0. No se realiza procesamiento en la ISR, ya   *
    * que la gestión completa se delega a una tarea (deferred interrupt) */
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);        
}

//------------Tarea asociada a la ISR (Deferred Interrupt)-------------
static void TareaAsociadaInt (void* arg)
{
    uint32_t io_num;
    while(1)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) 
        {
            //Gestiona la tarea
            printf("Tarea asociada a la ISR en ejecución\n");
        }
    }
}

