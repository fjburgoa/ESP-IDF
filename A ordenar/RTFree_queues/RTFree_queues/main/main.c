#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" 
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"

void vTask1(void *pvParameters);
void vTask2(void *pvParameters);
void vTask3(void *pvParameters);

 

static const char *TAG = "EX";
 

QueueHandle_t HandlerColaGlobal=0;



/*-------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------*/
#define STACK_SIZE 1024*2

/*-------------------------------------------------------------------------------*/
esp_err_t print_esp_details(void)
{
    
    printf("Cooomeeeenceeemos...\n");
    
    esp_chip_info_t chip_info;  /* Print chip information */
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU cores.", CONFIG_IDF_TARGET,  chip_info.cores);

    uint32_t flash_size=0;
    esp_flash_get_size(NULL, &flash_size); 
    
    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024) 
                                     , (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
    printf("\n\n\n\n");

    return ESP_OK;
};


void app_main(void)
{

     print_esp_details();
     HandlerColaGlobal = xQueueCreate ( 40, sizeof(uint32_t));
     
     static uint8_t ucParameterToPass;
     
     TaskHandle_t hTask1;
     TaskHandle_t hTask2;
     TaskHandle_t hTask3;
      
     
 

     //xTaskCreate( vTask1, "MyTarea1", STACK_SIZE, &ucParameterToPass, 1, &hTask1 );
     //xTaskCreate( vTask2, "MyTarea2", STACK_SIZE, &ucParameterToPass, 2, &hTask2 );
     //xTaskCreate( vTask3, "MyTarea3", STACK_SIZE, &ucParameterToPass, 3, &hTask3 );

     xTaskCreatePinnedToCore( vTask1, "MyTarea1", STACK_SIZE, &ucParameterToPass, 1, &hTask1, 0 );
     xTaskCreatePinnedToCore( vTask2, "MyTarea2", STACK_SIZE, &ucParameterToPass, 1, &hTask2, 0 );
     xTaskCreatePinnedToCore( vTask3, "MyTarea3", STACK_SIZE, &ucParameterToPass, 1, &hTask3, 1 );  //tskNO_AFFINITY


}

/*-------------------------------------------------------------------------------*/
esp_err_t init_led(int led)
{
    gpio_reset_pin(led);
    gpio_set_direction(led,GPIO_MODE_DEF_OUTPUT);
    return ESP_OK;    
}


/*-------------------------------------------------------------------------------*/
void vTask1(void *pvParameters)
{
    uint8_t led_status=0;
    while(1)
    {
       int coreID = xPortGetCoreID();
        
       for(int i=0;i<8;i++)
       {         
         ESP_LOGI(TAG, "Enviado: %d",i); 
         if (!xQueueSend(HandlerColaGlobal,&i,pdMS_TO_TICKS(10)))
               ESP_LOGE(TAG, "Task1, error enviando a cola");           
       }

       vTaskDelay(pdMS_TO_TICKS(4000));
    }
}
/*-------------------------------------------------------------------------------*/
void vTask2(void *pvParameters)
{


    uint8_t led_status=0; 
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));

        int valor_recibido = 0;
        //if (xQueueReceive(HandlerColaGlobal, &valor_recibido,pdMS_TO_TICKS(10)))
        while (xQueueReceive(HandlerColaGlobal, &valor_recibido,0))
        {
            ESP_LOGI(TAG, "Recibido: %d",valor_recibido); 
        }
        //else
        //{
        //    ESP_LOGE(TAG, "Recibido timeout:"); 
        //}

    }
}
/*-------------------------------------------------------------------------------*/
void vTask3(void *pvParameters)
{
    while(1)
    {
          int coreID = xPortGetCoreID();

          //ESP_LOGE(TAG, "Task3 in Core %d", coreID); 
          vTaskDelay(pdMS_TO_TICKS(1000));


    }
}
 