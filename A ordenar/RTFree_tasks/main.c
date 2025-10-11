#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" 
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

//leds externos
#define led1 4
#define led2 6


static const char *TAG = "EX";

esp_err_t init_led(int led);
esp_err_t blink_led(int led, uint8_t* led_status);

 



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
 
     
     static uint8_t ucParameterToPass;
     
     TaskHandle_t hTask1;
     TaskHandle_t hTask2;
     TaskHandle_t hTask3;
      
     

     init_led(led1);
     init_led(led2);

     

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
        
       blink_led(led1, &led_status);
       ESP_LOGW(TAG, "Task1 in Core %d, Led1 state: %d",coreID, led_status);  
       vTaskDelay(pdMS_TO_TICKS(100));
       
    }
}
/*-------------------------------------------------------------------------------*/
void vTask2(void *pvParameters)
{


    uint8_t led_status=0; 
    while(1)
    {
        int coreID = xPortGetCoreID();

        blink_led(led2, &led_status);
        ESP_LOGE(TAG, "Task2 in Core %d, Led2 state: %d",coreID, led_status);          
        vTaskDelay(pdMS_TO_TICKS(500));

    }
}
/*-------------------------------------------------------------------------------*/
void vTask3(void *pvParameters)
{
    while(1)
    {
          int coreID = xPortGetCoreID();
          ESP_LOGE(TAG, "Task3 in Core %d", coreID); 
          vTaskDelay(pdMS_TO_TICKS(1000));


    }
}
/*-------------------------------------------------------------------------------*/
esp_err_t blink_led(int led, uint8_t* led_status)
{    
    if(*led_status==0)
        *led_status = 1;
    else 
        *led_status = 0;

    gpio_set_level(led,*led_status);
    return ESP_OK;
}
