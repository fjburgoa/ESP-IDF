#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"

#define BUTTON_GPIO   0        // Pin BOOT

void app_main(void)
{
    // 1.- Configura GPIO del pulsador BOOT como entrada con pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    //2.- la tarea app_main está asociada al TWDT
    esp_task_wdt_add(NULL);   
    
    //3️.- Bucle principal
    while (true) 
    {
        int level = gpio_get_level(BUTTON_GPIO);

        if (level == 0)  // cuando BOOT es pulsado alimenta el TWDT 
 {  
            printf("BOOT pulsado y watchdog alimentado\n");
            esp_task_wdt_reset();
            
            vTaskDelay(pdMS_TO_TICKS(20));  // Pequeña espera para evitar rebotes
        }

        // El código vendría aquí
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
