#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#define BUTTON_GPIO   0        // Pin BOOT

static const char *TAG = "TWDT_BUTTON";

void app_main(void)
{
    // 1.- Configurar GPIO del botón BOOT como entrada con pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    //2.- la tarea de main está asociada al TWDT
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));   

    ESP_LOGI(TAG, "Task Watchdog iniciado. Pulsa el botón BOOT para resetearlo.");

    // 3️.- Bucle principal
    while (true) {
        int level = gpio_get_level(BUTTON_GPIO);

        if (level == 0) {  // pulsado (BOOT tiene pull-up → LOW al pulsar)
            ESP_LOGI(TAG, "BOTÓN pulsado → watchdog alimentado ✅");
            esp_task_wdt_reset();
            
            vTaskDelay(pdMS_TO_TICKS(200));  // Pequeña espera para evitar rebotes
        }

        // Muestra contador de segundos desde último reset
        static int seconds = 0;
        ESP_LOGI(TAG, "Décimas de milisegundo sin pulsar: %d", seconds++);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}