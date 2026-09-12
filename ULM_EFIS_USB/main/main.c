#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "BNO055.h"
#include "BNO086.h"
#include "EFIS_I2C.h"
#include "MPU6050.h"
#include "config.h"
#include "settings.h"
#include "usb_hid.h"

static const char *TAG = "EFIS_USB";

void app_main(void)
{
    ESP_ERROR_CHECK(settings_init());

    ESP_ERROR_CHECK(efis_i2c_init());

#if defined(PlacaLarga)

    ESP_LOGI(TAG, "PlacaLarga: BNO055 + BNO086");

    /*
     * Primero inicializamos completamente el BNO055.
     */
    // ESP_ERROR_CHECK(BNO055_start());

    // vTaskDelay(pdMS_TO_TICKS(200));

    /*
     * Después configuramos SH-2 y arrancamos la tarea del BNO086.
     */
    ESP_ERROR_CHECK(BNO086_start());

#elif defined(PlacaCorta)

    ESP_LOGI(TAG, "PlacaCorta: BNO055 + MPU6050");

    ESP_ERROR_CHECK(MPU6050_start());

    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_ERROR_CHECK(BNO055_start());

#endif

    /*
     * Se conserva la secuencia original: primero el segundo sensor
     * y después el BNO055, compartiendo el mismo bus I2C.
     */
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_ERROR_CHECK(usb_hid_start());

    ESP_LOGI(TAG, "EFIS USB iniciado");
}
