
#include <math.h>

#include "esp_log.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "wifi_ap.h"
#include "webserver.h"
#include "websocket.h"
#include "esp_wifi.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "EFIS_I2C.h"
#include "freertos/semphr.h"

#include "BMP280.h"
#include "BNO086.h"
#include "GPS.h"

#include "driver/gpio.h"

#include "config.h"

#if DATALOGGER_ENABLED
#include "DataLogger.h"
#endif

static const char *TAG = "MAIN";

/*************************************************************************/
/*************************************************************************/
/*************************************************************************/
/*************************************************************************/

static void print_rtos_statistics(void)
{
#if defined(CONFIG_FREERTOS_USE_TRACE_FACILITY) &&             \
    CONFIG_FREERTOS_USE_TRACE_FACILITY &&                      \
    defined(CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS) &&        \
    CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS &&                 \
    defined(CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS) && \
    CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS

    const UBaseType_t task_count = uxTaskGetNumberOfTasks();
    const size_t buffer_size = ((size_t)task_count + 1U) * 96U;
    char *stats = pvPortMalloc(buffer_size);

    if (stats == NULL)
    {
        ESP_LOGE(TAG, "Sin memoria para las estadísticas del RTOS");
        return;
    }

    stats[0] = '\0';
    vTaskGetRunTimeStats(stats);

    printf("\n========== USO DE CPU POR TAREA ==========\n");
    printf("Tarea\t\tTiempo\tCPU\n");
    printf("%s", stats);
    printf("===========================================\n\n");

    vPortFree(stats);
#else
    ESP_LOGW(TAG,
             "Estadísticas RTOS deshabilitadas. Active "
             "CONFIG_FREERTOS_USE_TRACE_FACILITY, "
             "CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS y "
             "CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS");
#endif
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
#define BOOT_BUTTON_GPIO GPIO_NUM_0
#define BOOT_POLL_PERIOD_MS 20U
#define BOOT_DEBOUNCE_MS 40U
#define RTOS_STATS_STACK_SIZE 3072U
#define RTOS_STATS_PRIORITY 2U

static void boot_button_task(void *arg)
{
    (void)arg;

    while (true)
    {
        if (gpio_get_level(BOOT_BUTTON_GPIO) == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(BOOT_DEBOUNCE_MS));

            if (gpio_get_level(BOOT_BUTTON_GPIO) == 0)
            {
                print_rtos_statistics();

                /* Una sola impresión por pulsación: esperar a soltar BOOT. */
                while (gpio_get_level(BOOT_BUTTON_GPIO) == 0)
                {
                    vTaskDelay(pdMS_TO_TICKS(BOOT_POLL_PERIOD_MS));
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(BOOT_POLL_PERIOD_MS));
    }
}
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
static esp_err_t boot_button_start(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << BOOT_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_RETURN_ON_ERROR(
        gpio_config(&config),
        TAG,
        "No se pudo configurar el botón BOOT");

    const BaseType_t created = xTaskCreate(
        boot_button_task,
        "rtos_stats",
        RTOS_STATS_STACK_SIZE,
        NULL,
        RTOS_STATS_PRIORITY,
        NULL);

    return (created == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

void app_main(void)
{

    /*
     * Mantener el BNO086 en reset desde el principio del arranque.
     * Cableado actual:
     *   GPIO4 <- H_INTN
     *   GPIO6 -> RESET_N
     */
    gpio_set_direction(BNO086_RESET_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BNO086_RESET_GPIO, 0);

    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND))
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    else
    {
        ESP_ERROR_CHECK(err);
    }
    vTaskDelay(pdMS_TO_TICKS(STARTUP_BNO_TO_BMP_DELAY_MS));
    ESP_ERROR_CHECK(efis_i2c_init());

    /* BNO086 primero: su driver libera RESET_N y espera H_INTN. */
    ESP_ERROR_CHECK(BNO086_start());

    /* El BMP280 empieza a usar el bus después de configurar SH-2. */
    vTaskDelay(pdMS_TO_TICKS(STARTUP_BNO_TO_BMP_DELAY_MS));
    ESP_ERROR_CHECK(bmp280_start());

    vTaskDelay(pdMS_TO_TICKS(STARTUP_GPS_TO_I2C_DELAY_MS));
    ESP_ERROR_CHECK(GPS_start());

    /* DataLogger SPIFFS: activacion centralizada en config.h. */
#if DATALOGGER_ENABLED
    esp_err_t logger_err = DataLogger_start();

    if (logger_err != ESP_OK)
    {
        ESP_LOGE(TAG, "DataLogger no disponible: %s", esp_err_to_name(logger_err));
    }
#endif

    ESP_ERROR_CHECK(wifi_ap_start()); // arranca el Access point

    httpd_handle_t server = webserver_start(); // arranca el servidor web http
    if (server == NULL)
    {
        ESP_LOGE(TAG, "No se ha podido iniciar el servidor web");
        return;
    }

    ESP_ERROR_CHECK(websocket_start_dummy_stream(server)); // Register WebSocket endpoint

    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(WIFI_TX_POWER_DBM));

    int8_t power = 0;

    ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&power));

    ESP_LOGI(TAG, "WiFi TX power = %.1f dBm", power / 4.0f);

    ESP_ERROR_CHECK(boot_button_start());

    ESP_LOGI(TAG, "Sistema iniciado correctamente");
}
