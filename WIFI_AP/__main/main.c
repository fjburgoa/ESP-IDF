

#include "esp_log.h"
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
#include "driver/i2c.h"

#include "BMP280.h"
#include "BNO055.h"
#include "GPS.h"
#include "config.h"

#if DATALOGGER_ENABLED
#include "DataLogger.h"
#endif

#define I2C_MASTER_SCL_IO GPIO_NUM_9
#define I2C_MASTER_SDA_IO GPIO_NUM_8
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

#define VSI_TEST_MODE 0

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static const char *TAG = "MAIN";

/*
 * Función pública implementada en BMP280.c.
 * Conviene añadir también su prototipo a BMP280.h.
 */
extern void bmp280_set_test_altitude(float altitude_m, bool enable);

/* -------------------------------------------------------------------------- */
/* Prueba del altímetro / variómetro                                           */
/* -------------------------------------------------------------------------- */

#define ALTITUDE_TEST_PERIOD_MS 80U
#define ALTITUDE_TEST_RATE_FPM 500.0f
#define ALTITUDE_TEST_MIN_M 700.0f
#define ALTITUDE_TEST_MAX_M 760.0f

#define FEET_TO_METERS 0.3048f
#define SECONDS_PER_MINUTE 60.0f

static void altitude_test_task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(ALTITUDE_TEST_PERIOD_MS);

    const float dt_s =
        (float)ALTITUDE_TEST_PERIOD_MS / 1000.0f;

    /* 500 ft/min = 2.54 m/s. */
    const float rate_mps = ALTITUDE_TEST_RATE_FPM * FEET_TO_METERS / SECONDS_PER_MINUTE;

    float fake_altitude_m = ALTITUDE_TEST_MIN_M;
    float direction = 1.0f;
    uint32_t log_divider = 0U;

    for (;;)
    {
        fake_altitude_m += direction * rate_mps * dt_s;

        if (fake_altitude_m >= ALTITUDE_TEST_MAX_M)
        {
            fake_altitude_m = ALTITUDE_TEST_MAX_M;
            direction = -1.0f;
        }
        else if (fake_altitude_m <= ALTITUDE_TEST_MIN_M)
        {
            fake_altitude_m = ALTITUDE_TEST_MIN_M;
            direction = 1.0f;
        }

        /*
         * Solo se inyecta altitud. BMP280.c calcula la velocidad vertical a
         * partir de esta señal exactamente igual que lo hará con la altitud
         * barométrica real.
         */
        bmp280_set_test_altitude(fake_altitude_m, true);

        if (++log_divider >= (1000U / ALTITUDE_TEST_PERIOD_MS))
        {
            log_divider = 0U;

            ESP_LOGI(
                TAG,
                "ALT TEST: %.1f m | consigna VSI=%+.0f ft/min",
                (double)fake_altitude_m,
                (double)(direction * ALTITUDE_TEST_RATE_FPM));
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

void app_main(void)
{

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

    /*
     * GPS por UART1. Se arranca después de los sensores I2C para mantener
     * el arranque secuencial que ha resultado estable.
     */

    ESP_ERROR_CHECK(GPS_start());
    vTaskDelay(pdMS_TO_TICKS(1500U));

    ESP_ERROR_CHECK(i2c_master_init());

    vTaskDelay(pdMS_TO_TICKS(500U));
    ESP_ERROR_CHECK(BNO055_start());

    vTaskDelay(pdMS_TO_TICKS(500U));
    ESP_ERROR_CHECK(bmp280_start());

    /* DataLogger SPIFFS: activacion centralizada en config.h. */
#if DATALOGGER_ENABLED
    esp_err_t logger_err = DataLogger_start();

    if (logger_err != ESP_OK)
    {
        ESP_LOGE(TAG, "DataLogger no disponible: %s", esp_err_to_name(logger_err));
    }
#endif

#if VSI_TEST_MODE
    /*
     * Generador de altitud triangular para comprobar el variómetro.
     * Se puede eliminar o comentar cuando se quiera volver a utilizar
     * exclusivamente la altitud barométrica real.
     */
    BaseType_t test_task_ok = xTaskCreate(
        altitude_test_task,
        "alt_test",
        3072,
        NULL,
        4,
        NULL);

    if (test_task_ok != pdPASS)
    {
        ESP_LOGE(TAG, "No se pudo crear la tarea de prueba de altitud");
        return;
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

    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(32));

    int8_t power = 0;

    ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&power));

    ESP_LOGI(TAG, "WiFi TX power = %.1f dBm", power / 4.0f);

    ESP_LOGI(TAG, "Sistema iniciado correctamente");
}
