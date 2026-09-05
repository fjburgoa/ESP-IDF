
#include <math.h>

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
#include "EFIS_I2C.h"
#include "freertos/semphr.h"

#include "BMP280.h"
#include "BNO055.h"
#include "BNO055_driver.h"
#include "GPS.h"
#include "MPU6050.h"
#include "Hybrid_pitch_roll.h"

#include "config.h"

#if DATALOGGER_ENABLED
#include "DataLogger.h"
#endif

SemaphoreHandle_t xMutex;

/*************************************************************************/
/*************************************************************************/
/*************************************************************************/
/*************************************************************************/
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

    xMutex = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(efis_i2c_init());

#ifndef BNOTESTMODE
    vTaskDelay(pdMS_TO_TICKS(STARTUP_GPS_TO_I2C_DELAY_MS));
    ESP_ERROR_CHECK(GPS_start());
#endif
    vTaskDelay(pdMS_TO_TICKS(STARTUP_I2C_TO_BNO_DELAY_MS));
    ESP_ERROR_CHECK(MPU6050_start());

    vTaskDelay(pdMS_TO_TICKS(STARTUP_I2C_TO_BNO_DELAY_MS));
    ESP_ERROR_CHECK(BNO055_start());

#ifndef BNOTESTMODE
    vTaskDelay(pdMS_TO_TICKS(STARTUP_BNO_TO_BMP_DELAY_MS));
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
        ALTITUDE_TEST_TASK_STACK_SIZE,
        NULL,
        ALTITUDE_TEST_TASK_PRIORITY,
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

    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(WIFI_TX_POWER_QDBM));

    int8_t power = 0;

    ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&power));

    ESP_LOGI(TAG, "WiFi TX power = %.1f dBm", power / 4.0f);

    ESP_LOGI(TAG, "Sistema iniciado correctamente");
#endif

    while (1)
    {
        bno055_data_t bno = BNO055_get_data();
        mpu6050_data_t mpu = MPU6050_get_data();

        float mod_a_mpu = sqrtf(mpu.accel_x_g * mpu.accel_x_g + mpu.accel_y_g * mpu.accel_y_g + mpu.accel_z_g * mpu.accel_z_g);
        // float mod_a_grv = sqrtf(bno.gravity_ms2.x * bno.gravity_ms2.x + bno.gravity_ms2.y * bno.gravity_ms2.y + bno.gravity_ms2.z * bno.gravity_ms2.z);

        if (bno.valid && mpu.valid)
        {

            hybrid_pitch_roll_data_t attitude = Hybrid_pitch_roll_update(
                mpu.accel_x_g,
                mpu.accel_y_g,
                mpu.accel_z_g,
                bno.gravity_ms2.x,
                bno.gravity_ms2.y,
                bno.gravity_ms2.z);

            printf(
                "P=%+7.3f R=%+7.3f "
                "|a|=%6.3f e=%5.3f "
                "gamma=%5.1f "
                "am=%4.2f ag=%4.2f alpha=%4.2f "
                "|gBNO|=%6.3f MODE=%s\n",

                (double)attitude.pitch_deg,
                (double)attitude.roll_deg,

                (double)attitude.accel_norm_ms2,
                (double)attitude.accel_error_ms2,

                (double)attitude.angle_error_deg,

                (double)attitude.accel_mod_weight,
                (double)attitude.accel_angle_weight,
                (double)attitude.accel_weight,

                (double)attitude.gravity_norm_ms2,

                bno055_driver_get_mode_name());
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
