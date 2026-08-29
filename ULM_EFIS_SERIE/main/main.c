

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
#include "BNO055_driver.h"
#include "GPS.h"
#include "MPU6050.h"
#include "config.h"

#if DATALOGGER_ENABLED
#include "DataLogger.h"
#endif

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EFIS_I2C_SDA_GPIO,
        .scl_io_num = EFIS_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = EFIS_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(EFIS_I2C_PORT, &conf));
    return i2c_driver_install(EFIS_I2C_PORT, conf.mode, 0, 0, 0);
}

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

    ESP_ERROR_CHECK(i2c_master_init());

#ifndef BNOTESTMODE
    vTaskDelay(pdMS_TO_TICKS(STARTUP_GPS_TO_I2C_DELAY_MS));
    ESP_ERROR_CHECK(GPS_start());
#endif
    vTaskDelay(pdMS_TO_TICKS(STARTUP_I2C_TO_BNO_DELAY_MS));
    ESP_ERROR_CHECK(BNO055_start());

    vTaskDelay(pdMS_TO_TICKS(STARTUP_I2C_TO_BNO_DELAY_MS));
    ESP_ERROR_CHECK(MPU6050_start());

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

        if (bno.valid && mpu.valid)
        {
            printf(
                "Ampu=%+7.3f %+7.3f %+7.3f #"
                "Abno=%+7.3f %+7.3f %+7.3f #"
                "LIN=%+7.3f %+7.3f %+7.3f "
                "GRV=%+7.3f %+7.3f %+7.3f m/s2 | "
                "Gmpu=%+7.3f %+7.3f %+7.3f #"
                "Gbno=%+7.3f %+7.3f %+7.3f deg/s | "
                "MODE=%s\n",

                /* Aceleración total MPU6050 */
                (double)mpu.accel_x_g,
                (double)mpu.accel_y_g,
                (double)mpu.accel_z_g,

                /* Aceleración total BNO055 */
                (double)bno.acceleration_ms2.x,
                (double)bno.acceleration_ms2.y,
                (double)bno.acceleration_ms2.z,

                /* Aceleración lineal BNO055 */
                (double)bno.linear_acceleration_ms2.x,
                (double)bno.linear_acceleration_ms2.y,
                (double)bno.linear_acceleration_ms2.z,

                /* Vector gravedad BNO055 */
                (double)bno.gravity_ms2.x,
                (double)bno.gravity_ms2.y,
                (double)bno.gravity_ms2.z,

                /* Giróscopo MPU6050 */
                (double)mpu.gyro_x_dps,
                (double)mpu.gyro_y_dps,
                (double)mpu.gyro_z_dps,

                /* Giróscopo BNO055 */
                (double)bno.gyro_dps.x,
                (double)bno.gyro_dps.y,
                (double)bno.gyro_dps.z,

                /* Modo BNO055 */
                bno055_driver_get_mode_name());
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
