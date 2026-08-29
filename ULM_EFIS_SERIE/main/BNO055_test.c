/**
 * @file BNO055_test.c
 * @brief Prueba mínima del BNO055 usando únicamente BNO055_driver.c.
 *
 * Objetivo:
 *   - Inicializar el BNO055 mediante bno055_driver_init().
 *   - Crear una tarea periódica.
 *   - Leer directamente del driver las variables utilizadas hasta ahora.
 *   - Mostrar los valores por el terminal serie.
 *   - No realizar ningún filtrado ni procesado matemático.
 *
 * Uso:
 *   - Sustituir BNO055.c por este fichero en CMakeLists.txt.
 *   - BNO055_processing.c no es necesario para esta prueba.
 *   - main.c puede seguir llamando a BNO055_start().
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "BNO055.h"
#include "BNO055_driver.h"
#include "config.h"

static const char *TAG = "BNO055_TEST";

static TaskHandle_t s_bno055_test_task = NULL;
static portMUX_TYPE s_data_mux = portMUX_INITIALIZER_UNLOCKED;
static bno055_data_t s_data = {0};

/* -------------------------------------------------------------------------- */

static void bno055_log_vector(
    const char *name,
    const bno055_vector3f_t *v,
    const char *unit)
{
    ESP_LOGI(
        TAG,
        "%-5s X=%+8.3f  Y=%+8.3f  Z=%+8.3f %s",
        name,
        (double)v->x,
        (double)v->y,
        (double)v->z,
        unit);
}

/* -------------------------------------------------------------------------- */

static void bno055_test_boot_init(void)
{
    const gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BNO055_TEST_BOOT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */

bno055_data_t BNO055_get_data(void)
{
    bno055_data_t snapshot;

    portENTER_CRITICAL(&s_data_mux);
    snapshot = s_data;
    portEXIT_CRITICAL(&s_data_mux);

    return snapshot;
}

/* -------------------------------------------------------------------------- */

static void BNO055Task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(BNO055_PERIOD_MS);

    int boot_previous = gpio_get_level(BNO055_TEST_BOOT_GPIO);
    TickType_t last_mode_change = 0;

    for (;;)
    {
        /*
         * BOOT es activo a nivel bajo. En cada flanco 1 -> 0 se avanza:
         *
         *   AMG -> IMUPLUS -> NDOF -> NDOF_FMC_OFF -> AMG ...
         */
        const int boot_now = gpio_get_level(BNO055_TEST_BOOT_GPIO);
        const TickType_t now = xTaskGetTickCount();

        if ((boot_previous == 1) && (boot_now == 0) && ((now - last_mode_change) >= pdMS_TO_TICKS(BNO055_TEST_BOOT_DEBOUNCE_MS)))
        {
            bno055_driver_cycle_test_mode();
            last_mode_change = now;
        }

        boot_previous = boot_now;

        bno055_vector3f_t acceleration_ms2 = {0};
        bno055_vector3f_t gyro_dps = {0};
        bno055_quaternionf_t quaternion = {0};
        bno055_vector3f_t linear_acceleration_ms2 = {0};
        bno055_vector3f_t gravity_ms2 = {0};

        uint8_t calibration_system = 0U;
        uint8_t calibration_gyro = 0U;
        uint8_t calibration_accel = 0U;
        uint8_t calibration_mag = 0U;

        bool sample_ok = true;
        esp_err_t err;

        /* 1. Acelerómetro */
        err = bno055_driver_read_acceleration(&acceleration_ms2);

        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "Error leyendo ACC: %s", esp_err_to_name(err));
            sample_ok = false;
        }

        /* 2. Giróscopo */
        if (sample_ok)
        {
            err = bno055_driver_read_gyro(&gyro_dps);

            if (err != ESP_OK)
            {
                ESP_LOGW(TAG, "Error leyendo GYR: %s", esp_err_to_name(err));
                sample_ok = false;
            }
        }

        /* 3. Cuaternión */
        if (sample_ok)
        {
            err = bno055_driver_read_quaternion(&quaternion);

            if (err != ESP_OK)
            {
                ESP_LOGW(TAG, "Error leyendo QUAT: %s", esp_err_to_name(err));
                sample_ok = false;
            }
        }

        /* 4. Aceleración lineal */
        if (sample_ok)
        {
            err = bno055_driver_read_linear_acceleration(
                &linear_acceleration_ms2);

            if (err != ESP_OK)
            {
                ESP_LOGW(TAG, "Error leyendo LIN: %s", esp_err_to_name(err));
                sample_ok = false;
            }
        }

        /* 5. Vector gravedad */
        if (sample_ok)
        {
            err = bno055_driver_read_gravity(&gravity_ms2);

            if (err != ESP_OK)
            {
                ESP_LOGW(TAG, "Error leyendo GRAV: %s", esp_err_to_name(err));
                sample_ok = false;
            }
        }

        /* 6. Temperatura */
        /*
        if (sample_ok)
        {
            err = bno055_driver_read_temperature(&temperature_c);

            if (err != ESP_OK)
            {
                ESP_LOGW(TAG, "Error leyendo TEMP: %s", esp_err_to_name(err));
                sample_ok = false;
            }
        }
        */

        /* 7. Calibración */
        if (sample_ok)
        {
            err = bno055_driver_read_calibration(
                &calibration_system,
                &calibration_gyro,
                &calibration_accel,
                &calibration_mag);

            if (err != ESP_OK)
            {
                ESP_LOGW(TAG, "Error leyendo CAL: %s", esp_err_to_name(err));
                sample_ok = false;
            }
        }

        /* Publicar la última muestra completa para otros módulos. */
        if (sample_ok)
        {
            portENTER_CRITICAL(&s_data_mux);

            s_data.acceleration_ms2 = acceleration_ms2;
            s_data.gyro_dps = gyro_dps;
            s_data.quaternion = quaternion;
            s_data.linear_acceleration_ms2 = linear_acceleration_ms2;
            s_data.gravity_ms2 = gravity_ms2;
            s_data.calibration_system = calibration_system;
            s_data.calibration_gyro = calibration_gyro;
            s_data.calibration_accel = calibration_accel;
            s_data.calibration_mag = calibration_mag;
            s_data.valid = true;

            portEXIT_CRITICAL(&s_data_mux);
        }

        /* 8. Salida por terminal sólo si la muestra completa es válida */
        /*
                if (sample_ok)
                {
                    // ESP_LOGI(TAG, "------------------------------------------------------------");

                    ESP_LOGI(
                        TAG,
                        "MODE: %s (OPR_MODE=0x%02X)",
                        bno055_driver_get_mode_name(),
                        bno055_driver_get_mode_register());

                    bno055_log_vector("ACC", &acceleration_ms2, "m/s2");
                    bno055_log_vector("GYR", &gyro_dps, "deg/s");


                    ESP_LOGI(
                        TAG,
                        "QUAT  W=%+9.5f  X=%+9.5f  Y=%+9.5f  Z=%+9.5f",
                        (double)quaternion.w,
                        (double)quaternion.x,
                        (double)quaternion.y,
                        (double)quaternion.z);

                    bno055_log_vector("LIN", &linear_acceleration_ms2, "m/s2");
                    bno055_log_vector("GRAV", &gravity_ms2, "m/s2");

                    ESP_LOGI(
                        TAG,
                        "CAL SYS=%u GYR=%u ACC=%u MAG=%u",
                        (unsigned int)calibration_system,
                        (unsigned int)calibration_gyro,
                        (unsigned int)calibration_accel,
                        (unsigned int)calibration_mag);
                }
        */

        vTaskDelayUntil(&last_wake, period);
    }
}

/* -------------------------------------------------------------------------- */
/* API compatible con main.c                                                  */
/* -------------------------------------------------------------------------- */

esp_err_t BNO055_start(void)
{
    if (s_bno055_test_task != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Inicializando BNO055 en modo de prueba");
    bno055_test_boot_init();

    /*
     * Toda la detección y configuración física permanece en BNO055_driver.c.
     */
    ESP_RETURN_ON_ERROR(
        bno055_driver_init(),
        TAG,
        "No se pudo inicializar BNO055");

    ESP_LOGI(
        TAG,
        "Driver inicializado: %s (OPR_MODE=0x%02X), periodo=%u ms (%u Hz)",
        bno055_driver_get_mode_name(),
        bno055_driver_get_mode_register(),
        (unsigned int)BNO055_PERIOD_MS,
        (unsigned int)(1000U / BNO055_PERIOD_MS));

    const BaseType_t ok =
        xTaskCreate(
            BNO055Task,
            "bno055_test",
            BNO055_TASK_STACK_SIZE,
            NULL,
            BNO055_TASK_PRIORITY,
            &s_bno055_test_task);

    if (ok != pdPASS)
    {
        s_bno055_test_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Tarea BNO055_TEST iniciada");

    return ESP_OK;
}
