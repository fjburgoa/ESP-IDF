/**
 * @file BNO055.c
 * @brief Orquestación, tarea FreeRTOS y API pública del módulo BNO055.
 *
 * Este fichero no contiene acceso directo a registros ni algoritmos complejos:
 *
 *   BNO055_driver.c
 *       -> hardware, I2C, registros y configuración.
 *
 *   BNO055_processing.c
 *       -> filtros, actitud, giro, bola y magnitudes derivadas.
 *
 *   BNO055.c
 *       -> adquisición de una muestra, tarea, estado compartido y API pública.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "BNO055.h"
#include "BNO055_driver.h"
#include "BNO055_processing.h"
#include "config.h"

/* -------------------------------------------------------------------------- */
/* Tarea                                                                      */
/* -------------------------------------------------------------------------- */

/*
 *  40 ms = 25 Hz.
 * 100 ms = 10 Hz.
 */
#define BNO055_PERIOD_MS 40U
#define BNO055_TASK_STACK_SIZE 5120U
#define BNO055_TASK_PRIORITY 5U

/* -------------------------------------------------------------------------- */

static const char *TAG = "BNO055";

static TaskHandle_t s_bno055_task = NULL;

static portMUX_TYPE s_data_mux = portMUX_INITIALIZER_UNLOCKED;

static bno055_data_t s_data = {0};

static bno055_processing_state_t s_processing_state = {0};

/* -------------------------------------------------------------------------- */
/* Adquisición de una muestra                                                 */
/* -------------------------------------------------------------------------- */

static esp_err_t bno055_read_sample(bno055_data_t *sample, float dt_s)
{
    if (sample == NULL)
        return ESP_ERR_INVALID_ARG;

    bno055_data_t data = {0};

    /* ---------------------------------------------------------------------- */
    /* Datos físicos básicos                                                  */
    /* ---------------------------------------------------------------------- */

    ESP_RETURN_ON_ERROR(bno055_driver_read_acceleration(&data.acceleration_ms2), TAG,
                        "Error leyendo acceleration_ms2");

    ESP_RETURN_ON_ERROR(bno055_driver_read_magnetic_field(&data.magnetic_field_ut), TAG,
                        "Error leyendo magnetometro");

    ESP_RETURN_ON_ERROR(bno055_driver_read_gyro(&data.gyro_dps), TAG,
                        "Error leyendo giroscopo");

    /* ---------------------------------------------------------------------- */
    /* Datos de la fusión interna, sólo en NDOF                               */
    /* ---------------------------------------------------------------------- */

#if BNO055_USE_INTERNAL_FUSION

    ESP_RETURN_ON_ERROR(bno055_driver_read_euler(&data.heading_deg,
                                                 &data.roll_deg,
                                                 &data.pitch_deg),
                        TAG, "Error leyendo Euler");

    ESP_RETURN_ON_ERROR(bno055_driver_read_quaternion(&data.quaternion), TAG,
                        "Error leyendo cuaternion");

    ESP_RETURN_ON_ERROR(bno055_driver_read_linear_acceleration(&data.linear_acceleration_ms2), TAG,
                        "Error leyendo aceleracion lineal");

    ESP_RETURN_ON_ERROR(bno055_driver_read_gravity(&data.gravity_ms2), TAG,
                        "Error leyendo gravedad");

#else

    /*
     * Estos campos no son proporcionados por el BNO055 en AMG.
     * Se mantienen a cero para conservar la interfaz pública.
     */
    data.heading_deg = 0.0f;
    data.quaternion = (bno055_quaternionf_t){0};
    data.linear_acceleration_ms2 = (bno055_vector3f_t){0};
    data.gravity_ms2 = (bno055_vector3f_t){0};

#endif

    /* ---------------------------------------------------------------------- */
    /* Temperatura y calibración                                              */
    /* ---------------------------------------------------------------------- */

    ESP_RETURN_ON_ERROR(bno055_driver_read_temperature(&data.temperature_c), TAG,
                        "Error leyendo temperatura");

    ESP_RETURN_ON_ERROR(bno055_driver_read_calibration(&data.calibration_system,
                                                       &data.calibration_gyro,
                                                       &data.calibration_accel,
                                                       &data.calibration_mag),
                        TAG, "Error leyendo calibracion");

    /* ---------------------------------------------------------------------- */
    /* Procesado matemático                                                   */
    /* ---------------------------------------------------------------------- */

    bno055_process_sample(&s_processing_state, &data, dt_s,
#if BNO055_USE_INTERNAL_FUSION
                          true
#else
                          false
#endif
    );

    data.valid = true;
    *sample = data;

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Tarea                                                                      */
/* -------------------------------------------------------------------------- */

static void BNO055Task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();

    const TickType_t period = pdMS_TO_TICKS(BNO055_PERIOD_MS);

    const float dt_s = (float)BNO055_PERIOD_MS / 1000.0f;

    for (;;)
    {
        bno055_data_t sample = {0};

        const esp_err_t err = bno055_read_sample(&sample, dt_s);

        if (err == ESP_OK)
        {
            portENTER_CRITICAL(&s_data_mux);

            s_data = sample;

            portEXIT_CRITICAL(&s_data_mux);

            ESP_LOGD(
                TAG,
                "H=%.2f R=%.2f P=%.2f | "
                "A=[%.3f %.3f %.3f] G | "
                "G=%.3f [%.3f %.3f] | "
                "CAL=%u/%u/%u/%u",
                (double)sample.heading_deg,
                (double)sample.roll_deg,
                (double)sample.pitch_deg,
                (double)sample.accel_x_g,
                (double)sample.accel_y_g,
                (double)sample.accel_z_g,
                (double)sample.g_current,
                (double)sample.g_min,
                (double)sample.g_max,
                sample.calibration_system,
                sample.calibration_gyro,
                sample.calibration_accel,
                sample.calibration_mag);
        }
        else
        {
            ESP_LOGW(TAG, "Lectura BNO055 fallida: %s", esp_err_to_name(err));
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

/* -------------------------------------------------------------------------- */
/* API pública                                                                */
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

esp_err_t BNO055_set_mount_mode(bno055_mount_mode_t mode)
{
    if ((mode != BNO055_MOUNT_VERTICAL) && (mode != BNO055_MOUNT_HORIZONTAL))
        return ESP_ERR_INVALID_ARG;

    if (mode == bno055_driver_get_mount_mode())
        return ESP_OK;

    /*
     * La tarea periódica no debe acceder al bus I2C mientras el driver entra
     * temporalmente en CONFIGMODE para cambiar AXIS_MAP.
     */
    if (s_bno055_task != NULL)
    {
        vTaskSuspend(s_bno055_task);
    }

    const esp_err_t err = bno055_driver_set_mount_mode(mode);

    if (err == ESP_OK)
    {
        /*
         * El cambio de frame invalida la muestra previa y todos los estados de filtros/estimadores.
         */
        portENTER_CRITICAL(&s_data_mux);

        s_data = (bno055_data_t){0};

        portEXIT_CRITICAL(&s_data_mux);

        bno055_processing_reset(&s_processing_state);

        vTaskDelay(pdMS_TO_TICKS(150U));
    }

    if (s_bno055_task != NULL)
    {
        vTaskResume(s_bno055_task);
    }

    return err;
}

/* -------------------------------------------------------------------------- */

bno055_mount_mode_t BNO055_get_mount_mode(void)
{
    return bno055_driver_get_mount_mode();
}

/* -------------------------------------------------------------------------- */

void BNO055_reset_accel_peaks(void)
{
    if (s_bno055_task != NULL)
        vTaskSuspend(s_bno055_task);

    portENTER_CRITICAL(&s_data_mux);

    /*
     * El reset no modifica la medida instantánea; sólo reinicia los extremos
     * tomando como origen la G actual.
     */
    bno055_processing_reset_g_peaks(&s_processing_state, s_data.g_current);
    s_data.g_min = s_data.g_current;
    s_data.g_max = s_data.g_current;

    portEXIT_CRITICAL(&s_data_mux);

    if (s_bno055_task != NULL)
        vTaskResume(s_bno055_task);

    ESP_LOGI(TAG, "G-meter reseteado: min/max = G actual");
}

/* -------------------------------------------------------------------------- */

esp_err_t BNO055_start(void)
{
    if (s_bno055_task != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(bno055_driver_init(), TAG, "No se pudo inicializar BNO055");

    portENTER_CRITICAL(&s_data_mux);

    s_data = (bno055_data_t){0};

    portEXIT_CRITICAL(&s_data_mux);

    bno055_processing_reset(&s_processing_state);

    const BaseType_t ok = xTaskCreate(BNO055Task, "bno055", BNO055_TASK_STACK_SIZE, NULL,
                                      BNO055_TASK_PRIORITY, &s_bno055_task);

    if (ok != pdPASS)
    {
        s_bno055_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(
        TAG,
        "BNO055 iniciado en %s a %u Hz",
#if BNO055_USE_INTERNAL_FUSION
        "NDOF",
#else
        "AMG",
#endif
        1000U / BNO055_PERIOD_MS);

    return ESP_OK;
}
