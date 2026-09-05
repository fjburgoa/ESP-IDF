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

/* -------------------------------------------------------------------------- */

static const char *TAG = "BNO055";

static TaskHandle_t s_bno055_task = NULL;

static portMUX_TYPE s_data_mux = portMUX_INITIALIZER_UNLOCKED;

static bno055_data_t s_data = {0};

static bno055_processing_state_t s_processing_state = {0};

static bno055_operation_mode_t s_operation_mode = (bno055_operation_mode_t)BNO055_DEFAULT_OPERATION_MODE;

/* -------------------------------------------------------------------------- */
/* Adquisición de una muestra                                                 */
/* -------------------------------------------------------------------------- */

static esp_err_t bno055_read_sample(bno055_data_t *sample)
{
    if (sample == NULL)
        return ESP_ERR_INVALID_ARG;

    bno055_data_t data = {0};

    /* ---------------------------------------------------------------------- */
    /* Datos físicos básicos                                                  */
    /* ---------------------------------------------------------------------- */

    ESP_RETURN_ON_ERROR(bno055_driver_read_acceleration(&data.acceleration_ms2), TAG,
                        "Error leyendo acceleration_ms2");

    // ESP_RETURN_ON_ERROR(bno055_driver_read_magnetic_field(&data.magnetic_field_ut), TAG,
    //                     "Error leyendo magnetometro");

    ESP_RETURN_ON_ERROR(bno055_driver_read_gyro(&data.gyro_dps), TAG,
                        "Error leyendo giroscopo");

    /* ---------------------------------------------------------------------- */
    /* Datos de la fusión interna, sólo en NDOF                               */
    /* ---------------------------------------------------------------------- */

    const bool use_internal_fusion = (s_operation_mode == BNO055_OPERATION_MODE_NDOF);

    if (use_internal_fusion)
    {

        // ESP_RETURN_ON_ERROR(
        //     bno055_driver_read_euler(
        //         &data.heading_deg,
        //         &data.roll_deg,
        //         &data.pitch_deg),
        //     TAG,
        //     "Error leyendo Euler");

        // ESP_RETURN_ON_ERROR(
        //     bno055_driver_read_quaternion(&data.quaternion),
        //     TAG,
        //     "Error leyendo cuaternion");

        ESP_RETURN_ON_ERROR(
            bno055_driver_read_linear_acceleration(
                &data.linear_acceleration_ms2),
            TAG,
            "Error leyendo aceleracion lineal");

        ESP_RETURN_ON_ERROR(
            bno055_driver_read_gravity(&data.gravity_ms2),
            TAG,
            "Error leyendo gravedad");
    }
    else
    {
        data.heading_deg = 0.0f;
        data.quaternion = (bno055_quaternionf_t){0};
        data.linear_acceleration_ms2 = (bno055_vector3f_t){0};
        data.gravity_ms2 = (bno055_vector3f_t){0};
    }

    /* ---------------------------------------------------------------------- */
    /* Temperatura y calibración                                              */
    /* ---------------------------------------------------------------------- */

    ESP_RETURN_ON_ERROR(bno055_driver_read_temperature(&data.temperature_c),
                        TAG,
                        "Error leyendo temperatura");

    ESP_RETURN_ON_ERROR(bno055_driver_read_calibration(&data.calibration_system,
                                                       &data.calibration_gyro,
                                                       &data.calibration_accel,
                                                       &data.calibration_mag),
                        TAG,
                        "Error leyendo calibracion");

    *sample = data;

    return ESP_OK;
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

        vTaskDelay(pdMS_TO_TICKS(BNO055_MOUNT_CHANGE_SETTLE_MS));
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

esp_err_t BNO055_set_operation_mode(
    bno055_operation_mode_t mode)
{
    if ((mode != BNO055_OPERATION_MODE_AMG) &&
        (mode != BNO055_OPERATION_MODE_NDOF))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (mode == s_operation_mode)
        return ESP_OK;

    if (s_bno055_task != NULL)
        vTaskSuspend(s_bno055_task);

    const esp_err_t err = bno055_driver_set_operation_mode(mode);

    if (err == ESP_OK)
    {
        s_operation_mode = mode;

        portENTER_CRITICAL(&s_data_mux);
        s_data = (bno055_data_t){0};
        portEXIT_CRITICAL(&s_data_mux);

        bno055_processing_reset(&s_processing_state);
        vTaskDelay(pdMS_TO_TICKS(BNO055_MOUNT_CHANGE_SETTLE_MS));
    }

    if (s_bno055_task != NULL)
        vTaskResume(s_bno055_task);

    if (err == ESP_OK)
    {
        ESP_LOGI(
            TAG,
            "Modo IMU cambiado a %s",
            mode == BNO055_OPERATION_MODE_NDOF ? "NDOF" : "AMG");
    }

    return err;
}

/* -------------------------------------------------------------------------- */

bno055_operation_mode_t BNO055_get_operation_mode(void)
{
    return s_operation_mode;
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
/* -------------------------------------------------------------------------- */
/* ------------------------BNO055Task---------------------------------------- */
/* -------------------------------------------------------------------------- */
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

        /* ------------------------------------------------------------------ */
        /* 1. Adquisición                                                     */
        /* ------------------------------------------------------------------ */

        const esp_err_t err = bno055_read_sample(&sample);

        if (err == ESP_OK)
        {
            /* -------------------------------------------------------------- */
            /* 2. Procesado según el modo de operación                        */
            /* -------------------------------------------------------------- */

            switch (s_operation_mode)
            {
            case BNO055_OPERATION_MODE_NDOF:
                bno055_process_sample_ndof(&s_processing_state, &sample, dt_s);
                break;

            case BNO055_OPERATION_MODE_AMG:
                bno055_process_sample_amg(&s_processing_state, &sample, dt_s);
                break;

            default:
                ESP_LOGW(TAG, "Modo BNO055 no válido: %d", (int)s_operation_mode);
                break;
            }

            /* -------------------------------------------------------------- */
            /* 3. Publicación                                                 */
            /* -------------------------------------------------------------- */

            sample.valid = true;

            portENTER_CRITICAL(&s_data_mux);
            s_data = sample;
            portEXIT_CRITICAL(&s_data_mux);
        }
        else
        {
            ESP_LOGW(TAG, "Lectura BNO055 fallida: %s", esp_err_to_name(err));
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* ------------------------BNO055_start ------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

esp_err_t BNO055_start(void)
{
    if (s_bno055_task != NULL)
        return ESP_ERR_INVALID_STATE;

    ESP_RETURN_ON_ERROR(bno055_driver_init(), TAG, "No se pudo inicializar BNO055");

    s_operation_mode = bno055_driver_get_operation_mode();

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
        s_operation_mode == BNO055_OPERATION_MODE_NDOF ? "NDOF" : "AMG",
        1000U / BNO055_PERIOD_MS);

    return ESP_OK;
}
