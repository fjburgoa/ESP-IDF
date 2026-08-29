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

static void BNO055Task(void *pvParameters);

/* -------------------------------------------------------------------------- */

static const char *TAG = "BNO055";

static TaskHandle_t s_bno055_task = NULL;

static portMUX_TYPE s_data_mux = portMUX_INITIALIZER_UNLOCKED;

static bno055_data_t s_data = {0};

static bno055_processing_state_t s_processing_state = {0};

static bno055_operation_mode_t s_operation_mode = (bno055_operation_mode_t)BNO055_DEFAULT_OPERATION_MODE;

/*
 * Datos de baja frecuencia.
 *
 * Temperatura y estado de calibracion no necesitan actualizarse a la misma
 * frecuencia que actitud, aceleracion y giróscopo. Se almacenan aqui y se
 * refrescan aproximadamente a 1 Hz.
 */
static int8_t s_temperature_c = 0;
static uint8_t s_calibration_system = 0U;
static uint8_t s_calibration_gyro = 0U;
static uint8_t s_calibration_accel = 0U;
static uint8_t s_calibration_mag = 0U;

/* -------------------------------------------------------------------------- */
/* Adquisición de una muestra                                                 */
/* -------------------------------------------------------------------------- */

static esp_err_t bno055_read_sample(bno055_data_t *sample, bool read_slow_data)
{
    if (sample == NULL)
        return ESP_ERR_INVALID_ARG;

    bno055_data_t data = {0};

    /* ---------------------------------------------------------------------- */
    /* Datos rápidos comunes                                                  */
    /* ---------------------------------------------------------------------- */

    ESP_RETURN_ON_ERROR(
        bno055_driver_read_acceleration(&data.acceleration_ms2),
        TAG,
        "Error leyendo acceleration_ms2");

    ESP_RETURN_ON_ERROR(
        bno055_driver_read_gyro(&data.gyro_dps),
        TAG,
        "Error leyendo giroscopo");

    /*
     * El magnetómetro no se utiliza en el EFIS.
     * Se fuerza explícitamente a cero y no se genera ninguna transacción I2C.
     */
    data.magnetic_field_ut = (bno055_vector3f_t){0};

    /* ---------------------------------------------------------------------- */
    /* Datos rápidos exclusivos de NDOF                                       */
    /* ---------------------------------------------------------------------- */

    if (s_operation_mode == BNO055_OPERATION_MODE_NDOF)
    {
        /*
         * La actitud se obtiene del cuaternión. No se leen los registros Euler,
         * eliminando una transacción I2C adicional por ciclo.
         */
        /*
                ESP_RETURN_ON_ERROR(
                    bno055_driver_read_quaternion(&data.quaternion),
                    TAG,
                    "Error leyendo cuaternion");

                ESP_RETURN_ON_ERROR(
                    bno055_driver_read_linear_acceleration(
                        &data.linear_acceleration_ms2),
                    TAG,
                    "Error leyendo aceleracion lineal");
        */
        ESP_RETURN_ON_ERROR(bno055_driver_read_gravity(&data.gravity_ms2), TAG, "Error leyendo gravedad");
    }
    else
    {
        data.heading_deg = 0.0f;
        data.roll_deg = 0.0f;
        data.pitch_deg = 0.0f;

        data.quaternion = (bno055_quaternionf_t){0};
        data.linear_acceleration_ms2 = (bno055_vector3f_t){0};
        data.gravity_ms2 = (bno055_vector3f_t){0};
    }

    /* ---------------------------------------------------------------------- */
    /* Datos lentos: aproximadamente 1 Hz                                      */
    /* ---------------------------------------------------------------------- */

    if (read_slow_data)
    {
        ESP_RETURN_ON_ERROR(
            bno055_driver_read_temperature(&s_temperature_c),
            TAG,
            "Error leyendo temperatura");

        ESP_RETURN_ON_ERROR(
            bno055_driver_read_calibration(
                &s_calibration_system,
                &s_calibration_gyro,
                &s_calibration_accel,
                &s_calibration_mag),
            TAG,
            "Error leyendo calibracion");
    }

    data.temperature_c = s_temperature_c;
    data.calibration_system = s_calibration_system;
    data.calibration_gyro = s_calibration_gyro;
    data.calibration_accel = s_calibration_accel;
    data.calibration_mag = s_calibration_mag;

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

esp_err_t BNO055_set_operation_mode(bno055_operation_mode_t mode)
{
    if ((mode != BNO055_OPERATION_MODE_AMG) && (mode != BNO055_OPERATION_MODE_NDOF))
        return ESP_ERR_INVALID_ARG;

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
        ESP_LOGI(TAG, "Modo IMU cambiado a %s", mode == BNO055_OPERATION_MODE_NDOF ? "NDOF" : "AMG");

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

/* -------------------------------------------------------------------------- */
/* Tarea                                                                      */
/* -------------------------------------------------------------------------- */

static void BNO055Task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();

    const TickType_t period = pdMS_TO_TICKS(BNO055_PERIOD_MS);
    const float dt_s = (float)BNO055_PERIOD_MS / 1000.0f;

    /*
     * Temperatura y calibracion se leen aproximadamente a 1 Hz.
     * La division se protege para que el divisor nunca sea cero.
     */
    const uint32_t slow_data_divider = (BNO055_PERIOD_MS < 1000U) ? (1000U / BNO055_PERIOD_MS) : 1U;

    uint32_t slow_data_counter = slow_data_divider;

    for (;;)
    {
        bno055_data_t sample = {0};

        /* ------------------------------------------------------------------ */
        /* 1. Adquisición                                                     */
        /* ------------------------------------------------------------------ */

        const bool read_slow_data = (++slow_data_counter >= slow_data_divider);

        if (read_slow_data)
            slow_data_counter = 0U;

        const esp_err_t err = bno055_read_sample(&sample, read_slow_data);

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
