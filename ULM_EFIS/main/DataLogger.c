/**
 * @file DataLogger.c
 * @brief Registrador circular en PSRAM/RAM a la frecuencia del BNO086.
 *
 * Las muestras se guardan como estructuras binarias de tamaño fijo. Esto evita
 * escribir Flash cada 40 ms y permite sobrescribir de forma determinista la
 * muestra más antigua cuando el buffer se llena. El servidor HTTP convierte el
 * contenido a CSV, en orden cronológico, después de detener la grabación.
 */

#include "DataLogger.h"

#include <inttypes.h>
#include <math.h>
#include <stdlib.h>

#include "BNO086.h"
#include "GPS.h"
#include "config.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "DATALOGGER";

static datalogger_sample_t *s_buffer = NULL;
static TaskHandle_t s_task = NULL;
static SemaphoreHandle_t s_mutex = NULL;

/* Índice de la próxima escritura y número de posiciones válidas. */
static uint32_t s_write_index = 0U;
static uint32_t s_sample_count = 0U;
static uint32_t s_total_samples = 0U;

static bool s_initialized = false;
static bool s_recording = false;
static bool s_wrapped = false;

/* Relación entre el reloj monotónico y UTC fijada al comenzar la grabación. */
static int64_t s_start_monotonic_us = 0;
static int64_t s_start_utc_ms = 0;

/* -------------------------------------------------------------------------- */
static bool datalogger_utc_valid(const gps_data_t *gps)
{
    return (gps != NULL) && gps->fix_valid && (gps->utc_timestamp > 0U);
}

/* -------------------------------------------------------------------------- */
static void datalogger_fill_sample(datalogger_sample_t *destination,
                                   const bno086_data_t *imu,
                                   const gps_data_t *gps,
                                   int64_t monotonic_us)
{
    destination->utc_time_ms =
        s_start_utc_ms + (monotonic_us - s_start_monotonic_us) / 1000;

    /*
     * El GNSS se actualiza más despacio que el BNO086. Por eso varias muestras
     * consecutivas pueden contener las mismas coordenadas. Si se pierde el FIX,
     * NAN permite distinguirlo de una posición real situada en 0°, 0°.
     */
    destination->latitude_deg =
        (gps != NULL) && gps->fix_valid ? gps->latitude_deg : NAN;
    destination->longitude_deg =
        (gps != NULL) && gps->fix_valid ? gps->longitude_deg : NAN;

    destination->acceleration_x_ms2 = imu->acceleration_ms2.x;
    destination->acceleration_y_ms2 = imu->acceleration_ms2.y;
    destination->acceleration_z_ms2 = imu->acceleration_ms2.z;

    destination->linear_acceleration_x_ms2 = imu->linear_acceleration_ms2.x;
    destination->linear_acceleration_y_ms2 = imu->linear_acceleration_ms2.y;
    destination->linear_acceleration_z_ms2 = imu->linear_acceleration_ms2.z;

    destination->gravity_x_ms2 = imu->gravity_ms2.x;
    destination->gravity_y_ms2 = imu->gravity_ms2.y;
    destination->gravity_z_ms2 = imu->gravity_ms2.z;

    destination->gyro_x_dps = imu->gyro_dps.x;
    destination->gyro_y_dps = imu->gyro_dps.y;
    destination->gyro_z_dps = imu->gyro_dps.z;

    destination->pitch_deg = imu->pitch_deg;
    destination->roll_deg = imu->roll_deg;
    destination->slip_ball_deg = imu->slip_ball_deg;
    destination->turn_rate_dps = imu->yaw_rate_dps;
}

/* -------------------------------------------------------------------------- */
static void datalogger_task(void *argument)
{
    (void)argument;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(DATALOGGER_PERIOD_MS);

    for (;;)
    {
        bool recording = false;

        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10U)) == pdTRUE)
        {
            recording = s_recording;
            xSemaphoreGive(s_mutex);
        }

        if (recording)
        {
            const bno086_data_t imu = BNO086_get_data();
            const gps_data_t gps = GPS_get_data();
            const int64_t monotonic_us = esp_timer_get_time();

            if (imu.valid)
            {
                datalogger_sample_t sample;
                datalogger_fill_sample(&sample, &imu, &gps, monotonic_us);

                if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10U)) == pdTRUE)
                {
                    if (s_recording)
                    {
                        s_buffer[s_write_index] = sample;
                        s_write_index =
                            (s_write_index + 1U) % DATALOGGER_MAX_SAMPLES;

                        if (s_sample_count < DATALOGGER_MAX_SAMPLES)
                        {
                            ++s_sample_count;
                        }
                        else
                        {
                            s_wrapped = true;
                        }

                        ++s_total_samples;
                    }

                    xSemaphoreGive(s_mutex);
                }
            }
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

/* -------------------------------------------------------------------------- */
esp_err_t DataLogger_start(void)
{
    if (s_task != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    s_mutex = xSemaphoreCreateMutex();

    if (s_mutex == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    const size_t memory_bytes =
        (size_t)DATALOGGER_MAX_SAMPLES * sizeof(datalogger_sample_t);

    const size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    const size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    if (psram_total == 0U)
    {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        ESP_LOGE(TAG,
                 "PSRAM no disponible; active CONFIG_SPIRAM en menuconfig");
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* El buffer se fuerza a PSRAM para no agotar la SRAM interna del sistema. */
    s_buffer = heap_caps_calloc(
        DATALOGGER_MAX_SAMPLES,
        sizeof(datalogger_sample_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (s_buffer == NULL)
    {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        ESP_LOGE(TAG,
                 "PSRAM insuficiente: solicitados=%u, libres=%u, total=%u bytes",
                 (unsigned)memory_bytes,
                 (unsigned)psram_free,
                 (unsigned)psram_total);
        return ESP_ERR_NO_MEM;
    }

    s_write_index = 0U;
    s_sample_count = 0U;
    s_total_samples = 0U;
    s_recording = false;
    s_wrapped = false;
    s_initialized = true;

    const BaseType_t created = xTaskCreate(
        datalogger_task,
        "data_logger",
        DATALOGGER_TASK_STACK_SIZE,
        NULL,
        DATALOGGER_TASK_PRIORITY,
        &s_task);

    if (created != pdPASS)
    {
        s_initialized = false;
        heap_caps_free(s_buffer);
        s_buffer = NULL;
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG,
             "Buffer circular preparado: %u muestras, %u bytes, periodo=%u ms",
             (unsigned)DATALOGGER_MAX_SAMPLES,
             (unsigned)memory_bytes,
             (unsigned)DATALOGGER_PERIOD_MS);

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
esp_err_t DataLogger_begin_recording(void)
{
    if (!s_initialized || (s_mutex == NULL) || (s_buffer == NULL))
    {
        return ESP_ERR_INVALID_STATE;
    }

    const bno086_data_t imu = BNO086_get_data();
    const gps_data_t gps = GPS_get_data();

    if (!imu.valid)
    {
        ESP_LOGW(TAG, "No se puede grabar: BNO086 todavía no válido");
        return ESP_ERR_INVALID_STATE;
    }

    if (!datalogger_utc_valid(&gps))
    {
        ESP_LOGW(TAG, "No se puede grabar: FIX y fecha/hora GPS no válidos");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100U)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    if (s_recording)
    {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    s_write_index = 0U;
    s_sample_count = 0U;
    s_total_samples = 0U;
    s_wrapped = false;

    s_start_monotonic_us = esp_timer_get_time();
    s_start_utc_ms = (int64_t)gps.utc_timestamp * 1000;
    s_recording = true;

    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Grabación circular iniciada a %u Hz",
             (unsigned)(1000U / DATALOGGER_PERIOD_MS));
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
esp_err_t DataLogger_stop_recording(void)
{
    if (!s_initialized || (s_mutex == NULL))
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100U)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    s_recording = false;
    const uint32_t samples = s_sample_count;
    const uint32_t total_samples = s_total_samples;
    const bool wrapped = s_wrapped;

    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG,
             "Grabación detenida: conservadas=%u, totales=%u, circular=%s",
             (unsigned)samples,
             (unsigned)total_samples,
             wrapped ? "sí" : "no");
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
datalogger_status_t DataLogger_get_status(void)
{
    datalogger_status_t status = {0};

    if (s_mutex == NULL)
    {
        return status;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20U)) == pdTRUE)
    {
        status.initialized = s_initialized;
        status.recording = s_recording;
        status.data_available = s_sample_count > 0U;
        status.wrapped = s_wrapped;
        status.samples = s_sample_count;
        status.capacity = DATALOGGER_MAX_SAMPLES;
        status.total_samples = s_total_samples;
        status.memory_bytes =
            (size_t)DATALOGGER_MAX_SAMPLES * sizeof(datalogger_sample_t);

        xSemaphoreGive(s_mutex);
    }

    return status;
}

/* -------------------------------------------------------------------------- */
esp_err_t DataLogger_get_sample(uint32_t chronological_index,
                                datalogger_sample_t *sample)
{
    if ((sample == NULL) || !s_initialized || (s_mutex == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100U)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    if (s_recording || (chronological_index >= s_sample_count))
    {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    const uint32_t oldest_index =
        (s_sample_count == DATALOGGER_MAX_SAMPLES) ? s_write_index : 0U;
    const uint32_t physical_index =
        (oldest_index + chronological_index) % DATALOGGER_MAX_SAMPLES;

    *sample = s_buffer[physical_index];

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}
