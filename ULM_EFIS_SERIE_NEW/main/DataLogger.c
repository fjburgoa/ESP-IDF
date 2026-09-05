/**
 * @file DataLogger.c
 * @brief Data logger SPIFFS para el experimento de aceleración vertical.
 *
 * Formato CSV:
 *
 *   utc,accel_z_ms2
 *   2026-08-12T18:23:01Z,9.80742
 *
 * Se registra una muestra por segundo. Después de cada fprintf() se ejecuta
 * fflush(), de modo que ante una pérdida de alimentación las muestras previas
 * ya transferidas al sistema de archivos permanecen en Flash.
 */

#include "DataLogger.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "BNO055.h"
#include "BMP280.h"
#include "GPS.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_spiffs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* -------------------------------------------------------------------------- */

#define DATALOGGER_PARTITION_LABEL "FicheroAcelerac"
#define DATALOGGER_BASE_PATH "/spiffs"
#define DATALOGGER_FILE_PATH "/spiffs/aceleracion.csv"

#define DATALOGGER_PERIOD_MS 1000U
#define DATALOGGER_TASK_STACK_SIZE 4096U
#define DATALOGGER_TASK_PRIORITY 4U

/* -------------------------------------------------------------------------- */

static const char *TAG = "DATALOGGER";

static FILE *s_file = NULL;
static TaskHandle_t s_task = NULL;
static SemaphoreHandle_t s_mutex = NULL;

static bool s_mounted = false;
static bool s_recording = false;
static uint32_t s_samples = 0U;

/* -------------------------------------------------------------------------- */
static esp_err_t datalogger_mount_spiffs(void)
{
    /*
     * La partición FicheroAcelerac se ha creado expresamente para este
     * registrador. En el primer arranque puede estar completamente virgen
     * y todavía no contener un sistema de archivos SPIFFS válido.
     *
     * format_if_mount_failed=true permite formatearla automáticamente en
     * ese primer montaje.
     *
     * IMPORTANTE:
     * Una vez montada correctamente, los siguientes arranques no formatean
     * la partición: simplemente montan el SPIFFS existente.
     */
    esp_vfs_spiffs_conf_t conf = {
        .base_path = DATALOGGER_BASE_PATH,
        .partition_label = DATALOGGER_PARTITION_LABEL,
        .max_files = 4,
        .format_if_mount_failed = true,
    };

    ESP_LOGI(
        TAG,
        "Montando SPIFFS '%s' en %s",
        DATALOGGER_PARTITION_LABEL,
        DATALOGGER_BASE_PATH);

    const esp_err_t err =
        esp_vfs_spiffs_register(&conf);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "No se pudo montar/formatear SPIFFS '%s': %s",
            DATALOGGER_PARTITION_LABEL,
            esp_err_to_name(err));

        s_mounted = false;
        return err;
    }

    s_mounted = true;

    ESP_LOGI(
        TAG,
        "SPIFFS '%s' montado correctamente",
        DATALOGGER_PARTITION_LABEL);

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
static size_t datalogger_file_size(void)
{
    struct stat st;

    if (stat(DATALOGGER_FILE_PATH, &st) != 0)
    {
        return 0U;
    }

    return (size_t)st.st_size;
}

/* -------------------------------------------------------------------------- */
static bool datalogger_utc_valid(const gps_data_t *gps)
{
    if (gps == NULL)
    {
        return false;
    }

    return gps->fix_valid &&
           (gps->utc_timestamp > 0U);
}

/* -------------------------------------------------------------------------- */
static void datalogger_task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();

    const TickType_t period = pdMS_TO_TICKS(DATALOGGER_PERIOD_MS);

    for (;;)
    {
        bool recording = false;

        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100U)) == pdTRUE)
        {
            recording = s_recording;
            xSemaphoreGive(s_mutex);
        }

        if (recording)
        {
            const bno055_data_t imu = BNO055_get_data();

            const gps_data_t gps = GPS_get_data();

            if (!imu.valid)
                ESP_LOGW(TAG, "Muestra omitida: BNO055 todavía no válido");
            else if (!datalogger_utc_valid(&gps))
                ESP_LOGW(TAG, "Muestra omitida: fecha/hora GPS no válida");
            else if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(250U)) == pdTRUE)
            {
                if (s_recording && (s_file != NULL))
                {
                    const time_t utc_time = (time_t)gps.utc_timestamp;
                    struct tm utc_tm = {0};

                    gmtime_r(&utc_time, &utc_tm);

                    const int written = fprintf(
                        s_file,
                        "%04d-%02d-%02dT%02d:%02d:%02dZ,%.5f,%.2f;\n",
                        utc_tm.tm_year + 1900,
                        utc_tm.tm_mon + 1,
                        utc_tm.tm_mday,
                        utc_tm.tm_hour,
                        utc_tm.tm_min,
                        utc_tm.tm_sec,
                        (double)imu.acceleration_ms2.z,
                        (double)pressure_hpa);

                    if (written > 0)
                    {
                        /*
                         * Decisión de diseño: vaciar stdio después de CADA
                         * muestra para minimizar pérdidas ante power-off.
                         */
                        if (fflush(s_file) == 0)
                            ++s_samples;
                        else
                            ESP_LOGE(TAG, "Error haciendo fflush() del registro");
                    }
                    else
                        ESP_LOGE(TAG, "Error escribiendo una muestra CSV");
                }

                xSemaphoreGive(s_mutex);
            }
        }

        vTaskDelayUntil(
            &last_wake,
            period);
    }
}

/* -------------------------------------------------------------------------- */
esp_err_t DataLogger_start(void)
{
    if (s_task != NULL)
        return ESP_ERR_INVALID_STATE;

    s_mutex = xSemaphoreCreateMutex();

    if (s_mutex == NULL)
        return ESP_ERR_NO_MEM;

    esp_err_t err = datalogger_mount_spiffs();

    if (err != ESP_OK)
    {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return err;
    }

    size_t total_bytes = 0U;
    size_t used_bytes = 0U;

    if (esp_spiffs_info(
            DATALOGGER_PARTITION_LABEL,
            &total_bytes,
            &used_bytes) == ESP_OK)
    {
        ESP_LOGI(
            TAG,
            "SPIFFS montado: total=%u bytes, usados=%u bytes",
            (unsigned)total_bytes,
            (unsigned)used_bytes);
    }
    else
    {
        ESP_LOGW(
            TAG,
            "SPIFFS montado, pero no se pudo consultar su capacidad");
    }

    /*
     * Nunca se reanuda automáticamente una grabación después de reset.
     * El fichero previo permanece disponible para descarga.
     */
    s_recording = false;
    s_file = NULL;
    s_samples = 0U;

    BaseType_t ok = xTaskCreate(datalogger_task, "data_logger", DATALOGGER_TASK_STACK_SIZE, NULL, DATALOGGER_TASK_PRIORITY, &s_task);

    if (ok != pdPASS)
    {
        s_task = NULL;
        esp_vfs_spiffs_unregister(DATALOGGER_PARTITION_LABEL);
        s_mounted = false;
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "DataLogger preparado; fichero: %s", DATALOGGER_FILE_PATH);

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
esp_err_t DataLogger_begin_recording(void)
{
    if (!s_mounted)
    {
        ESP_LOGE(TAG, "No se puede grabar: SPIFFS no está montado");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_mutex == NULL)
    {
        ESP_LOGE(TAG, "No se puede grabar: mutex no inicializado");
        return ESP_ERR_INVALID_STATE;
    }

    const bno055_data_t imu = BNO055_get_data();
    const gps_data_t gps = GPS_get_data();

    ESP_LOGI(TAG,
             "Estado previo a grabación: "
             "IMU=%d FIX=%d GPSvalid=%d "
             "UTC timestamp=%" PRIu32,
             imu.valid,
             gps.fix_valid,
             gps.valid,
             gps.utc_timestamp);

    if (!imu.valid)
    {
        ESP_LOGW(TAG, "No se puede grabar: BNO055 no válido");
        return ESP_ERR_INVALID_STATE;
    }

    if (!datalogger_utc_valid(&gps))
    {
        ESP_LOGW(TAG, "No se puede grabar: fecha/hora GPS no válida");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(500U)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    if (s_recording)
    {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    //
    // "w" crea un registro limpio. El fichero anterior solo se destruye
    // cuando el usuario pulsa explícitamente GRABAR.
    //
    s_file = fopen(DATALOGGER_FILE_PATH, "w");

    if (s_file == NULL)
    {
        xSemaphoreGive(s_mutex);
        ESP_LOGE(TAG, "No se pudo crear el fichero CSV");
        return ESP_FAIL;
    }

    if (fprintf(s_file, "utc,accel_z_ms2,pressure_hpa\n") <= 0)
    {
        fclose(s_file);
        s_file = NULL;
        xSemaphoreGive(s_mutex);
        return ESP_FAIL;
    }

    if (fflush(s_file) != 0)
    {
        fclose(s_file);
        s_file = NULL;
        xSemaphoreGive(s_mutex);
        return ESP_FAIL;
    }

    s_samples = 0U;
    s_recording = true;

    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Grabación iniciada: %s", DATALOGGER_FILE_PATH);

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
esp_err_t DataLogger_stop_recording(void)
{
    if (!s_mounted || (s_mutex == NULL))
        return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(1000U)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    if (!s_recording)
    {
        xSemaphoreGive(s_mutex);
        return ESP_OK;
    }

    s_recording = false;

    esp_err_t result = ESP_OK;

    if (s_file != NULL)
    {
        if (fflush(s_file) != 0)
            result = ESP_FAIL;

        if (fclose(s_file) != 0)
            result = ESP_FAIL;

        s_file = NULL;
    }

    const uint32_t samples = s_samples;

    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Grabación detenida: %" PRIu32 " muestras", samples);

    return result;
}

/* -------------------------------------------------------------------------- */
datalogger_status_t DataLogger_get_status(void)
{
    datalogger_status_t status = {0};

    if (s_mutex == NULL)
        return status;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100U)) == pdTRUE)
    {
        status.mounted = s_mounted;
        status.recording = s_recording;
        status.samples = s_samples;

        xSemaphoreGive(s_mutex);
    }

    status.file_size_bytes = datalogger_file_size();

    status.file_available = status.file_size_bytes > 0U;

    return status;
}

/* -------------------------------------------------------------------------- */
const char *DataLogger_get_file_path(void)
{
    return DATALOGGER_FILE_PATH;
}
