#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "websocket.h"

#define TELEMETRY_PERIOD_MS 80U
#define MAX_WS_CLIENTS 4U
#define JSON_BUFFER_SIZE 576U

#define QNH_DEFAULT_HPA 1013.25f
#define QNH_STEP_HPA 0.10f
#define QNH_MIN_HPA 800.00f
#define QNH_MAX_HPA 1100.00f

#define PITCH_OFFSET_DEFAULT_DEG 0
#define PITCH_OFFSET_STEP_DEG 1
#define PITCH_OFFSET_MIN_DEG -90
#define PITCH_OFFSET_MAX_DEG 90

#define HEADING_OFFSET_DEFAULT_DEG 0U
#define HEADING_OFFSET_STEP_DEG 1U
#define HEADING_OFFSET_MAX_DEG 359U

#define NVS_NAMESPACE "flight_cfg"
#define NVS_KEY_QNH_X100 "qnh_x100"
#define NVS_KEY_PITCH_OFFSET "pitch_off"
#define NVS_KEY_HEAD_OFFSET "head_off"

static const char *TAG = "WEBSOCKET";

static TaskHandle_t s_dummy_task = NULL;
static portMUX_TYPE s_settings_mux = portMUX_INITIALIZER_UNLOCKED;

static float s_qnh_hpa = QNH_DEFAULT_HPA;
static int32_t s_pitch_offset_deg = PITCH_OFFSET_DEFAULT_DEG;
static uint32_t s_heading_offset_deg = HEADING_OFFSET_DEFAULT_DEG;

/*
 * Mayor valor de aceleración recibido en valor absoluto.
 * Se conserva el signo del valor que produjo el máximo.
 * No se guarda en NVS: se reinicia con cada arranque o con G_RESET.
 */
static float s_g_peak = 0.0f;

typedef struct
{
    httpd_handle_t server;
    char json[JSON_BUFFER_SIZE];
} websocket_work_t;

typedef struct
{
    float qnh_hpa;
    int32_t pitch_offset_deg;
    uint32_t heading_offset_deg;
} settings_snapshot_t;

static settings_snapshot_t settings_get_snapshot(void)
{
    settings_snapshot_t snapshot;

    portENTER_CRITICAL(&s_settings_mux);
    snapshot.qnh_hpa = s_qnh_hpa;
    snapshot.pitch_offset_deg = s_pitch_offset_deg;
    snapshot.heading_offset_deg = s_heading_offset_deg;
    portEXIT_CRITICAL(&s_settings_mux);

    return snapshot;
}

static esp_err_t settings_save_i32(const char *key, int32_t value)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo abrir NVS: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_i32(handle, key, value);

    if (err == ESP_OK)
    {
        err = nvs_commit(handle);
    }

    nvs_close(handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error guardando %s en NVS: %s",
                 key, esp_err_to_name(err));
    }

    return err;
}

static esp_err_t settings_save_u32(const char *key, uint32_t value)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo abrir NVS: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_u32(handle, key, value);

    if (err == ESP_OK)
    {
        err = nvs_commit(handle);
    }

    nvs_close(handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error guardando %s en NVS: %s",
                 key, esp_err_to_name(err));
    }

    return err;
}

static void settings_load(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "NVS sin configuración previa; se usan valores por defecto");
        return;
    }

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "No se pudo leer NVS: %s", esp_err_to_name(err));
        return;
    }

    int32_t qnh_x100 = (int32_t)lroundf(QNH_DEFAULT_HPA * 100.0f);
    int32_t pitch_offset = PITCH_OFFSET_DEFAULT_DEG;
    uint32_t heading_offset = HEADING_OFFSET_DEFAULT_DEG;

    (void)nvs_get_i32(handle, NVS_KEY_QNH_X100, &qnh_x100);
    (void)nvs_get_i32(handle, NVS_KEY_PITCH_OFFSET, &pitch_offset);
    (void)nvs_get_u32(handle, NVS_KEY_HEAD_OFFSET, &heading_offset);

    nvs_close(handle);

    float qnh = (float)qnh_x100 / 100.0f;

    if ((qnh < QNH_MIN_HPA) || (qnh > QNH_MAX_HPA))
    {
        qnh = QNH_DEFAULT_HPA;
    }

    if (pitch_offset < PITCH_OFFSET_MIN_DEG)
    {
        pitch_offset = PITCH_OFFSET_MIN_DEG;
    }
    else if (pitch_offset > PITCH_OFFSET_MAX_DEG)
    {
        pitch_offset = PITCH_OFFSET_MAX_DEG;
    }

    heading_offset %= 360U;

    portENTER_CRITICAL(&s_settings_mux);
    s_qnh_hpa = qnh;
    s_pitch_offset_deg = pitch_offset;
    s_heading_offset_deg = heading_offset;
    portEXIT_CRITICAL(&s_settings_mux);

    ESP_LOGI(TAG,
             "Configuración recuperada: QNH=%.2f hPa, pitch offset=%" PRId32
             " deg, heading offset=%" PRIu32 " deg",
             (double)qnh,
             pitch_offset,
             heading_offset);
}

static void qnh_change(float increment_hpa)
{
    float new_value;

    portENTER_CRITICAL(&s_settings_mux);

    s_qnh_hpa += increment_hpa;

    if (s_qnh_hpa < QNH_MIN_HPA)
    {
        s_qnh_hpa = QNH_MIN_HPA;
    }
    else if (s_qnh_hpa > QNH_MAX_HPA)
    {
        s_qnh_hpa = QNH_MAX_HPA;
    }

    new_value = s_qnh_hpa;
    portEXIT_CRITICAL(&s_settings_mux);

    const int32_t qnh_x100 = (int32_t)lroundf(new_value * 100.0f);
    (void)settings_save_i32(NVS_KEY_QNH_X100, qnh_x100);

    ESP_LOGI(TAG, "QNH actualizado: %.2f hPa", (double)new_value);
}

static void pitch_offset_change(int32_t increment_deg)
{
    int32_t new_value;

    portENTER_CRITICAL(&s_settings_mux);

    s_pitch_offset_deg += increment_deg;

    if (s_pitch_offset_deg < PITCH_OFFSET_MIN_DEG)
    {
        s_pitch_offset_deg = PITCH_OFFSET_MIN_DEG;
    }
    else if (s_pitch_offset_deg > PITCH_OFFSET_MAX_DEG)
    {
        s_pitch_offset_deg = PITCH_OFFSET_MAX_DEG;
    }

    new_value = s_pitch_offset_deg;
    portEXIT_CRITICAL(&s_settings_mux);

    (void)settings_save_i32(NVS_KEY_PITCH_OFFSET, new_value);

    ESP_LOGI(TAG, "Offset de pitch actualizado: %" PRId32 " deg", new_value);
}

static void heading_offset_change(int32_t increment_deg)
{
    uint32_t new_value;

    portENTER_CRITICAL(&s_settings_mux);

    int32_t value = (int32_t)s_heading_offset_deg + increment_deg;

    while (value < 0)
    {
        value += 360;
    }

    while (value >= 360)
    {
        value -= 360;
    }

    s_heading_offset_deg = (uint32_t)value;
    new_value = s_heading_offset_deg;

    portEXIT_CRITICAL(&s_settings_mux);

    (void)settings_save_u32(NVS_KEY_HEAD_OFFSET, new_value);

    ESP_LOGI(TAG, "Offset de rumbo actualizado: %" PRIu32 " deg", new_value);
}


static float g_peak_update(float g_value)
{
    float peak;

    portENTER_CRITICAL(&s_settings_mux);

    if (fabsf(g_value) > fabsf(s_g_peak))
    {
        s_g_peak = g_value;
    }

    peak = s_g_peak;

    portEXIT_CRITICAL(&s_settings_mux);

    return peak;
}

static void g_peak_reset(void)
{
    portENTER_CRITICAL(&s_settings_mux);
    s_g_peak = 0.0f;
    portEXIT_CRITICAL(&s_settings_mux);

    ESP_LOGI(TAG, "Indicador de G máxima reseteado a 0");
}

static esp_err_t websocket_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "Cliente WebSocket conectado, fd=%d",
                 httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);

    if ((err != ESP_OK) || (frame.len == 0U))
    {
        return err;
    }

    uint8_t *payload = calloc(1U, frame.len + 1U);

    if (payload == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    frame.payload = payload;
    err = httpd_ws_recv_frame(req, &frame, frame.len);

    if (err == ESP_OK)
    {
        payload[frame.len] = '\0';
        ESP_LOGI(TAG, "Mensaje del navegador: %s", (char *)payload);

        if (strcmp((char *)payload, "QNH_UP") == 0)
        {
            qnh_change(QNH_STEP_HPA);
        }
        else if (strcmp((char *)payload, "QNH_DOWN") == 0)
        {
            qnh_change(-QNH_STEP_HPA);
        }
        else if (strcmp((char *)payload, "PITCH_OFFSET_UP") == 0)
        {
            pitch_offset_change(PITCH_OFFSET_STEP_DEG);
        }
        else if (strcmp((char *)payload, "PITCH_OFFSET_DOWN") == 0)
        {
            pitch_offset_change(-PITCH_OFFSET_STEP_DEG);
        }
        else if (strcmp((char *)payload, "HEADING_OFFSET_UP") == 0)
        {
            heading_offset_change((int32_t)HEADING_OFFSET_STEP_DEG);
        }
        else if (strcmp((char *)payload, "HEADING_OFFSET_DOWN") == 0)
        {
            heading_offset_change(-(int32_t)HEADING_OFFSET_STEP_DEG);
        }
        else if (strcmp((char *)payload, "G_RESET") == 0)
        {
            g_peak_reset();
        }
    }

    free(payload);
    return err;
}

static void websocket_broadcast_work(void *arg)
{
    websocket_work_t *work = arg;

    if ((work == NULL) || (work->server == NULL))
    {
        free(work);
        return;
    }

    int client_fds[MAX_WS_CLIENTS];
    size_t client_count = MAX_WS_CLIENTS;

    if (httpd_get_client_list(work->server,
                              &client_count,
                              client_fds) == ESP_OK)
    {
        httpd_ws_frame_t frame = {
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)work->json,
            .len = strlen(work->json),
        };

        for (size_t i = 0; i < client_count; ++i)
        {
            const int fd = client_fds[i];

            if (httpd_ws_get_fd_info(work->server, fd) ==
                HTTPD_WS_CLIENT_WEBSOCKET)
            {
                esp_err_t err = httpd_ws_send_frame_async(
                    work->server,
                    fd,
                    &frame);

                if (err != ESP_OK)
                {
                    ESP_LOGD(TAG,
                             "Error enviando a fd=%d: %s",
                             fd,
                             esp_err_to_name(err));
                }
            }
        }
    }

    free(work);
}

static void dummy_telemetry_task(void *arg)
{
    httpd_handle_t server = (httpd_handle_t)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(TELEMETRY_PERIOD_MS);
    const float pi = 3.14159265358979323846f;
    uint32_t counter = 0U;

    float roll = 0.0f;
    float pitch = 0.0f;

    for (;;)
    {
        const float t = (float)esp_timer_get_time() / 1000000.0f;

        roll += 0.1f;
        if (roll > 30.0f)
        {
            roll = -30.0f;
        }

        pitch += 0.1f;
        if (pitch > 40.0f)
        {
            pitch = -40.0f;
        }

        const float yaw = fmodf(15.0f * t, 360.0f);

        const float altitude =
            730.0f + 20.0f * sinf((2.0f * pi / 20.0f) * t);

        const float vertical_speed =
            20.0f * (2.0f * pi / 20.0f) *
            cosf((2.0f * pi / 20.0f) * t);

        const float turn_rate_dps =
            3.0f * sinf((2.0f * pi / 12.0f) * t);

        const float force_direction_deg =
            18.0f * sinf((2.0f * pi / 7.0f) * t + pi / 3.0f);

        /*
         * Señal dummy del acelerómetro, expresada directamente en G.
         * No debe dividirse entre 9.80665 porque no está en m/s².
         * Oscila aproximadamente entre -4.2 G y +4.2 G para comprobar
         * que el indicador conserva el valor de mayor módulo y su signo.
         */
        float g_load =
            3.65f * sinf((2.0f * pi / 11.0f) * t) +
            0.55f * sinf((2.0f * pi / 2.3f) * t);

        if (g_load > 5.0f)
        {
            g_load = 5.0f;
        }
        else if (g_load < -5.0f)
        {
            g_load = -5.0f;
        }

        const float g_peak = g_peak_update(g_load);
        const settings_snapshot_t settings = settings_get_snapshot();

        websocket_work_t *work = calloc(1U, sizeof(*work));

        if (work != NULL)
        {
            work->server = server;

            snprintf(
                work->json,
                sizeof(work->json),
                "{"
                "\"counter\":%" PRIu32 ","
                "\"time_s\":%.3f,"
                "\"roll\":%.2f,"
                "\"pitch\":%.2f,"
                "\"yaw\":%.2f,"
                "\"altitude\":%.2f,"
                "\"vertical_speed\":%.2f,"
                "\"qnh\":%.2f,"
                "\"pitch_offset_deg\":%" PRId32 ","
                "\"heading_offset_deg\":%" PRIu32 ","
                "\"turn_rate_dps\":%.2f,"
                "\"force_direction_deg\":%.2f,"
                "\"g_load\":%.2f,"
                "\"g_peak\":%.2f"
                "}",
                counter++,
                (double)t,
                (double)roll,
                (double)pitch,
                (double)yaw,
                (double)altitude,
                (double)vertical_speed,
                (double)settings.qnh_hpa,
                settings.pitch_offset_deg,
                settings.heading_offset_deg,
                (double)turn_rate_dps,
                (double)force_direction_deg,
                (double)g_load,
                (double)g_peak);

            esp_err_t err = httpd_queue_work(
                server,
                websocket_broadcast_work,
                work);

            if (err != ESP_OK)
            {
                free(work);
            }
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

esp_err_t websocket_register_uri(httpd_handle_t server)
{
    if (server == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const httpd_uri_t uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = websocket_handler,
        .user_ctx = NULL,
        .is_websocket = true,
    };

    return httpd_register_uri_handler(server, &uri);
}

esp_err_t websocket_start_dummy_stream(httpd_handle_t server)
{
    if (server == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_dummy_task != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    settings_load();

    BaseType_t ok = xTaskCreate(
        dummy_telemetry_task,
        "dummy_telemetry",
        4096,
        server,
        5,
        &s_dummy_task);

    if (ok != pdPASS)
    {
        s_dummy_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG,
             "Telemetría dummy iniciada a %u Hz",
             1000U / TELEMETRY_PERIOD_MS);

    return ESP_OK;
}
