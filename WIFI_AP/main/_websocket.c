#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "websocket.h"

#define TELEMETRY_PERIOD_MS 40U
#define MAX_WS_CLIENTS 4U
#define JSON_BUFFER_SIZE 256U
#define QNH_DEFAULT_HPA 1013.25f
#define QNH_STEP_HPA 0.10f
#define QNH_MIN_HPA 800.00f
#define QNH_MAX_HPA 1100.00f

static const char *TAG = "WEBSOCKET";
static TaskHandle_t s_dummy_task = NULL;
static portMUX_TYPE s_qnh_mux = portMUX_INITIALIZER_UNLOCKED;
static float s_qnh_hpa = QNH_DEFAULT_HPA;

//---------------------------------------------------------------------------------

typedef struct
{
    httpd_handle_t server;
    char json[JSON_BUFFER_SIZE];
} websocket_work_t;

//---------------------------------------------------------------------------------

static float qnh_get(void)
{
    float qnh;

    portENTER_CRITICAL(&s_qnh_mux);
    qnh = s_qnh_hpa;
    portEXIT_CRITICAL(&s_qnh_mux);

    return qnh;
}

//---------------------------------------------------------------------------------

static void qnh_change(float increment_hpa)
{
    portENTER_CRITICAL(&s_qnh_mux);

    s_qnh_hpa += increment_hpa;

    if (s_qnh_hpa < QNH_MIN_HPA)
    {
        s_qnh_hpa = QNH_MIN_HPA;
    }
    else if (s_qnh_hpa > QNH_MAX_HPA)
    {
        s_qnh_hpa = QNH_MAX_HPA;
    }

    portEXIT_CRITICAL(&s_qnh_mux);
}

//---------------------------------------------------------------------------------

static esp_err_t websocket_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "Cliente WebSocket conectado, fd=%d", httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err != ESP_OK || frame.len == 0U)
    {
        return err;
    }

    uint8_t *payload = calloc(1U, frame.len + 1U);
    if (payload == NULL)
        return ESP_ERR_NO_MEM;
    frame.payload = payload;
    err = httpd_ws_recv_frame(req, &frame, frame.len);
    if (err == ESP_OK)
    {
        payload[frame.len] = '\0';
        ESP_LOGI(TAG, "Mensaje del navegador: %s", (char *)payload);

        if (strcmp((char *)payload, "QNH_UP") == 0)
        {
            qnh_change(QNH_STEP_HPA);
            ESP_LOGI(TAG, "QNH incrementado: %.2f hPa", (double)qnh_get());
        }
        else if (strcmp((char *)payload, "QNH_DOWN") == 0)
        {
            qnh_change(-QNH_STEP_HPA);
            ESP_LOGI(TAG, "QNH decrementado: %.2f hPa", (double)qnh_get());
        }
        else
        {
            ESP_LOGW(TAG, "Comando WebSocket no reconocido");
        }
    }
    free(payload);
    return err;
}

//---------------------------------------------------------------------------------
static void websocket_broadcast_work(void *arg)
{
    websocket_work_t *work = arg;
    if (work == NULL || work->server == NULL)
    {
        free(work);
        return;
    }

    int client_fds[MAX_WS_CLIENTS];
    size_t client_count = MAX_WS_CLIENTS;
    if (httpd_get_client_list(work->server, &client_count, client_fds) == ESP_OK)
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
            int fd = client_fds[i];
            if (httpd_ws_get_fd_info(work->server, fd) == HTTPD_WS_CLIENT_WEBSOCKET)
            {
                esp_err_t err = httpd_ws_send_frame_async(work->server, fd, &frame);
                if (err != ESP_OK)
                {
                    ESP_LOGD(TAG, "Error enviando a fd=%d: %s", fd, esp_err_to_name(err));
                }
            }
        }
    }
    free(work);
}

//---------------------------------------------------------------------------------

static void dummy_telemetry_task(void *arg)
{
    httpd_handle_t server = (httpd_handle_t)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(TELEMETRY_PERIOD_MS);
    const float pi = 3.14159265358979323846f;
    uint32_t counter = 0U;

    for (;;)
    {
        float t = (float)esp_timer_get_time() / 1000000.0f;
        float roll = 30.0f * sinf((2.0f * pi / 8.0f) * t);
        float pitch = 12.0f * sinf((2.0f * pi / 5.0f) * t);
        float yaw = fmodf(15.0f * t, 360.0f);
        float altitude = 730.0f + 20.0f * sinf((2.0f * pi / 20.0f) * t);
        float vertical_speed = 20.0f * (2.0f * pi / 20.0f) * cosf((2.0f * pi / 20.0f) * t);
        float qnh = qnh_get();

        websocket_work_t *work = calloc(1U, sizeof(*work));

        if (work != NULL)
        {
            work->server = server;
            snprintf(work->json, sizeof(work->json),
                     "{\"counter\":%" PRIu32 ",\"time_s\":%.3f,"
                     "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
                     "\"altitude\":%.2f,\"vertical_speed\":%.2f,\"qnh\":%.2f}",
                     counter++, (double)t, (double)roll, (double)pitch,
                     (double)yaw, (double)altitude, (double)vertical_speed,
                     (double)qnh);

            esp_err_t err = httpd_queue_work(server, websocket_broadcast_work, work);
            if (err != ESP_OK)
                free(work);
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

//---------------------------------------------------------------------------------

esp_err_t websocket_register_uri(httpd_handle_t server)
{
    if (server == NULL)
        return ESP_ERR_INVALID_ARG;
    const httpd_uri_t uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = websocket_handler,
        .user_ctx = NULL,
        .is_websocket = true,
    };
    return httpd_register_uri_handler(server, &uri);
}

//---------------------------------------------------------------------------------
esp_err_t websocket_start_dummy_stream(httpd_handle_t server)
{
    if (server == NULL)
        return ESP_ERR_INVALID_ARG;
    if (s_dummy_task != NULL)
        return ESP_ERR_INVALID_STATE;

    BaseType_t ok = xTaskCreate(dummy_telemetry_task, "dummy_telemetry", 4096, server, 5, &s_dummy_task);
    if (ok != pdPASS)
    {
        s_dummy_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Telemetría dummy iniciada a %u Hz", 1000U / TELEMETRY_PERIOD_MS);
    return ESP_OK;
}
