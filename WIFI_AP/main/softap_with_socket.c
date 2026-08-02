#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define WIFI_SSID "ESP32-FlightDisplay"
#define WIFI_PASSWORD "esp32s3test"
#define WIFI_CHANNEL 6
#define WIFI_MAX_CONNECTIONS 4

#define TELEMETRY_PERIOD_MS 80U // 12,5 Hz
#define HTTP_MAX_CLIENTS 8U

static const char *TAG = "FLIGHT_DISPLAY";
static httpd_handle_t s_http_server = NULL;

typedef struct
{
    httpd_handle_t server;
    int socket_fd;
    size_t length;
    char payload[64];
} websocket_send_job_t;

/* Página almacenada directamente en la Flash del ESP32-S3. */
static const char INDEX_HTML[] =
    "<!doctype html>"
    "<html lang='es'>"
    "<head>"
    "  <meta charset='utf-8'>"
    "  <meta name='viewport' content='width=device-width,initial-scale=1'>"
    "  <title>ESP32-S3 WebSocket</title>"
    "  <style>"
    "    *{box-sizing:border-box}"
    "    body{margin:0;min-height:100vh;display:grid;place-items:center;"
    "         font-family:Arial,sans-serif;background:#101820;color:#fff}"
    "    .card{width:min(90vw,420px);padding:32px;border-radius:18px;"
    "          background:#1b2836;text-align:center;box-shadow:0 12px 35px #0008}"
    "    h1{font-size:1.25rem;margin:0 0 24px}"
    "    #counter{font-size:5rem;font-weight:700;font-variant-numeric:tabular-nums}"
    "    #status{margin-top:20px;font-size:.95rem;color:#ffb74d}"
    "    .info{margin-top:10px;color:#b7c5d3;font-size:.85rem}"
    "  </style>"
    "</head>"
    "<body>"
    "  <main class='card'>"
    "    <h1>ESP32-S3 · Contador a 25 Hz</h1>"
    "    <div id='counter'>0</div>"
    "    <div id='status'>Conectando…</div>"
    "    <div class='info' id='rate'>0 mensajes/s</div>"
    "  </main>"
    "  <script>"
    "    const counterEl = document.getElementById('counter');"
    "    const statusEl = document.getElementById('status');"
    "    const rateEl = document.getElementById('rate');"
    "    let received = 0;"
    "    let ws;"
    "    function connect(){"
    "      ws = new WebSocket(`ws://${location.host}/ws`);"
    "      ws.onopen = () => {statusEl.textContent='WebSocket conectado';statusEl.style.color='#66bb6a';};"
    "      ws.onmessage = (event) => {"
    "        const data = JSON.parse(event.data);"
    "        counterEl.textContent = data.counter;"
    "        received++;"
    "      };"
    "      ws.onclose = () => {"
    "        statusEl.textContent='Desconectado. Reconectando…';"
    "        statusEl.style.color='#ef5350';"
    "        setTimeout(connect,1000);"
    "      };"
    "      ws.onerror = () => ws.close();"
    "    }"
    "    setInterval(()=>{rateEl.textContent=`${received} mensajes/s`;received=0;},1000);"
    "    connect();"
    "  </script>"
    "</body>"
    "</html>";

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t websocket_handler(httpd_req_t *req)
{
    /* Durante el GET se realiza automáticamente el upgrade HTTP -> WebSocket. */
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "Cliente WebSocket conectado, socket=%d", httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    /* Esta aplicación no necesita datos entrantes, pero se consume la trama
       para mantener correctamente el endpoint bidireccional. */
    httpd_ws_frame_t frame = {0};
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "No se pudo consultar la trama WebSocket: %s", esp_err_to_name(err));
        return err;
    }

    if (frame.len > 0)
    {
        uint8_t payload[128];
        if (frame.len >= sizeof(payload))
        {
            ESP_LOGW(TAG, "Trama recibida demasiado grande: %u bytes", (unsigned)frame.len);
            return ESP_ERR_INVALID_SIZE;
        }

        frame.payload = payload;
        err = httpd_ws_recv_frame(req, &frame, frame.len);
        if (err != ESP_OK)
        {
            return err;
        }
        payload[frame.len] = '\0';
        ESP_LOGI(TAG, "WebSocket RX: %s", (char *)payload);
    }

    return ESP_OK;
}

static const httpd_uri_t ROOT_URI = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler,
    .user_ctx = NULL,
};

static const httpd_uri_t WS_URI = {
    .uri = "/ws",
    .method = HTTP_GET,
    .handler = websocket_handler,
    .user_ctx = NULL,
    .is_websocket = true,
};

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;

    httpd_handle_t server = NULL;
    ESP_LOGI(TAG, "Iniciando servidor HTTP en el puerto %u", config.server_port);

    //    ESP_ERROR_CHECK(httpd_start(&server, &config));

    esp_err_t err = httpd_start(&server, &config);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,
                 "No se pudo iniciar el servidor HTTP: %s",
                 esp_err_to_name(err));
        return NULL;
    }

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ROOT_URI));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &WS_URI));

    return server;
}

static void websocket_send_worker(void *arg)
{
    websocket_send_job_t *job = arg;

    httpd_ws_frame_t frame = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)job->payload,
        .len = job->length,
    };

    esp_err_t err = httpd_ws_send_frame_async(job->server, job->socket_fd, &frame);
    if (err != ESP_OK)
    {
        ESP_LOGD(TAG, "Fallo al enviar al socket %d: %s",
                 job->socket_fd, esp_err_to_name(err));
    }

    free(job);
}

static esp_err_t queue_websocket_message(httpd_handle_t server,
                                         int socket_fd,
                                         const char *message,
                                         size_t length)
{
    if (length >= sizeof(((websocket_send_job_t *)0)->payload))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    websocket_send_job_t *job = malloc(sizeof(*job));
    if (job == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    job->server = server;
    job->socket_fd = socket_fd;
    job->length = length;
    memcpy(job->payload, message, length);
    job->payload[length] = '\0';

    esp_err_t err = httpd_queue_work(server, websocket_send_worker, job);
    if (err != ESP_OK)
    {
        free(job);
    }
    return err;
}

static void telemetry_task(void *arg)
{
    uint32_t counter = 0;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(TELEMETRY_PERIOD_MS);

    while (true)
    {
        char json[64];
        const int length = snprintf(json, sizeof(json),
                                    "{\"counter\":%" PRIu32 ",\"period_ms\":%u}",
                                    counter++, TELEMETRY_PERIOD_MS);

        if ((s_http_server != NULL) && (length > 0) && (length < (int)sizeof(json)))
        {
            int client_fds[HTTP_MAX_CLIENTS];
            size_t client_count = HTTP_MAX_CLIENTS;

            if (httpd_get_client_list(s_http_server, &client_count, client_fds) == ESP_OK)
            {
                for (size_t i = 0; i < client_count; ++i)
                {
                    const int fd = client_fds[i];
                    if (httpd_ws_get_fd_info(s_http_server, fd) == HTTPD_WS_CLIENT_WEBSOCKET)
                    {
                        esp_err_t err = queue_websocket_message(
                            s_http_server, fd, json, (size_t)length);
                        if (err != ESP_OK)
                        {
                            ESP_LOGD(TAG, "No se pudo encolar el envío al socket %d: %s",
                                     fd, esp_err_to_name(err));
                        }
                    }
                }
            }
        }

        vTaskDelayUntil(&last_wake_time, period_ticks);
    }
}

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if ((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_AP_STACONNECTED))
    {
        const wifi_event_ap_staconnected_t *event = event_data;
        ESP_LOGI(TAG, "Dispositivo conectado al AP, AID=%d", event->aid);
    }
    else if ((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_AP_STADISCONNECTED))
    {
        const wifi_event_ap_stadisconnected_t *event = event_data;
        ESP_LOGI(TAG, "Dispositivo desconectado del AP, AID=%d", event->aid);
    }
}

static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(ap_netif != NULL);

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = 0,
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASSWORD,
            .max_connection = WIFI_MAX_CONNECTIONS,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    if (strlen(WIFI_PASSWORD) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Punto de acceso iniciado");
    ESP_LOGI(TAG, "SSID: %s", WIFI_SSID);
    ESP_LOGI(TAG, "Contraseña: %s", WIFI_PASSWORD);
    ESP_LOGI(TAG, "Abra en el móvil: http://192.168.4.1");
}

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

    wifi_init_softap();
    s_http_server = start_webserver();

    BaseType_t task_created = xTaskCreate(
        telemetry_task,
        "telemetry_task",
        4096,
        NULL,
        5,
        NULL);

    if (task_created != pdPASS)
    {
        ESP_LOGE(TAG, "No se pudo crear telemetry_task");
        abort();
    }
}
