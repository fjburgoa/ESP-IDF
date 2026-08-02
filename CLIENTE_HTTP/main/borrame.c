#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_err.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "esp_http_client.h"
#include "esp_crt_bundle.h"

#include "cJSON.h"

#define WIFI_SSID "TU_WIFI"
#define WIFI_PASS "TU_PASSWORD"

#define GOOGLE_API_KEY "TU_API_KEY"

#define MAX_APS 10
#define HTTP_BUF_SIZE 2048

static const char *TAG = "wifi_geo";

static char response_buffer[HTTP_BUF_SIZE];
static int response_len = 0;

static void bssid_to_string(const uint8_t bssid[6], char *out, size_t out_len)
{
    snprintf(out, out_len,
             "%02X:%02X:%02X:%02X:%02X:%02X",
             bssid[0], bssid[1], bssid[2],
             bssid[3], bssid[4], bssid[5]);
}

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_DATA)
    {
        if (response_len + evt->data_len < HTTP_BUF_SIZE)
        {
            memcpy(response_buffer + response_len, evt->data, evt->data_len);
            response_len += evt->data_len;
            response_buffer[response_len] = '\0';
        }
    }

    return ESP_OK;
}

static void wifi_connect(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Conectando a WiFi...");
    ESP_ERROR_CHECK(esp_wifi_connect());

    vTaskDelay(pdMS_TO_TICKS(6000));
}

static char *build_wifi_geolocation_json(void)
{
    uint16_t ap_count = MAX_APS;
    wifi_ap_record_t ap_records[MAX_APS];

    memset(ap_records, 0, sizeof(ap_records));

    ESP_LOGI(TAG, "Escaneando redes WiFi...");

    ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, true));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_records));

    ESP_LOGI(TAG, "Redes usadas para geolocalización: %d", ap_count);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "considerIp", true);

    cJSON *aps = cJSON_CreateArray();
    cJSON_AddItemToObject(root, "wifiAccessPoints", aps);

    for (int i = 0; i < ap_count; i++)
    {
        char mac[18];

        bssid_to_string(ap_records[i].bssid, mac, sizeof(mac));

        cJSON *ap = cJSON_CreateObject();
        cJSON_AddStringToObject(ap, "macAddress", mac);
        cJSON_AddNumberToObject(ap, "signalStrength", ap_records[i].rssi);

        cJSON_AddItemToArray(aps, ap);

        ESP_LOGI(TAG, "%s RSSI=%d CH=%d",
                 mac,
                 ap_records[i].rssi,
                 ap_records[i].primary);
    }

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    return json;
}

static esp_err_t google_geolocation_request(const char *json_body)
{
    char url[256];

    snprintf(url, sizeof(url),
             "https://www.googleapis.com/geolocation/v1/geolocate?key=%s",
             GOOGLE_API_KEY);

    response_len = 0;
    memset(response_buffer, 0, sizeof(response_buffer));

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 15000,
        .event_handler = http_event_handler,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_body, strlen(json_body));

    ESP_LOGI(TAG, "Enviando POST a Google Geolocation...");

    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK)
    {
        int status = esp_http_client_get_status_code(client);

        ESP_LOGI(TAG, "HTTP status = %d", status);
        ESP_LOGI(TAG, "Respuesta: %s", response_buffer);
    }
    else
    {
        ESP_LOGE(TAG, "Error HTTP: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);

    return err;
}

static void parse_geolocation_response(void)
{
    cJSON *root = cJSON_Parse(response_buffer);

    if (root == NULL)
    {
        ESP_LOGE(TAG, "No se pudo parsear JSON");
        return;
    }

    cJSON *location = cJSON_GetObjectItem(root, "location");
    cJSON *accuracy = cJSON_GetObjectItem(root, "accuracy");

    if (location == NULL)
    {
        ESP_LOGE(TAG, "No hay campo location");
        cJSON_Delete(root);
        return;
    }

    cJSON *lat = cJSON_GetObjectItem(location, "lat");
    cJSON *lng = cJSON_GetObjectItem(location, "lng");

    if (lat && lng && accuracy)
    {
        ESP_LOGI(TAG, "LAT = %.6f", lat->valuedouble);
        ESP_LOGI(TAG, "LON = %.6f", lng->valuedouble);
        ESP_LOGI(TAG, "Accuracy = %.1f m", accuracy->valuedouble);
    }
    else
    {
        ESP_LOGE(TAG, "Respuesta incompleta");
    }

    cJSON_Delete(root);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_connect();

    char *json_body = build_wifi_geolocation_json();

    if (json_body == NULL)
    {
        ESP_LOGE(TAG, "No se pudo generar JSON");
        return;
    }

    ESP_LOGI(TAG, "JSON enviado:");
    ESP_LOGI(TAG, "%s", json_body);

    esp_err_t err = google_geolocation_request(json_body);

    free(json_body);

    if (err == ESP_OK)
    {
        parse_geolocation_response();
    }
}