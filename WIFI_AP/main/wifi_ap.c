#include <string.h>
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "wifi_ap.h"

#define WIFI_AP_SSID "ESP32-FlightDisplay"
#define WIFI_AP_PASSWORD "esp32s3test"
#define WIFI_AP_CHANNEL 6
#define WIFI_AP_MAX_CLIENTS 4

static const char *TAG = "WIFI_AP";

//---------------------------------------------------------------------------------------------------------------
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;

    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        const wifi_event_ap_staconnected_t *event = event_data;
        ESP_LOGI(TAG, "Cliente conectado al AP, AID=%d", event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        const wifi_event_ap_stadisconnected_t *event = event_data;
        ESP_LOGI(TAG, "Cliente desconectado del AP, AID=%d", event->aid);
    }
}

//---------------------------------------------------------------------------------------------------------------
esp_err_t wifi_ap_start(void)
{
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "Error inicializando esp_netif");

    esp_err_t err = esp_event_loop_create_default();
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE))
    {
        ESP_LOGE(TAG, "Error creando el bucle de eventos: %s", esp_err_to_name(err));
        return err;
    }

    if (esp_netif_create_default_wifi_ap() == NULL)
    {
        return ESP_FAIL;
    }

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_config), TAG, "Error inicializando Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
                            WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler,
                            NULL, NULL),
                        TAG, "Error registrando eventos Wi-Fi");

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = sizeof(WIFI_AP_SSID) - 1,
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASSWORD,
            .max_connection = WIFI_AP_MAX_CLIENTS,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {.required = false},
        },
    };

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "Error seleccionando modo AP");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wifi_config), TAG, "Error configurando AP");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Error arrancando Wi-Fi");

    ESP_LOGI(TAG, "SSID: %s", WIFI_AP_SSID);
    ESP_LOGI(TAG, "Contraseña: %s", WIFI_AP_PASSWORD);
    ESP_LOGI(TAG, "Abra: http://192.168.4.1");
    return ESP_OK;
}
