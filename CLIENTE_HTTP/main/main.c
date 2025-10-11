#include <string.h>
#include <sys/param.h>
#include <stdlib.h>
#include <ctype.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_http_client.h"

#define MAX_HTTP_OUTPUT_BUFFER 4096
static const char *TAG = "HTTP_CLIENT";

typedef struct
{
    int hour;
    float price;
}pvpc_struct;

pvpc_struct myPVPC[24]       = { 0 };

void convert_to_pvpc(pvpc_struct *myPVPC, char *text);

static char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};     // buffer donde se guardará la respuesta completa 
static int output_len = 0;                                           // variable estática para acumular los datos recibidos


//------------------------------------------------------------------------------------------------------
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
        case HTTP_EVENT_ERROR:
            ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
            break;

        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;

        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
            break;

        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;

        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (evt->user_data) 
            {
                // Acumular datos en el buffer
                int copy_len = MIN(evt->data_len, MAX_HTTP_OUTPUT_BUFFER - output_len - 1);
                if (copy_len > 0) 
                {
                    memcpy((char*)evt->user_data + output_len, evt->data, copy_len);
                    output_len += copy_len;
                }
            }
            break;

        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
            output_len = 0;  // reiniciar el contador
            break;

        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            break;

        case HTTP_EVENT_REDIRECT:
            ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
            esp_http_client_set_header(evt->client, "From", "user@example.com");
            esp_http_client_set_header(evt->client, "Accept", "text/html");
            esp_http_client_set_redirection(evt->client);
            break;
    }
    return ESP_OK;
}

//------------------------------------------------------------------------------------------------------
static void https_with_url(void)
{
    esp_http_client_config_t config = {
        //.url = "https://apidatos.ree.es/es/datos/mercados/precios-mercados-tiempo-real?start_date=2024-11-17T00:00&end_date=2024-11-17T23:59&time_trunc=hour&geo_limit=peninsular",
        .url = "https://catfact.ninja/fact",                                   //devuelve historias de gatos
        .event_handler = _http_event_handler,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .user_data = local_response_buffer,                                    // buffer donde se guarda el cuerpo
    };

    ESP_LOGI(TAG, "HTTPS request with url => %s", config.url);

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) 
    {
        ESP_LOGI(TAG, "HTTPS Status = %d, content_length = %" PRId64,
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
        
        ESP_LOGI(TAG, "Response:\n%s", local_response_buffer);               // Mostrar respuesta completa

        // convert_to_pvpc(myPVPC,local_response_buffer);                         // Extrae la info del buffer

        // for (int i = 0; i < 24; i++) 
        //     printf("Hora %02d -> %.3f €/MWh\n", myPVPC[i].hour, myPVPC[i].price);

    } else {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}
//------------------------------------------------------------------------------------------------------
void convert_to_pvpc(pvpc_struct *myPVPC, char *text)
{
    int count = 0;
    const char *ptr = text;

    // Busca cada aparición de {"value":...}
    while ((ptr = strstr(ptr, "{\"value\":")) != NULL) 
    {
        ptr += strlen("{\"value\":");
        float value;

        if (sscanf(ptr, "%f", &value) == 1) 
        {
            if (count < 24) 
            {
                myPVPC[count].hour = count;
                myPVPC[count].price = value;
                count++;
            }
        }
        ptr++; // Avanza para seguir buscando
    }
}
//------------------------------------------------------------------------------------------------------

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    ESP_LOGI(TAG, "Conectado al Router, comienza el ejemplo");

    https_with_url();

    while (1) 
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
