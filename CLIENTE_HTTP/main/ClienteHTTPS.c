
#include <sys/param.h>
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_http_client.h"
#include "driver/gpio.h"

#define PULS 0
#define MAX_HTTP_OUT_BUFF 4096

/*
LEMD = Madrid-Barajas
LEBL = Barcelona
LEVC = Valencia
LEZG = Zaragoza
LECU = Cuatro Vientos
LEMT = Torrejón
*/

static int extract_qnh_hpa(const char *metar);

static char local_response_buffer[MAX_HTTP_OUT_BUFF] = {0}; // buffer
static int output_len = 0;

//---------------------------------------------------------------------------------
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        printf("HTTP_EVENT_ERROR\n");
        break;

    case HTTP_EVENT_ON_CONNECTED:
        printf("HTTP_EVENT_ON_CONNECTED\n");
        break;

    case HTTP_EVENT_HEADER_SENT:
        printf("HTTP_EVENT_HEADER_SENT\n");
        break;

    case HTTP_EVENT_ON_HEADER:
        printf("HTTP_EVENT_ON_HEADER, key=%s, value=%s\n", evt->header_key,
               evt->header_value);
        break;

    case HTTP_EVENT_ON_DATA: // puede ejecutarse múltiples veces
        printf("HTTP_EVENT_ON_DATA, len=%d\n", evt->data_len);
        if (evt->user_data)
        {
            // Acumular datos en el buffer
            int copy_len = MIN(evt->data_len, MAX_HTTP_OUT_BUFF - output_len - 1);
            if (copy_len > 0)
            {
                memcpy((char *)evt->user_data + output_len, evt->data, copy_len);
                output_len += copy_len;
            }
        }
        break;

    case HTTP_EVENT_ON_FINISH:
        printf("HTTP_EVENT_ON_FINISH\n");
        output_len = 0; // reiniciar el contador
        break;

    case HTTP_EVENT_DISCONNECTED:
        printf("HTTP_EVENT_DISCONNECTED\n");
        break;

    case HTTP_EVENT_REDIRECT:
        printf("HTTP_EVENT_REDIRECT\n");
        esp_http_client_set_header(evt->client, "From", "user@example.com");
        esp_http_client_set_header(evt->client, "Accept", "text/html");
        esp_http_client_set_redirection(evt->client);
        break;
    }
    return ESP_OK;
}

//--------------------------------------------------------------------------------
static void https_with_url(void)
{
    esp_http_client_config_t config = {
        //.url = "https://catfact.ninja/fact",        // URL de prueba
        //.url = "https://aviationweather.gov/api/data/metar?ids=LEMD&format=raw",
        //.url = "https://opensky-network.org/api/states/all?lamin=39.8&lomin=-4.5&lamax=41.0&lomax=-2.8",
        .url = "https://api.airplanes.live/v2/point/40.4168/-3.7038/25",
        .event_handler = _http_event_handler,       // callback
        .crt_bundle_attach = esp_crt_bundle_attach, // certificado
        .user_data = local_response_buffer,         // buffer
    };

    printf("Petición HTTPS a la URL:  => %s\n", config.url);

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK)
    {
        printf("HTTPS Status = %d, content_length = %llu\n",
               esp_http_client_get_status_code(client),
               esp_http_client_get_content_length(client));

        printf("Response: \n %s\n", local_response_buffer); // Mostrar respuesta

        int QNH = extract_qnh_hpa(local_response_buffer);

        printf("QNH: %d hPa\n", QNH);

        memset(local_response_buffer, 0, MAX_HTTP_OUT_BUFF);
    }
    else
    {
        printf("Error perform http request %s\n", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

/***************************************************************/

static int extract_qnh_hpa(const char *metar)
{
    const char *q = strstr(metar, " Q");
    if (!q)
        q = strstr(metar, "\nQ");
    if (!q)
        return -1;

    q++; // apunta a 'Q'

    if (q[0] == 'Q' &&
        q[1] >= '0' && q[1] <= '9' &&
        q[2] >= '0' && q[2] <= '9' &&
        q[3] >= '0' && q[3] <= '9' &&
        q[4] >= '0' && q[4] <= '9')
    {
        return atoi(&q[1]);
    }

    return -1;
}

/***************************************************************/

//--------------------------------------------------------------------------------
int pulsador = 0;
int pulsador1 = 0;

void app_main(void)
{
    // configura la entrada digital
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << PULS,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);
    // configura la memoria nvs
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }

    // conecta con la SSID y PASSWORD definida en protocol_examples_common
    esp_netif_init();
    esp_event_loop_create_default();
    example_connect();

    printf("\n Conectado al Router, comienza el ejemplo\n");
    printf("Pulsa BOOT para saber el QNH de Madrid Barajas\n");

    while (1)
    {
        // detección de flanco
        pulsador = gpio_get_level(PULS);
        if ((pulsador != pulsador1) && (pulsador == 0))
        {
            printf("Pulsador activado\n");
            https_with_url(); // petición de URL en cada flanco de BOOT
        }
        pulsador1 = pulsador; // actualiza valor con el actual

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
