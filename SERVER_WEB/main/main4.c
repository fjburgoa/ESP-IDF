/*

Podamos el ejemplo simple, lo podamos y lo dejamos sencillito para facilitar la comprensión.
Servicios GET / POST


*/
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include "protocol_examples_common.h"
#include "protocol_examples_utils.h"
#include <esp_https_server.h>
#include "esp_tls.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "esp_check.h"
#include "esp_tls_crypto.h"



 


#define EXAMPLE_HTTP_QUERY_KEY_MAX_LEN  (64)

static const char *TAG = "example";

 


// Our URI handler function to be called during GET /uri request 
esp_err_t get_handler(httpd_req_t *req)
{
    // Send a simple response 
    const char resp[] = "URI GET Response";
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}



// Our URI handler function to be called during POST /uri request 
esp_err_t post_handler(httpd_req_t *req)
{
    // Destination buffer for content of HTTP POST request.
    // httpd_req_recv() accepts char* only, but content could
    // as well be any binary data (needs type casting).
    // In case of string data, null termination will be absent, and
    // content length would give length of string 
    char content[100];

    // Truncate if content length larger than the buffer 
    size_t recv_size = MIN(req->content_len, sizeof(content));

    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {  // 0 return value indicates connection closed 
        // Check if timeout occurred 
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            // In case of timeout one can choose to retry calling
            // httpd_req_recv(), but to keep it simple, here we
            // respond with an HTTP 408 (Request Timeout) error 
            httpd_resp_send_408(req);
        }
        // In case of error, returning ESP_FAIL will
        // ensure that the underlying socket is closed 
        return ESP_FAIL;
    }

    // Send a simple response
    const char resp[] = "URI POST Response";
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}


httpd_uri_t uri_get = {
    .uri       = "/get",
    .method    = HTTP_GET,
    .handler   = get_handler,
    .user_ctx  = NULL
};

httpd_uri_t uri_post = {
    .uri       = "/post",
    .method    = HTTP_POST,
    .handler   = post_handler,
    .user_ctx  = NULL
};



//-------------------------------------------------------------------------------------
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
 
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);  // Start the httpd server
    if (httpd_start(&server, &config) == ESP_OK) 
    {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &uri_post);

        return server;
    }
   
    return NULL;
}


 


//-------------------------------------------------------------------------------------
static esp_err_t stop_webserver(httpd_handle_t server)
{    
    return httpd_stop(server); // Stop the httpd server
}

//-------------------------------------------------------------------------------------
static void disconnect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        if (stop_webserver(*server) == ESP_OK) 
        {
            *server = NULL;
        } else {
            ESP_LOGE(TAG, "Failed to stop http server");
        }
    }
}

//-------------------------------------------------------------------------------------
static void connect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) 
    {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}

//-------------------------------------------------------------------------------------
void app_main(void)
{
    static httpd_handle_t server = NULL;

    nvs_flash_init();                                             // inicializa la nvs, ahí está el ssid y el password
    esp_netif_init();                                             // inicializa el stack TCP/IP
    esp_event_loop_create_default();                              // crea un gestor de eventos 

    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server);
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server);
    example_connect();    
    server = start_webserver();                                    // Start the server for the first time  

    while (server)
    {
        sleep(5);
    }
}