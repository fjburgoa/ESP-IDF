#include <stddef.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "webserver.h"
#include "websocket.h"

static const char *TAG = "WEBSERVER";
extern const unsigned char index_html_start[] asm("_binary_index_html_start");
extern const unsigned char index_html_end[] asm("_binary_index_html_end");

//-------------------------------------------------------------------------------------------------------

static esp_err_t root_get_handler(httpd_req_t *req)
{
    const size_t html_length = (size_t)(index_html_end - index_html_start);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)index_html_start, html_length);
}

//-------------------------------------------------------------------------------------------------------

httpd_handle_t webserver_start(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Iniciando servidor HTTP en puerto %u", config.server_port);
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo iniciar HTTP: %s", esp_err_to_name(err));
        return NULL;
    }

    const httpd_uri_t root_uri = {
        .uri = "/", .method = HTTP_GET, .handler = root_get_handler, .user_ctx = NULL};

    if (httpd_register_uri_handler(server, &root_uri) != ESP_OK || websocket_register_uri(server) != ESP_OK)
    {
        httpd_stop(server);
        return NULL;
    }

    ESP_LOGI(TAG, "Servidor HTTP iniciado correctamente");
    return server;
}
