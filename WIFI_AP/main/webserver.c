#include <stddef.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "webserver.h"
#include "websocket.h"
#include "config.h"

#if DATALOGGER_ENABLED
#include "DataLogger.h"
#endif

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

#if DATALOGGER_ENABLED
static esp_err_t download_log_get_handler(httpd_req_t *req)
{
    const datalogger_status_t status =
        DataLogger_get_status();

    /*
     * No se permite descargar mientras el FILE* de escritura está abierto.
     * Así evitamos acceso concurrente al mismo fichero desde SPIFFS.
     */
    if (status.recording)
    {
        httpd_resp_set_status(
            req,
            "409 Conflict");

        httpd_resp_set_type(
            req,
            "text/plain; charset=utf-8");

        return httpd_resp_sendstr(
            req,
            "Detenga la grabacion antes de descargar el fichero.");
    }

    if (!status.file_available)
    {
        httpd_resp_set_status(
            req,
            "404 Not Found");

        httpd_resp_set_type(
            req,
            "text/plain; charset=utf-8");

        return httpd_resp_sendstr(
            req,
            "No existe ningun registro de aceleracion.");
    }

    FILE *file =
        fopen(
            DataLogger_get_file_path(),
            "r");

    if (file == NULL)
    {
        httpd_resp_set_status(
            req,
            "500 Internal Server Error");

        return httpd_resp_sendstr(
            req,
            "No se pudo abrir el registro.");
    }

    httpd_resp_set_type(
        req,
        "text/csv; charset=utf-8");

    httpd_resp_set_hdr(
        req,
        "Content-Disposition",
        "attachment; filename=\"aceleracion.csv\"");

    httpd_resp_set_hdr(
        req,
        "Cache-Control",
        "no-store");

    char buffer[512];
    esp_err_t result = ESP_OK;

    for (;;)
    {
        const size_t read_bytes =
            fread(
                buffer,
                1U,
                sizeof(buffer),
                file);

        if (read_bytes > 0U)
        {
            result =
                httpd_resp_send_chunk(
                    req,
                    buffer,
                    read_bytes);

            if (result != ESP_OK)
            {
                break;
            }
        }

        if (read_bytes < sizeof(buffer))
        {
            if (feof(file))
            {
                break;
            }

            if (ferror(file))
            {
                result = ESP_FAIL;
                break;
            }
        }
    }

    fclose(file);

    if (result == ESP_OK)
    {
        result =
            httpd_resp_send_chunk(
                req,
                NULL,
                0U);
    }

    ESP_LOGI(
        TAG,
        "Descarga de aceleracion.csv finalizada: %s",
        esp_err_to_name(result));

    return result;
}

#endif

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
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL};

#if DATALOGGER_ENABLED
    const httpd_uri_t download_uri = {
        .uri = "/download_log",
        .method = HTTP_GET,
        .handler = download_log_get_handler,
        .user_ctx = NULL};
#endif

    if (httpd_register_uri_handler(server, &root_uri) != ESP_OK)
    {
        httpd_stop(server);
        return NULL;
    }

#if DATALOGGER_ENABLED
    if (httpd_register_uri_handler(server, &download_uri) != ESP_OK)
    {
        httpd_stop(server);
        return NULL;
    }
#endif

    if (websocket_register_uri(server) != ESP_OK)
    {
        httpd_stop(server);
        return NULL;
    }

    ESP_LOGI(TAG, "Servidor HTTP iniciado correctamente");
    return server;
}
