#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <time.h>

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

/* -------------------------------------------------------------------------- */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    const size_t html_length = (size_t)(index_html_end - index_html_start);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)index_html_start, html_length);
}

/* -------------------------------------------------------------------------- */
#if DATALOGGER_ENABLED
static esp_err_t download_log_get_handler(httpd_req_t *req)
{
    const datalogger_status_t status = DataLogger_get_status();

    if (status.recording)
    {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        return httpd_resp_sendstr(req, "Detenga la grabación antes de descargar.");
    }

    if (!status.data_available)
    {
        httpd_resp_set_status(req, "404 Not Found");
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        return httpd_resp_sendstr(req, "No hay muestras registradas.");
    }

    httpd_resp_set_type(req, "text/csv; charset=utf-8");
    httpd_resp_set_hdr(
        req,
        "Content-Disposition",
        "attachment; filename=\"efis_datalogger.csv\"");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    static const char header[] =
        "date_utc,time_utc,latitude_deg,longitude_deg,"
        "accel_x_ms2,accel_y_ms2,accel_z_ms2,"
        "linear_x_ms2,linear_y_ms2,linear_z_ms2,"
        "gravity_x_ms2,gravity_y_ms2,gravity_z_ms2,"
        "gyro_x_dps,gyro_y_dps,gyro_z_dps,"
        "pitch_deg,roll_deg,slip_ball_deg,turn_rate_dps\n";

    esp_err_t result = httpd_resp_send_chunk(req, header, sizeof(header) - 1U);
    char line[384];

    for (uint32_t index = 0U;
         (index < status.samples) && (result == ESP_OK);
         ++index)
    {
        datalogger_sample_t sample;
        result = DataLogger_get_sample(index, &sample);

        if (result != ESP_OK)
        {
            break;
        }

        const time_t utc_seconds = (time_t)(sample.utc_time_ms / 1000);
        const unsigned milliseconds =
            (unsigned)(sample.utc_time_ms % 1000);
        struct tm utc = {0};
        gmtime_r(&utc_seconds, &utc);

        const int length = snprintf(
            line,
            sizeof(line),
            "%04d-%02d-%02d,%02d:%02d:%02d.%03u,"
            "%.8f,%.8f,"
            "%.5f,%.5f,%.5f,"
            "%.5f,%.5f,%.5f,"
            "%.5f,%.5f,%.5f,"
            "%.5f,%.5f,%.5f,"
            "%.3f,%.3f,%.3f,%.3f\n",
            utc.tm_year + 1900,
            utc.tm_mon + 1,
            utc.tm_mday,
            utc.tm_hour,
            utc.tm_min,
            utc.tm_sec,
            milliseconds,
            sample.latitude_deg,
            sample.longitude_deg,
            (double)sample.acceleration_x_ms2,
            (double)sample.acceleration_y_ms2,
            (double)sample.acceleration_z_ms2,
            (double)sample.linear_acceleration_x_ms2,
            (double)sample.linear_acceleration_y_ms2,
            (double)sample.linear_acceleration_z_ms2,
            (double)sample.gravity_x_ms2,
            (double)sample.gravity_y_ms2,
            (double)sample.gravity_z_ms2,
            (double)sample.gyro_x_dps,
            (double)sample.gyro_y_dps,
            (double)sample.gyro_z_dps,
            (double)sample.pitch_deg,
            (double)sample.roll_deg,
            (double)sample.slip_ball_deg,
            (double)sample.turn_rate_dps);

        if ((length < 0) || ((size_t)length >= sizeof(line)))
        {
            result = ESP_FAIL;
            break;
        }

        result = httpd_resp_send_chunk(req, line, (size_t)length);
    }

    if (result == ESP_OK)
    {
        result = httpd_resp_send_chunk(req, NULL, 0U);
    }

    ESP_LOGI(TAG,
             "Descarga CSV finalizada: %u muestras, circular=%s, resultado=%s",
             (unsigned)status.samples,
             status.wrapped ? "sí" : "no",
             esp_err_to_name(result));
    return result;
}
#endif

/* -------------------------------------------------------------------------- */
httpd_handle_t webserver_start(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WEBSERVER_PORT;
    config.max_open_sockets = WEBSERVER_MAX_OPEN_SOCKETS;
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Iniciando servidor HTTP en puerto %u", config.server_port);

    const esp_err_t err = httpd_start(&server, &config);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo iniciar HTTP: %s", esp_err_to_name(err));
        return NULL;
    }

    const httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL,
    };

#if DATALOGGER_ENABLED
    const httpd_uri_t download_uri = {
        .uri = "/download_log",
        .method = HTTP_GET,
        .handler = download_log_get_handler,
        .user_ctx = NULL,
    };
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
