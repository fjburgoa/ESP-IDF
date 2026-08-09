#ifndef WEBSOCKET_H
#define WEBSOCKET_H
#include "esp_err.h"
#include "esp_http_server.h"
#ifdef __cplusplus
extern "C"
{
#endif
    esp_err_t websocket_register_uri(httpd_handle_t server);
    esp_err_t websocket_start_dummy_stream(httpd_handle_t server);
    float websocket_get_qnh(void);
#ifdef __cplusplus
}
#endif
#endif
