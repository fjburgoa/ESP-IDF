#ifndef WEBSERVER_H
#define WEBSERVER_H
#include "esp_http_server.h"
#ifdef __cplusplus
extern "C" {
#endif
httpd_handle_t webserver_start(void);
#ifdef __cplusplus
}
#endif
#endif
