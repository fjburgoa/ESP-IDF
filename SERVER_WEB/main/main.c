/*

Descargado del curso de youtube y modificado por mi
Carga una página web (html) y enciende los leds de colores cuando se pulsa

*/

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include "protocol_examples_common.h"
#include <esp_https_server.h>
#include "esp_tls.h"
#include <string.h>
#include "driver/gpio.h"
#include "led_strip.h"
#include <stdio.h>

#define ledR 33
#define ledG 25
#define ledB 26

int8_t led_r_state = 0;
int8_t led_g_state = 0;
int8_t led_b_state = 0;

static const char *TAG = "main";

void toggle_led(int led);

static led_strip_handle_t led_strip;

/* An HTTP GET handler */
//-------------------------------------------------------------------------------
static esp_err_t root_get_handler(httpd_req_t *req)
{
    extern unsigned char view_start[] asm("_binary_view1_html_start");
    extern unsigned char view_end[] asm("_binary_view1_html_end");
    size_t view_len = view_end - view_start;
    char viewHtml[view_len];
    memcpy(viewHtml, view_start, view_len);
    ESP_LOGI(TAG, "URI: %s", req->uri);

    if (strcmp(req->uri, "/?led-r") == 0)
        toggle_led(ledR);

    if (strcmp(req->uri, "/?led-g") == 0)
        toggle_led(ledG);

    if (strcmp(req->uri, "/?led-b") == 0)
        toggle_led(ledB);

    char *viewHtmlUpdated;
    int formattedStrResult = asprintf(&viewHtmlUpdated, viewHtml, led_r_state ? "ON" : "OFF", led_g_state ? "ON" : "OFF", led_b_state ? "ON" : "OFF");

    httpd_resp_set_type(req, "text/html");

    if (formattedStrResult > 0)
    {
        httpd_resp_send(req, viewHtmlUpdated, view_len);
        free(viewHtmlUpdated);
    }
    else
    {
        ESP_LOGE(TAG, "Error updating variables");
        httpd_resp_send(req, viewHtml, view_len);
    }

    return ESP_OK;
}
//-------------------------------------------------------------------------------
static const httpd_uri_t root = 
{
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler
};
//-------------------------------------------------------------------------------
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server");

    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();
    conf.transport_mode = HTTPD_SSL_TRANSPORT_INSECURE;
    esp_err_t ret = httpd_ssl_start(&server, &conf);
    if (ESP_OK != ret)
    {
        ESP_LOGI(TAG, "Error starting server!");
        return NULL;
    }

    // Set URI handlers
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &root);
    return server;
}
//-------------------------------------------------------------------------------
static void stop_webserver(httpd_handle_t server)
{
    httpd_ssl_stop(server);      // Stop the httpd server
}
//-------------------------------------------------------------------------------
static void disconnect_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    httpd_handle_t *server = (httpd_handle_t *)arg;
    if (*server)
    {
        stop_webserver(*server);
        *server = NULL;
    }
}
//-------------------------------------------------------------------------------
static void connect_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    httpd_handle_t *server = (httpd_handle_t *)arg;
    if (*server == NULL)
    {
        *server = start_webserver();
    }
}
 
 
//-------------------------------------------------------------------------------
void toggle_led(int led)
{
    switch (led)
    {
    case ledR:
        led_r_state = !led_r_state;
        led_g_state = false;
        led_b_state = false;
        if (led_r_state)
            led_strip_set_pixel(led_strip, 0, 255, 0, 0);
        else
            led_strip_set_pixel(led_strip, 0, 0, 0, 0);                

        led_strip_refresh(led_strip);
        break;
    case ledG:        
        led_r_state = false;
        led_g_state = !led_g_state;        
        led_b_state = false;

        if (led_g_state)
            led_strip_set_pixel(led_strip, 0, 0, 255, 0);
        else
            led_strip_set_pixel(led_strip, 0, 0, 0, 0);

        led_strip_refresh(led_strip); 
        break;
        
    case ledB:
        led_r_state = false;
        led_g_state = false;
        led_b_state = !led_b_state;

        if (led_b_state)
            led_strip_set_pixel(led_strip, 0, 0, 0, 255);
        else
            led_strip_set_pixel(led_strip, 0, 0, 0, 0);

        led_strip_refresh(led_strip);
        break;

    default:
        led_strip_clear(led_strip);
        led_r_state = 0;
        led_g_state = 0;
        led_b_state = 0;
        break;
    }
}
//-------------------------------------------------------------------------------
static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = 48,
        .max_leds = 1, // at least one LED on board
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);

    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}
 
//-------------------------------------------------------------------------------
void app_main(void)
{
    configure_led();

    static httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
    ESP_ERROR_CHECK(example_connect());
}
