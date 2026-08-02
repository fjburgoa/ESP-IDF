#include <stdio.h>
#include <sys/param.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include "esp_netif.h"
#include <esp_wifi.h>
#include <esp_https_server.h>
#include "protocol_examples_common.h"
#include "led_strip.h"
#include "driver/gpio.h"

#define ledR 1
#define ledG 2
#define ledB 3

#define LED_GPIO 48

// variables que reciben la respuesta del cliente web
int8_t led_r_state = 0;
int8_t led_g_state = 0;
int8_t led_b_state = 0;

void toggle_led(int led);

static led_strip_handle_t led_strip;

float myvar = 0.0f;

/* An HTTP GET handler */
//-------------------------------------------------------------------------------
static esp_err_t root_get_handler(httpd_req_t *req)
{
    extern unsigned char view_start[] asm("_binary_view1_html_start");
    extern unsigned char view_end[] asm("_binary_view1_html_end");
    size_t view_len = view_end - view_start;
    char viewHtml[view_len];
    memcpy(viewHtml, view_start, view_len);
    printf("URI: %s", req->uri);

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
        printf("Error updating variables\n");
        httpd_resp_send(req, viewHtml, view_len);
    }

    return ESP_OK;
}

static esp_err_t myvar_get_handler(httpd_req_t *req)
{
    char buffer[32];

    myvar = myvar + 0.1;

    snprintf(buffer, sizeof(buffer), "%.2f", myvar);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, buffer, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

//-------------------------------------------------------------------------------
static const httpd_uri_t root =
    {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler};

static const httpd_uri_t myvar_uri =
    {
        .uri = "/myvar",
        .method = HTTP_GET,
        .handler = myvar_get_handler};

//-------------------------------------------------------------------------------
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;

    // Start servidor the httpd
    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();
    conf.transport_mode = HTTPD_SSL_TRANSPORT_INSECURE;
    esp_err_t ret = httpd_ssl_start(&server, &conf);
    if (ESP_OK != ret)
    {
        printf("Error arrancando el servidor!\n");
        return NULL;
    }

    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &myvar_uri);
    return server;
}
//-------------------------------------------------------------------------------
static void stop_webserver(httpd_handle_t server)
{
    httpd_ssl_stop(server); // Stop the httpd server
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
    led_strip_clear(led_strip);
    switch (led)
    {
    case ledR:
        led_strip_set_pixel(led_strip, 0, 255, 0, 0);
        led_r_state = 1;
        led_g_state = 0;
        led_b_state = 0;
        break;
    case ledG:
        led_strip_set_pixel(led_strip, 0, 0, 255, 0);
        led_r_state = 0;
        led_g_state = 1;
        led_b_state = 0;
        break;

    case ledB:
        led_strip_set_pixel(led_strip, 0, 0, 0, 255);
        led_r_state = 0;
        led_g_state = 0;
        led_b_state = 1;
        break;

    default:
        led_strip_clear(led_strip);
        led_r_state = 0;
        led_g_state = 0;
        led_b_state = 0;
        break;
    }
    led_strip_refresh(led_strip);
}
//-------------------------------------------------------------------------------
static void configure_led(void)
{
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    led_strip_config_t strip_config = {
        .strip_gpio_num = 48,
        .led_model = LED_MODEL_WS2812,                               // Modelo de LED
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // Formato RGB
        .max_leds = 1,                                               // at least one LED on board
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
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server);
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server);
    example_connect();
}
