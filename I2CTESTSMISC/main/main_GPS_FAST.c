#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* -------------------------------------------------------------------------- */
/* UART GPS                                                                   */
/* -------------------------------------------------------------------------- */

#define GPS_UART_PORT UART_NUM_1

#define GPS_UART_TX_GPIO 5
#define GPS_UART_RX_GPIO 7

#define RX_BUFFER_SIZE 2048
#define READ_BUFFER_SIZE 256

#define TEST_TIME_MS 5000

static const char *TAG = "GPS_SCAN";

/* -------------------------------------------------------------------------- */

static esp_err_t gps_uart_init(uint32_t baud_rate)
{
    /*
     * Eliminamos cualquier instalación anterior de UART1 antes de cambiar
     * la velocidad.
     */
    uart_driver_delete(GPS_UART_PORT);

    const uart_config_t config = {
        .baud_rate = (int)baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT, &config));

    ESP_ERROR_CHECK(
        uart_set_pin(
            GPS_UART_PORT,
            GPS_UART_TX_GPIO,
            GPS_UART_RX_GPIO,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE));

    return uart_driver_install(
        GPS_UART_PORT,
        RX_BUFFER_SIZE,
        0,
        0,
        NULL,
        0);
}

/* -------------------------------------------------------------------------- */

static void print_received_data(const uint8_t *data, size_t length)
{
    printf("RX (%u bytes): ", (unsigned)length);

    for (size_t i = 0; i < length; i++)
    {
        const uint8_t c = data[i];

        /*
         * Mostramos directamente caracteres ASCII.
         *
         * Si el receptor emite NMEA deberíamos ver:
         *
         *      $GNGGA,...
         *      $GNRMC,...
         *      $GNGSA,...
         */
        if ((c >= 32) && (c <= 126))
        {
            putchar((char)c);
        }
        else if (c == '\r')
        {
            printf("<CR>");
        }
        else if (c == '\n')
        {
            printf("<LF>\n");
        }
        else
        {
            /*
             * Los mensajes UBX son binarios y aparecerán de esta forma.
             *
             * Un mensaje UBX comienza normalmente por:
             *
             *      B5 62
             */
            printf("<%02X>", c);
        }
    }

    printf("\n");
}

/* -------------------------------------------------------------------------- */

static bool test_baud_rate(uint32_t baud_rate)
{
    uint8_t buffer[READ_BUFFER_SIZE];

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "Probando %lu baudios", (unsigned long)baud_rate);
    ESP_LOGI(TAG, "======================================");

    ESP_ERROR_CHECK(gps_uart_init(baud_rate));

    uart_flush_input(GPS_UART_PORT);

    const TickType_t start = xTaskGetTickCount();
    const TickType_t test_time = pdMS_TO_TICKS(TEST_TIME_MS);

    size_t total_received = 0;

    while ((xTaskGetTickCount() - start) < test_time)
    {
        const int length = uart_read_bytes(
            GPS_UART_PORT,
            buffer,
            sizeof(buffer),
            pdMS_TO_TICKS(200));

        if (length > 0)
        {
            total_received += (size_t)length;
            print_received_data(buffer, (size_t)length);
        }
    }

    ESP_LOGI(
        TAG,
        "%lu baudios -> %u bytes recibidos",
        (unsigned long)baud_rate,
        (unsigned)total_received);

    return total_received > 0;
}

/* -------------------------------------------------------------------------- */

void app_main(void)
{
    /*
     * Velocidades más habituales en receptores GNSS.
     */
    const uint32_t baud_rates[] = {
        4800,
        9600,
        19200,
        38400,
        57600,
        115200,
        230400};

    ESP_LOGI(TAG, "Scanner UART para GPS NEO-M10");
    ESP_LOGI(TAG, "RX ESP32-S3 = GPIO%d", GPS_UART_RX_GPIO);

    for (;;)
    {
        for (size_t i = 0; i < sizeof(baud_rates) / sizeof(baud_rates[0]); i++)
        {
            const bool data_received = test_baud_rate(baud_rates[i]);

            if (data_received)
            {
                ESP_LOGW(
                    TAG,
                    "Hay actividad a %lu baudios",
                    (unsigned long)baud_rates[i]);
            }

            vTaskDelay(pdMS_TO_TICKS(500));
        }

        ESP_LOGI(TAG, "Escaneo completo. Repitiendo...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}