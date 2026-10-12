

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#define EX_UART_NUM     UART_NUM_0
#define BUF_SIZE        1024

static QueueHandle_t uart_event_queue;   // Cola de eventos UART

//-------TAREA de recepción-------------------------------------
static void uart_rx_task(void *arg)
{
    uart_event_t event;
    uint8_t data[128];

    while (1)
    {
        if (xQueueReceive(uart_event_queue, (void *)&event, portMAX_DELAY))
        {
            switch (event.type)
            {
                case UART_DATA:
                {
                    int len = uart_read_bytes(EX_UART_NUM, data, event.size, 
                                                                 portMAX_DELAY);
                    if (len > 0)
                    {
                        uart_write_bytes(EX_UART_NUM, "recibido: ", sizeof("recibido: "));
                        uart_write_bytes(EX_UART_NUM, (const char*)data, len);
                    }
                }
                break;

                case UART_FIFO_OVF:
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart_event_queue);
                    break;

                case UART_BUFFER_FULL:
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart_event_queue);
                    break;

                default:
                    break;
            }
        }
    }
}

//-----app_main-------------------------------------------------------
void app_main(void)
{
    // Configura UART (115200, 8, N, 1)
    uart_config_t uart_config = 
    {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Instala driver con cola de eventos
    uart_driver_install(EX_UART_NUM,
                        BUF_SIZE,       // RX buffer
                        BUF_SIZE,       // TX buffer
                        10,             // eventos en cola
                        &uart_event_queue,
                        0);

    uart_param_config(EX_UART_NUM, &uart_config);
    
    uart_set_pin(EX_UART_NUM, -1, -1, -1, -1);

    // Crea tarea de recepción
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 12, NULL);
}

