#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#define EX_UART_NUM     UART_NUM_0
#define BUF_SIZE        1024
#define PATTERN_CHAR    '\n'      // Delimitador de trama

static QueueHandle_t uart_event_queue;

//------------------- Tarea de recepción -----------------------------
static void uart_rx_task(void *arg)
{
    uart_event_t event;
    uint8_t data[BUF_SIZE];

    while (1)
    {
        if (xQueueReceive(uart_event_queue, &event, portMAX_DELAY))
        {
            switch (event.type)
            {
                case UART_PATTERN_DET:
                {
                    // Posición del delimitador '\n' dentro del buffer RX
                    int pos = uart_pattern_pop_pos(EX_UART_NUM);

                    if (pos != -1)
                    {
                        // Leer trama completa (sin consumir el '\n')
                        int len = uart_read_bytes(EX_UART_NUM,
                                                  data,
                                                  pos + 1,
                                                  portMAX_DELAY);

                        if (len > 0)
                        {
                            data[len] = '\0';   // Terminar como string
                            // ---- PROCESAR TRAMA COMPLETA ----
                            //   data = "comando recibido\n"
                        }
                    }
                    else
                    {
                        // No se encontró el patrón: limpiar y continuar
                        uart_flush_input(EX_UART_NUM);
                    }
                }
                break;

                case UART_DATA:
                    // Bytes recibidos sin patrón (opcional)
                    // Se podrían leer aquí si interesa
                    break;

                default:
                    break;
            }
        }
    }
}

//---------------------------- app_main -------------------------------
void app_main(void)
{
    uart_config_t uart_config = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(EX_UART_NUM,
                        BUF_SIZE,
                        BUF_SIZE,
                        20,
                        &uart_event_queue,
                        0);

    uart_param_config(EX_UART_NUM, &uart_config);
    uart_set_pin(EX_UART_NUM, -1, -1, -1, -1);

    // Activar detección del carácter delimitador '\n'
    uart_enable_pattern_det_baud_intr(EX_UART_NUM,
                                      PATTERN_CHAR,
                                      1,   // número de repeticiones
                                      9,   // timeout del patrón
                                      0,
                                      0);

    // Guardar espacio para posiciones del patrón
    uart_pattern_queue_reset(EX_UART_NUM, 20);

    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 12, NULL);
}
