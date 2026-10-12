
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
 
#define EX_UART_NUM UART_NUM_0 //Usa UART 0
#define BUF_SIZE 1024          //Buffer circular en el driver

void app_main(void)
{
    //Propiedades de la conexión por RS-232 a través de la UART
    //115200,8,N,1
    uart_config_t uart_config = 
    {
                .baud_rate =  115200,               
                .data_bits =  UART_DATA_8_BITS,
                .parity    =  UART_PARITY_DISABLE,
                .stop_bits =  UART_STOP_BITS_1,
                .flow_ctrl =  UART_HW_FLOWCTRL_DISABLE,
                .source_clk = UART_SCLK_DEFAULT,
    };
    
    //Instala UART driver
    uart_driver_install(UART_NUM_0,    //UART 0
                          BUF_SIZE,    //Tamaño buffer entrada
                                 0,    //Tamaño buffer salida (0->bloqueante)
                                 0,     
                              NULL,   
                                 0);    
    uart_param_config(UART_NUM_0, &uart_config);
    
    //Define GPIOx UART. (-1 = sin cambio)
    uart_set_pin(UART_NUM_0, -1, -1, -1, -1);

    uint8_t data[128];  //Buffer usuario para ECO
    int length = 0;     //Tamaño datos leídos

    while(1)
    {
       //Cada 10 ms
       vTaskDelay(pdMS_TO_TICKS (10));

       //Comprueba si hay datos en el buffer RX de la UART.
       uart_get_buffered_data_len(UART_NUM_0, (size_t*)&length);

       if (length>0)        
       {
          length = uart_read_bytes(UART_NUM_0, data, length, 100); 

          /*Aquí acumularíamos el mensaje en otro buffer de usuario y se procesaría,             
            O también se puede procesar directamente*/
 
          uart_write_bytes(UART_NUM_0,"recibido: ", sizeof("recibido: ")); //Start ECO
          
          uart_write_bytes(UART_NUM_0, data, length);   //Reenvía recepción + longitud
       }

       /*Aquí se podría enviar un mensaje a la cola TX de la UART*/           

    }
}

