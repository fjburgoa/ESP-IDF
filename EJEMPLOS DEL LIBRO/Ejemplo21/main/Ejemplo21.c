
#include <stdint.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "sdkconfig.h"

#define PI 3.141592    
static uint8_t rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
 
typedef struct mensaje
{
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];  // Buffer de datos
    uint8_t itf;                                     // Índice de dispositivo CDC 
    size_t  buf_len;                                 // Número de bytes recibidos
} ;

struct mensaje tx_msg = {0};
int k=0;    

//----------------------app_main----------------------------------------------
void app_main(void)
{
    printf("Inicializa TinyUSB\n");
    const tinyusb_config_t tusb_cfg = 
    {
        .device_descriptor        = NULL,
        .string_descriptor        = NULL,
        .external_phy             = false,
        .configuration_descriptor = NULL,
    };
    tinyusb_driver_install(&tusb_cfg);     // Instala driver TinyUSB

    tinyusb_config_cdcacm_t acm_cfg =      // Registra clase CDC como ACM
    {
        .usb_dev                      = TINYUSB_USBDEV_0,
        .cdc_port                     = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz             = 64,        
        .callback_rx                  = NULL,  
        .callback_rx_wanted_char      = NULL,
        .callback_line_state_changed  = NULL,
        .callback_line_coding_changed = NULL
    };
    tusb_cdc_acm_init(&acm_cfg);            // Inicializa ACM 
    
    // Asigna Callback (si fuera necesario)
    tinyusb_cdcacm_register_callback( TINYUSB_CDC_ACM_0,    
                              CDC_EVENT_LINE_STATE_CHANGED,
                              NULL);

    printf("Inicialización del USB completada!\n");

    while (1) 
    {        
        //Prepara el dato a transmitir. Copia en tx_msg 
        sprintf(tx_msg.buf,"%d x=%.3f\r\n", 
                                        k++,
                                        sin((PI*(float)k)/100.0f));              

        //copia el dato a la cola de transmisión
        tinyusb_cdcacm_write_queue(tx_msg.itf, 
                                   tx_msg.buf,
                           sizeof(tx_msg.buf));   

        //transmite
        esp_err_t err = tinyusb_cdcacm_write_flush(tx_msg.itf,0);   

        //pequeño retraso de 10ms
        vTaskDelay(pdMS_TO_TICKS(10));
     }
}


