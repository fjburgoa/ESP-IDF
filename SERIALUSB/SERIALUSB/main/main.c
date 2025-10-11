/*
Este programa envía datos por el puerto serie conectado por USB sin necesidad de un convertidor USB-Serie
Simplemente conectando en USB-C del ESP32S3 izquierdo a un USB se envían los datos a un terminal serie en el PC.
*/

#include <stdint.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "sdkconfig.h"

#define PI 3.141592    

static const char *TAG = "LOG:";
static uint8_t rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
 
typedef struct mensaje
{
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];     // Data buffer
    uint8_t itf;                                        // Index of CDC device interface
    size_t buf_len;                                     // Number of bytes received
} ;

struct mensaje trx_msg = {0};

//callback cuando se abre/cierra la comunicación
void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;

    trx_msg.itf = (uint8_t)itf;

    ESP_LOGI(TAG, "Line state changed on channel %d: DTR:%d, RTS:%d", trx_msg.itf, dtr, rts);
}


void app_main(void)
{
    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = 
    {
        .device_descriptor        = NULL,
        .string_descriptor        = NULL,
        .external_phy             = false,
        .configuration_descriptor = NULL,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = 
    {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,        
        .callback_rx = NULL,  
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_0,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &tinyusb_cdc_line_state_changed_callback));

    int k=0;    

    ESP_LOGI(TAG, "USB initialization DONE");

    while (1) 
    {        
        sprintf(trx_msg.buf,"%d  x=%.3f\r\n",k,sin((PI*(float)k)/100.0f));              //copia al buffer el dato 
        tinyusb_cdcacm_write_queue(trx_msg.itf, trx_msg.buf, sizeof(trx_msg.buf));      //envía a la cola de transmisión
        esp_err_t err = tinyusb_cdcacm_write_flush(trx_msg.itf, 0);                     //flushea

        vTaskDelay(10 / portTICK_PERIOD_MS);
        k++;
    }
}


