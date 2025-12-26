#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "protocol_examples_common.h"
//#include "string.h"
#include "esp_crt_bundle.h"
#include "driver/gpio.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include <sys/socket.h>

#include "esp_wifi.h"

#define BOOT_GPIO GPIO_NUM_0
#define HASH_LEN 32

//Este es un ejemplo que ejecta un código y el ESP32s3 se va a buscar un binario "Ejemplo01.bin" a 
//un servidor https que está ejecutado en 192.168.0.15 con python a través del script https_server.py
//
//Es necesario generar el certificado correcto antes de empezar.



//1-descarga openssl
//https://slproweb.com/products/Win32OpenSSL.html

//2-Actualiza el certificado con:
//openssl req -x509 -newkey rsa:2048 -keyout ca_key.pem -out ca_cert.pem -days 365 -nodes
// cuando pida CN. poner la IP de la máquina que va a tener el binario Ejemplo01.bin, por ejemplo 192.168.0.15

//3-En el programa actualizar la dirección y el nombre ..url =  "https://192.168.0.15:8070/Ejemplo01.bin", --> compilar y flashear

//Para lanzar el servidor con python desde la máquina 192.168.0.15:
//python https_server.py

//probar antes desde un navegador: https://192.168.0.15:8070

//cuando se conecte en el servidor veremos: 
//C:\Users\fjbur\OneDrive\Documentos\ESP_programs\OTA\simple_ota_example>python https_server.py
//Serving HTTPS on https://192.168.0.15:8070
//192.168.0.15 - - [25/Dec/2025 20:23:02] "GET /Ejemplo01.bin HTTP/1.1" 200 -

//Tool (no firefox): https://thelastoutpostworkshop.github.io/microcontroller_devkit/espconnect/


static const char *TAG = "simple_ota_example";
extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

#define OTA_URL_SIZE 256

int state = 0; //variable que almacena el estado del LED en GPIO4

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    case HTTP_EVENT_REDIRECT:
        ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
        break;
    }
    return ESP_OK;
}
/*-------------------------------------------------------------------------*/
void update_ota_image(void *pvParameter)
{
    ESP_LOGI(TAG, "Buscando Imagen en Servidor para actualización de OTA");
 
    esp_http_client_config_t config = 
    {
        //.url =  "https://192.168.0.15:8070/Ejemplo01.bin",  
        .url =  "https://192.168.0.15:8070/simple_ota.bin",
 
        .event_handler = _http_event_handler,                 
        .keep_alive_enable = true,        
        .cert_pem = (const char *)server_cert_pem_start,        
    };

    esp_https_ota_config_t ota_config = 
    {
        .http_config = &config,
    };

    ESP_LOGI(TAG, "Intentando descargar imagen de: %s", config.url);

    esp_err_t ret = esp_https_ota(&ota_config);

    if (ret == ESP_OK) 
    {
        ESP_LOGI(TAG, "Actualización Correcta, Reset Microcontrolador...");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Actualización de Firmware Fallida");
    }
    while (1) 
    {
        //en teoría aquí no debería llegar si la actualización ha sido correcta
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Ejecutando Tarea update_ota_image");
    }
}
// /*-------------------------------------------------------------------------*/
// static void print_sha256(const uint8_t *image_hash, const char *label)
// {
//     char hash_print[HASH_LEN * 2 + 1];
//     hash_print[HASH_LEN * 2] = 0;
//     for (int i = 0; i < HASH_LEN; ++i) 
// 	{
//         sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
//     }
//     ESP_LOGI(TAG, "%s %s", label, hash_print);
// }
// /*-------------------------------------------------------------------------*/
// static void get_sha256_of_partitions(void)
// {
//     uint8_t sha_256[HASH_LEN] = { 0 };
//     esp_partition_t partition;

//     // get sha256 digest for bootloader
//     partition.address   = ESP_BOOTLOADER_OFFSET;
//     partition.size      = ESP_PARTITION_TABLE_OFFSET;
//     partition.type      = ESP_PARTITION_TYPE_APP;
//     esp_partition_get_sha256(&partition, sha_256);
//     print_sha256(sha_256, "SHA-256 for bootloader: ");

//     // get sha256 digest for running partition
//     esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
//     print_sha256(sha_256, "SHA-256 for current firmware: ");
// }

/*-------------------------------------------------------------------------*/
void app_main(void)
{
    
    gpio_config_t io_conf = 
    {
        .pin_bit_mask  = 1ULL << BOOT_GPIO,
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    #define LED4 4
    gpio_set_direction(LED4, GPIO_MODE_OUTPUT);
    
        
    ESP_LOGI(TAG, "OTA example app_main start");
  
    //INICIALIZA LA MEMORIA  
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    //CALCULA SHA-256 de las particiones 
	//Para: Verificar integridad, Comparar versiones, Depurar OTA, Documentar cambios de imagen
    //get_sha256_of_partitions();

    //Conexión con el AP
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
 
    esp_wifi_set_ps(WIFI_PS_NONE);

    int8_t entrada     = 1;
    int8_t entrada_old = entrada;

    while(1)
    {
         entrada = gpio_get_level(BOOT_GPIO);          //LEE Entrada

        if ((entrada==0)&&(entrada_old==1))        
        {
            //Si se pulsa BOOT -> actualiza imagen ejecutando la tarea "             
            ESP_LOGI(TAG, "Coge Nuevo Firmware");
            xTaskCreate(&update_ota_image, "ota_example_task", 8192, NULL, 5, NULL);
        }
        else
        {
            //Si NO se pulsa BOOT -> Ejecuta el firmware actual de esta tarea"             
            ESP_LOGI(TAG, "Arranque Normal ->Ejecutando el firmware ");

            vTaskDelay(pdMS_TO_TICKS(50)); 

            gpio_set_level(LED4, state);
            state = !state;
        }
        entrada_old = entrada;
    }
}
