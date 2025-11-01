/* Configuración y lectura del ADC_CHANNEL_6 (GPIO7)  *
 * por medio de lectura ciclica con DMA               *
 * Javier Burgoa, 12/10/25                            */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_continuous.h"

volatile int flag_adc = 0;

//----------------------------------------------------------------------------------------------------------------------
//10 - 
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    flag_adc = 1;
    return true;
};


#define BLOCK_SIZE 400
#define FREQUENCY  1000
#define NUM_ANALOG 1

//----------------------------------------------------------------------------------------------------------------------
void app_main(void)
{
    esp_err_t ret;

    //1 -     
    adc_continuous_handle_t handle = NULL;

    //2 - 
    adc_continuous_handle_cfg_t adc_config = 
    {
        .max_store_buf_size = BLOCK_SIZE,     // Longitud máxima de los resultados que se pueden almacenar en el driver        
        .conv_frame_size    = BLOCK_SIZE,     // Tamaño del frame. Múltiplos de SOC_ADC_DIGI_DATA_BYTES_PER_CONV
                                              // dado que BLOCK_SIZE = 32, podremos almacenar BLOCK_SIZE/4 datos    
    };

    //3 - 
    adc_continuous_new_handle(&adc_config, &handle);

    //4 - 
    adc_continuous_config_t dig_cfg = 
    {
        .sample_freq_hz = FREQUENCY,                    //frecuencia en Hertz   
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1 ,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2, //Para el ESP32S3 
        .pattern_num    = NUM_ANALOG,
    };
 
    //5 - 
    adc_digi_pattern_config_t adc_pattern[NUM_ANALOG] = {0}; 

    adc_pattern[0].atten     = ADC_ATTEN_DB_12;
    adc_pattern[0].channel   = ADC_CHANNEL_6;           // GPIO7 → ADC_CHANNEL_6
    adc_pattern[0].unit      = ADC_UNIT_1;
    adc_pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    //6 - 
    dig_cfg.adc_pattern = adc_pattern;

    //7 - 
    adc_continuous_config(handle, &dig_cfg);

    //8 - 
    adc_continuous_evt_cbs_t cbs = 
    {
        .on_conv_done = s_conv_done_cb,
    };
    adc_continuous_register_event_callbacks(handle, &cbs, NULL);
    
    
    //9 -
    adc_continuous_start(handle);

    uint8_t result[BLOCK_SIZE] = {0}; //Lee el Frame completo
    uint32_t ret_num           = 0;   //indica los datos del frame

    while (1) 
    {
         if (flag_adc)
         {
            //11 - 
            flag_adc = 0;            
            ret = adc_continuous_read(handle, result, BLOCK_SIZE, &ret_num, 0);

            uint32_t sum = 0;
            int count = 0;

            if (ret == ESP_OK) 
            {
                printf("%lu Datos:\n",ret_num);
                
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                    sum += p->type2.data;  // acumula el valor de cada muestra
                    count++;
                }
                
                float average = (float)sum / count;
                printf("Valor medio: %.2f - %lu\n", average, xTaskGetTickCount()*portTICK_PERIOD_MS);

            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("Timeout de lectura\n");
            } else {
                printf("Error ADC: %s\n", esp_err_to_name(ret));
            }
         }   
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}