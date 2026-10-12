#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

//SAR ADC1
#define ADC1_CHANN          ADC_CHANNEL_5 //<-> GPIO6
#define ADC_ATTEN           ADC_ATTEN_DB_12

static int adc_raw;

void app_main(void)
{
    //-------------Incia SAR ADC1  ---------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    //-------------Configura SAR ADC1 ------//
    adc_oneshot_chan_cfg_t config = {
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_oneshot_config_channel(adc1_handle, ADC1_CHANN, &config);
  
    while (1)    
    {
        //lectura
        adc_oneshot_read(adc1_handle, ADC1_CHANN, &adc_raw);

        //conversión a voltaje (12 bits de resolución)
        float voltaje = adc_raw * (3.3 / 4095);

        //muestra por terminal
        printf("ADC: %.1f V.\n", voltaje); 

        //200 ms de bloqueo
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
