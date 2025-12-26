
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_continuous.h"

volatile int flag_adc = 0;

#define BLOCK_SIZE 400         //con un tamaño de 400 se consigue 0,1 seg.
#define FREQUENCY  1000        //frecuencia del ADC = 1000 Hz
#define NUM_ANALOG 1

//------------------------ISR-----------------------------------------------
static bool IRAM_ATTR ISR_ADC_DMA (adc_continuous_handle_t handle, 
                                   const adc_continuous_evt_data_t *edata, 
                                   void *user_data)
{
    flag_adc = 1;
    return true;
};
//----------------------app_main---------------------------------------------
void app_main(void)
{
    esp_err_t ret;

    adc_continuous_handle_t handle = NULL;     

    adc_continuous_handle_cfg_t adc_config =     
    {
        .max_store_buf_size = BLOCK_SIZE,     //Longitud máx de los resultados que
                                              //se puede almacenar en el driver
        .conv_frame_size    = BLOCK_SIZE,     //Tamaño del frame. Múltiplos de 
                                              //SOC_ADC_DIGI_DATA_BYTES_PER_CONV
                                              //dado que BLOCK_SIZE = 32, se puede  
                                              //almacenar BLOCK_SIZE/4 datos    
    };

    adc_continuous_new_handle(&adc_config, &handle);    //registra el buffer    

    adc_continuous_config_t dig_cfg =                  
    {
        .sample_freq_hz = FREQUENCY,                    //frecuencia en Hertz   
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1 ,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,   
        .pattern_num    = NUM_ANALOG,
    };
 
    adc_digi_pattern_config_t adc_pattern[NUM_ANALOG] = {0};    

    adc_pattern[0].atten     = ADC_ATTEN_DB_12;
    adc_pattern[0].channel   = ADC_CHANNEL_5;           // GPIO6 → ADC_CHANNEL_5
    adc_pattern[0].unit      = ADC_UNIT_1;
    adc_pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    dig_cfg.adc_pattern = adc_pattern;                          

    adc_continuous_config(handle, &dig_cfg);            //registra el ADC al GPIO    

    adc_continuous_evt_cbs_t cbs =            
    {
        .on_conv_done = ISR_ADC_DMA,                    //ISR    
    };
    adc_continuous_register_event_callbacks(handle, &cbs, NULL); //registra ISR    
    
    adc_continuous_start(handle);
    
    uint8_t result[BLOCK_SIZE] = {0}; //Buffer para leer el frame completo
    uint32_t ret_num           = 0;   //indica los datos del frame

    while (1) 
    {
         if (flag_adc)
         {
            flag_adc = 0;             //borra el flag y lee
            ret = adc_continuous_read(handle, result, BLOCK_SIZE, &ret_num, 0); 

            uint32_t sum = 0;
            int count    = 0;

            if (ret == ESP_OK) 
            {
  		        //Suma de los datos almacenados en buffer y saca el valor medio
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) 
                {
                   adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                   sum += p->type2.data;  //acumula el valor de cada muestra
                   count++;
                }
                //saca el valor medio
		        printf("Valor medio buffer: %lu\n", sum/count); 
                
            } else if (ret == ESP_ERR_TIMEOUT) {
                //Timeout de lectura
            } else {
                //Error ADC
            }
         }   
    } //fin while

    adc_continuous_stop(handle);

    adc_continuous_deinit(handle);
}

