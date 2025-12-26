#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include <math.h>
#include "esp_timer.h"
#include "esp_dsp.h"

static const char *TAG = "main";

// Ejemplo para el uso de FFT en la libería ESP-DSP

#define N_SAMPLES 1024
int N = N_SAMPLES;

__attribute__((aligned(16)))
float x1[N_SAMPLES];                // Input test array

__attribute__((aligned(16)))
float wind[N_SAMPLES];              // Window coefficients

__attribute__((aligned(16)))
float y_cf[N_SAMPLES * 2];          // working complex array

float *y1_cf = &y_cf[0];            // Pointers to result arrays



void app_main()
{
    
    ESP_LOGI(TAG, "Start Example.");

    //dsps_wind_hann_f32(wind, N);                     // Generate hann window

    esp_err_t ret;
    ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret  != ESP_OK) 
    {
    ESP_LOGE(TAG, "Not possible to initialize FFT. Error = %i", ret);
    }      
    

    dsps_tone_gen_f32(x1, N, 1.0, 0.16,  0);         // Generate input signal for x1 A=1 , F=0.1

    while(1)
    {
    
        int64_t t1 = esp_timer_get_time();   // Tiempo t1 en microsegundos 
  
        int64_t t2 = esp_timer_get_time();   // Tiempo t2 en microsegundos                                  

        for (int i = 0 ; i < N ; i++) 
            y_cf[i * 2 + 0] = x1[i];// * wind[i];           // Convert input vectors to one complex vector

        dsps_fft2r_fc32(y_cf, N);                         //complex FFT of radix 2 

        dsps_bit_rev_fc32(y_cf, N);                       // Bit reverse
        
        dsps_cplx2reC_fc32(y_cf, N);                      // Convert one complex vector to two complex vectors 

        for (int i = 0 ; i < N / 2 ; i++)
            y1_cf[i] = 10 * log10f((y1_cf[i * 2 + 0] * y1_cf[i * 2 + 0] + y1_cf[i * 2 + 1] * y1_cf[i * 2 + 1]) / N);

        dsps_view(y1_cf, N / 2, 64, 10,  -60, 40, '|');   // Show power spectrum in 64x10 window from -100 to 0 dB from 0..N/4 samples
    
        ESP_LOGI(TAG, "FFT de %i puntos complejos lleva: %llu us", t2-t1);

     	vTaskDelay(pdMS_TO_TICKS(1000));                  //Delay 1 segundo 

        memset(y_cf,0,N_SAMPLES*2);
    }

}
