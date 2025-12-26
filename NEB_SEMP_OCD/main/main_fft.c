#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include <math.h>
#include "esp_timer.h"
#include "esp_dsp.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/task.h"




#define N_SAMPLES 1024*4

__attribute__((aligned(16))) float x1[N_SAMPLES];
__attribute__((aligned(16))) float wind[N_SAMPLES];
__attribute__((aligned(16))) float y_cf[N_SAMPLES * 2];
__attribute__((aligned(16))) float mag[N_SAMPLES / 2];

void app_main()
{
    ESP_LOGI("FFT", "Inicializando tablas FFT...");
    dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);

    float k = 0;

    float f = 1.0;          // frecuencia deseada
    float Fs = 100.0;      // frecuencia de muestreo
    



    while (1)        
    {

        
        float f_norm = (f+k) / Fs;  // frecuencia normalizad
        k = k+1;
        if (k>49)
            k =0;


        // Regenerar la señal cada iteración
        dsps_tone_gen_f32(x1, N_SAMPLES, 1.0, f_norm, 0);
        
        int64_t t1 = esp_timer_get_time();
        // Preparar vector complejo
        for (int i = 0; i < N_SAMPLES; i++)
        {
            y_cf[2*i]   = x1[i];
            y_cf[2*i+1] = 0.0f;
        }

        // Ejecutar FFT


        dsps_fft2r_fc32(y_cf, N_SAMPLES);
        dsps_bit_rev_fc32(y_cf, N_SAMPLES);
        dsps_cplx2reC_fc32(y_cf, N_SAMPLES);

        int64_t t2 = esp_timer_get_time();

        // Mostrar espectro
        //dsps_view(mag, N_SAMPLES/2, 64, 10, -60, 40, '|');

        int max_bin = 0;
        float max_power = 0.0f;

        // Opcional: saltamos i=0 para ignorar componente DC
        for (int i = 1; i < N_SAMPLES/2; i++) 
        {
            float re = y_cf[2*i];
            float im = y_cf[2*i + 1];
            float power = re*re + im*im;   // módulo^2

            if (power > max_power) {
                max_power = power;
                max_bin = i;
            }
        }

        // Frecuencia del armónico dominante
        float f_peak = ( (float)max_bin * Fs ) / (float)N_SAMPLES;
        ESP_LOGI("FFT", "Pico en bin %d -> %.1f Hz (power=%.0f), T = %llu ms", max_bin, f_peak, max_power,(t2-t1)/1000);




        //ESP_LOGI("FFT", "FFT %d puntos: %llu us", N_SAMPLES, (t2 - t1));

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // (opcional)
    // dsps_fft2r_deinit_fc32();
}
