#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include <math.h>
#include "esp_timer.h"
#include "esp_dsp.h"

#define N_SAMPLES 1024

__attribute__((aligned(16))) float x1[N_SAMPLES];
__attribute__((aligned(16))) float wind[N_SAMPLES];
__attribute__((aligned(16))) float y_cf[N_SAMPLES * 2];
__attribute__((aligned(16))) float mag[N_SAMPLES / 2];

void app_main()
{
    ESP_LOGI("FFT", "Inicializando tablas FFT...");
    dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);

    while (1)
    {
        // Regenerar la señal cada iteración
        dsps_tone_gen_f32(x1, N_SAMPLES, 1.0, 0.16, 0);

        // Preparar vector complejo
        for (int i = 0; i < N_SAMPLES; i++)
        {
            y_cf[2*i]   = x1[i];
            y_cf[2*i+1] = 0.0f;
        }

        // Ejecutar FFT
        int64_t t1 = esp_timer_get_time();

        dsps_fft2r_fc32(y_cf, N_SAMPLES);
        dsps_bit_rev_fc32(y_cf, N_SAMPLES);
        dsps_cplx2reC_fc32(y_cf, N_SAMPLES);

        int64_t t2 = esp_timer_get_time();

        // Magnitud en dB
        for (int i = 0; i < N_SAMPLES/2; i++)
        {
            float re = y_cf[2*i];
            float im = y_cf[2*i+1];
            float power = (re*re + im*im) / N_SAMPLES;
            mag[i] = 10.0f * log10f(power + 1e-12f);
        }

        // Mostrar espectro
        dsps_view(mag, N_SAMPLES/2, 64, 10, -60, 40, '|');

        ESP_LOGI("FFT", "FFT %d puntos: %llu us", N_SAMPLES, (t2 - t1));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // (opcional)
    // dsps_fft2r_deinit_fc32();
}
