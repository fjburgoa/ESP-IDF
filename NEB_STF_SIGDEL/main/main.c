#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/sdm.h"


#define PI                      (3.1416f)       // Constante PI
#define SIGDEL_GPIO             (5)             // PGIO6
#define OVER_SAMPLE_RATE        (10000000)      // 10 MHz (parám. modulador SigDel) 
#define dT                      (10)            // ms (temporización)
#define SINE_WAVE_FREQ_HZ       (1)   	        // Frecuencia señal, Periodo = 1/f
#define AMPL                    (127.0f)        // 1 ~ 127, amplitud de la onda
#define SINE_WAVE_POINTS        1000*(1/SINE_WAVE_FREQ_HZ)/dT      


static int8_t sine_wave[SINE_WAVE_POINTS];      // buffer para la señal senoidal

void app_main(void)
{
    //genera onda de prueba
    for (int i = 0; i < SINE_WAVE_POINTS; i++)     
    {
        float w = (2*PI*SINE_WAVE_FREQ_HZ);
        float x = 100*w*(float)i/SINE_WAVE_POINTS;
        sine_wave[i] = (int8_t) (127.0f*(sin(x))) ;
    }
    //handler canal sigma-delta + configuración
    sdm_channel_handle_t sdm_chan = NULL;
    sdm_config_t config = {
        .clk_src 		= SDM_CLK_SRC_DEFAULT,
        .gpio_num 	   	= SIGDEL_GPIO,
        .sample_rate_hz	= 10000000,
    };
    //crea canal y habilitalo
    sdm_new_channel(&config, &sdm_chan);
    sdm_channel_enable(sdm_chan);

    printf("Sigma-delta Activo en GPIO %d\n", SIGDEL_GPIO);
 
    int cnt = 0;
    while(1)
    {    
        sdm_channel_set_pulse_density(sdm_chan, sine_wave[cnt++]);
        if (cnt>= SINE_WAVE_POINTS)
            cnt  =0;
        vTaskDelay(pdMS_TO_TICKS(dT));
        //vTaskDelay(10/portTICK_PERIOD_MS);
        
    }
}

