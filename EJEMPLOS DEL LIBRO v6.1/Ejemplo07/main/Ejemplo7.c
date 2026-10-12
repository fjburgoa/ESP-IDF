
#include <stdio.h>
#include "esp_system.h"
#include "soc/clk_tree_defs.h"
#include "esp_clk_tree.h"
#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"

#define LED4 4

volatile int Flag_timer = 0;  //marcador de interrupción para su gestión en main_app

//---------------------------------------------------------------------------------
static uint32_t get_freq_cpu_hz(void)
{
    uint32_t freq = 0;
    esp_clk_tree_src_get_freq_hz(SOC_CPU_CLK_SRC_PLL, 
                          ESP_CLK_TREE_SRC_FREQ_PRECISION_EXACT, 
                          &freq);
    return freq;
}
//--------------------------------------------------------------------------------
static uint32_t get_freq_apb_hz(void)
{
    uint32_t freq = 0;
    esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_APB, 
                          ESP_CLK_TREE_SRC_FREQ_PRECISION_EXACT,
                          &freq);
    return freq;
}
//--------------------------------------------------------------------------------
static uint32_t get_freq_xtal_hz(void)
{
    uint32_t freq = 0;
    esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_XTAL,    
                                 ESP_CLK_TREE_SRC_FREQ_PRECISION_EXACT, 
                                 &freq);
    return freq;
}


//---------------------------------------------------------------------------------
static uint32_t timer_base_hz(bool use_xtal, uint32_t f_xtal, uint32_t f_apb)
{
    return use_xtal ? f_xtal : f_apb;
}
//---------------------------------------------------------------------------------
static uint32_t timer_freq_hz(uint32_t divider, bool use_xtal, 
 uint32_t freq_xtal, uint32_t freq_apb)
{
    if (divider == 0)  return 0;
    return timer_base_hz(use_xtal, freq_xtal, freq_apb) / divider;
}

//------ Función para imprimir la configuración de los GPT  ------------------------
void print_config_timers(void)
{
    uint32_t freq_cpu  = get_freq_cpu_hz();
    uint32_t freq_apb  = get_freq_apb_hz();
    uint32_t freq_xtal = get_freq_xtal_hz();

    printf("CPU  Freq = %lu MHz\n", freq_cpu  / 1000000);
    printf("APB  Freq = %lu MHz\n", freq_apb  / 1000000);
    printf("XTAL Freq = %lu MHz\n", freq_xtal / 1000000);

    // ----- Timer Group 0 -----
    uint32_t t0cfg_g0 = REG_READ(TIMG_T0CONFIG_REG(0));
    uint32_t t1cfg_g0 = REG_READ(TIMG_T1CONFIG_REG(0));

    uint32_t div_t0_g0 = REG_GET_FIELD(TIMG_T0CONFIG_REG(0), TIMG_T0_DIVIDER);
    uint32_t div_t1_g0 = REG_GET_FIELD(TIMG_T1CONFIG_REG(0), TIMG_T1_DIVIDER);

    bool use_xtal_t0_g0 = REG_GET_FIELD(TIMG_T0CONFIG_REG(0), TIMG_T0_USE_XTAL);
    bool use_xtal_t1_g0 = REG_GET_FIELD(TIMG_T1CONFIG_REG(0), TIMG_T1_USE_XTAL);

    // ----- Timer Group 1 -----
    uint32_t t0cfg_g1 = REG_READ(TIMG_T0CONFIG_REG(1));
    uint32_t t1cfg_g1 = REG_READ(TIMG_T1CONFIG_REG(1));

    uint32_t div_t0_g1 = REG_GET_FIELD(TIMG_T0CONFIG_REG(1), TIMG_T0_DIVIDER);
    uint32_t div_t1_g1 = REG_GET_FIELD(TIMG_T1CONFIG_REG(1), TIMG_T1_DIVIDER);

    bool use_xtal_t0_g1 = REG_GET_FIELD(TIMG_T0CONFIG_REG(1), TIMG_T0_USE_XTAL);
    bool use_xtal_t1_g1 = REG_GET_FIELD(TIMG_T1CONFIG_REG(1), TIMG_T1_USE_XTAL);

    // ----- LOG de salida -----
    printf("TIMG0.T0: CFG=0x%08lX DIV=%lu USE_XTAL=%d f_timer=%lu Hz\n",
             t0cfg_g0, div_t0_g0, use_xtal_t0_g0,
             timer_freq_hz(div_t0_g0, use_xtal_t0_g0, freq_xtal, freq_apb));

    printf("TIMG0.T1: CFG=0x%08lX  DIV=%lu  USE_XTAL=%d f_timer=%lu Hz\n",
             t1cfg_g0, div_t1_g0,
             use_xtal_t1_g0,
             timer_freq_hz(div_t1_g0, use_xtal_t1_g0, freq_xtal, freq_apb));

    printf("TIMG1.T0: CFG=0x%08lX DIV=%lu USE_XTAL=%d f_timer=%lu Hz\n",
             t0cfg_g1, div_t0_g1,
             use_xtal_t0_g1,
             timer_freq_hz(div_t0_g1, use_xtal_t0_g1, freq_xtal, freq_apb));

    printf("TIMG1.T1: CFG=0x%08lX DIV=%lu USE_XTAL=%d f_timer=%lu Hz\n",
             t1cfg_g1, div_t1_g1,
             use_xtal_t1_g1,
             timer_freq_hz(div_t1_g1, use_xtal_t1_g1, freq_xtal, freq_apb));
}
//----------------------------------------------------------------------
//----------------- ISR del timer --------------------------------------
//----------------------------------------------------------------------
bool IRAM_ATTR GPT_TIMER_ISR(gptimer_handle_t timer, 
                             const gptimer_alarm_event_data_t *event, 
                             void *user_data) 
{
    Flag_timer = 1;  // MARCA EL EVENTO!!  
    return true;     // Return true para que el temporizador vuelva a reiniciar   
}

// -----------------app_main--------------------------------------------
void app_main(void) 
{
    // Configurar GPIO4
    gpio_set_direction(LED4, GPIO_MODE_INPUT_OUTPUT);

    // Handler 
    gptimer_handle_t gptimer = NULL;
    
    // Configurar el temporizador
    gptimer_config_t timer_config = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,  // Clk por defecto (APB 80 MHz)
        .direction     = GPTIMER_COUNT_UP,         // Contar hacia arriba
        .resolution_hz = 1000*1000,                // 1 MHz (1us por tick)
    };

    gptimer_new_timer(&timer_config, &gptimer);

    // Configurar la alarma (en µs)
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,                   // Reiniciar el contador al valor inicial
        .alarm_count  = 250000,              // 0,5 segundos = 500000 µs
        .flags.auto_reload_on_alarm = true,  // Auto-reload para repetir la alarma
    };

    gptimer_set_alarm_action(gptimer, &alarm_config);

    // Registrar el callback para la interrupción
    gptimer_event_callbacks_t cbs = {
        .on_alarm = GPT_TIMER_ISR,           // Configurar el callback
    };
    gptimer_register_event_callbacks(gptimer, &cbs, NULL);

    // Habilita e inicia el temporizador
    gptimer_enable(gptimer);
    gptimer_start(gptimer);

    printf("GPTimer started!\n");

    // Muestra la configuración de los 4 GPT Timers.
    print_config_timers();

    while (1) 
    {
        // GESTIONA EL EVENTO temporal dentro de while(1)
        if (Flag_timer)
        {
            Flag_timer = 0;                              // Borra el flag

            printf("Timer\n");
            gpio_set_level(LED4, !gpio_get_level(LED4)); // conmuta GPIO4 
        }
    }
}

