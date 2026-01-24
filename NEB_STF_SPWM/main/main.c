/*
Sí. Aquí tienes un ejemplo completo (ESP-IDF v5.x, driver “mcpwm_prelude”) 
que genera SPWM trifásica con MCPWM (3 salidas PWM independientes), pensado 
para llevar esas 3 PWM a IN1/IN2/IN3 del DRV8313 (y dejar EN1/EN2/EN3 a nivel alto o controlados por GPIO).

El driver MCPWM está documentado en el ESP-IDF (timers, operadores, comparadores y generadores).

GENERADO POR CHAT GPT

*/


#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"

#include "esp_timer.h"

esp_timer_handle_t myESP_Timer;

static const char *TAG = "spwm_3ph";

/* ===================== Configuración ===================== */
// GPIOs PWM hacia DRV8313 (típicamente a IN1/IN2/IN3)
#define PWM_U_GPIO   6
#define PWM_V_GPIO   7
#define PWM_W_GPIO   15

// PWM carrier
#define MCPWM_GROUP_ID          0
#define MCPWM_RES_HZ            10000000UL  // 10 MHz -> 0.1 us/tick
#define PWM_CARRIER_HZ          20000.0f    // 20 kHz
#define MCPWM_PERIOD_TICKS      ((uint32_t)( (float)MCPWM_RES_HZ / (1.0f * PWM_CARRIER_HZ) ))

// Fundamental
#define SPWM_FUND_HZ            40.0f       // 50 Hz (eléctrica)
#define MOD_INDEX               0.25f       // 0..1 (no apures a 1.0)

#define DUTY_MIN  0.05f
#define DUTY_MAX  0.95f


// Protección numérica
static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* ===================== Sincronización actualización ===================== */
static bool on_timer_empty_cb(mcpwm_timer_handle_t timer,
                              const mcpwm_timer_event_data_t *edata,
                              void *user_ctx)
{
    BaseType_t hp_task_woken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)user_ctx, &hp_task_woken);
    return hp_task_woken == pdTRUE;
}

/* ===================== MCPWM: genera PWM center-aligned (UP_DOWN) ===================== */
static void config_pcpwm(mcpwm_gen_handle_t gen, mcpwm_cmpr_handle_t cmp)
{
    mcpwm_generator_set_actions_on_timer_event(gen,
                                               MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,   
                                                                            MCPWM_TIMER_EVENT_EMPTY, 
                                                                            MCPWM_GEN_ACTION_LOW),
                                               MCPWM_GEN_TIMER_EVENT_ACTION_END());

    mcpwm_generator_set_actions_on_compare_event(gen,
                                                 MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,   
                                                                              cmp, 
                                                                              MCPWM_GEN_ACTION_HIGH),
                                                 MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, 
                                                                              cmp, 
                                                                              MCPWM_GEN_ACTION_LOW),
                                                 MCPWM_GEN_COMPARE_EVENT_ACTION_END());
}

#define CMP_GUARD_TICKS   1  

uint32_t duty_to_cmp_updown(float duty)
{
    uint32_t cmp = (uint32_t)
    (
        (float)MCPWM_PERIOD_TICKS *0.5* (1.0f - duty) + 0.5f
    );

    if (cmp < CMP_GUARD_TICKS)
        cmp = CMP_GUARD_TICKS;

    if (cmp > 0.5*MCPWM_PERIOD_TICKS - CMP_GUARD_TICKS)
        cmp = 0.5*MCPWM_PERIOD_TICKS - CMP_GUARD_TICKS;

    return cmp;
}

/* ===================== app_main ===================== */
void app_main(void)
{
    ESP_LOGI(TAG, "SPWM 3-phase with MCPWM (UP_DOWN), carrier=%.0f Hz, fund=%.1f Hz, period_ticks=%u",
             PWM_CARRIER_HZ, SPWM_FUND_HZ, (unsigned)MCPWM_PERIOD_TICKS);

    SemaphoreHandle_t update_sem = xSemaphoreCreateCounting(1, 0);

    /* --------------------- Timer --------------------------- */
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t tcfg = 
    {
        .group_id      = MCPWM_GROUP_ID,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_RES_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
        .period_ticks  = MCPWM_PERIOD_TICKS,
    };
    mcpwm_new_timer(&tcfg, &timer);

    /* ----- Operators (necesitamos 3 generadores: U, V, W)
       - oper0: U 
       - oper1: V 
       - oper2: W 
    */
    mcpwm_oper_handle_t oper0 = NULL, oper1 = NULL, oper2 = NULL;;
    mcpwm_operator_config_t ocfg = 
    { 
        .group_id = MCPWM_GROUP_ID 
    };
    mcpwm_new_operator(&ocfg, &oper0);
    mcpwm_new_operator(&ocfg, &oper1);
    mcpwm_new_operator(&ocfg, &oper2);

    mcpwm_operator_connect_timer(oper0, timer);
    mcpwm_operator_connect_timer(oper1, timer);
    mcpwm_operator_connect_timer(oper2, timer);

    /* ----- Comparators ----- */
    mcpwm_cmpr_handle_t cmp_u = NULL, cmp_v = NULL, cmp_w = NULL;
    mcpwm_comparator_config_t ccfg = 
    { 
         .flags.update_cmp_on_tez = true 
    };
    mcpwm_new_comparator(oper0, &ccfg, &cmp_u);
    mcpwm_new_comparator(oper1, &ccfg, &cmp_v);
    mcpwm_new_comparator(oper2, &ccfg, &cmp_w);

    /* ----- Generators ----- */
    mcpwm_gen_handle_t gen_u = NULL, gen_v = NULL, gen_w = NULL;

    mcpwm_generator_config_t gcfg_u = { .gen_gpio_num = PWM_U_GPIO };
    mcpwm_generator_config_t gcfg_v = { .gen_gpio_num = PWM_V_GPIO };
    mcpwm_generator_config_t gcfg_w = { .gen_gpio_num = PWM_W_GPIO };

    mcpwm_new_generator(oper0, &gcfg_u, &gen_u);
    mcpwm_new_generator(oper1, &gcfg_v, &gen_v);
    mcpwm_new_generator(oper2, &gcfg_w, &gen_w);

    // Configura acciones para PWM center-aligned
    config_pcpwm(gen_u, cmp_u);
    config_pcpwm(gen_v, cmp_v);
    config_pcpwm(gen_w, cmp_w);

    // Duty inicial (50%)
    mcpwm_comparator_set_compare_value(cmp_u, duty_to_cmp_updown(0.5f));
    mcpwm_comparator_set_compare_value(cmp_v, duty_to_cmp_updown(0.5f));
    mcpwm_comparator_set_compare_value(cmp_w, duty_to_cmp_updown(0.5f));

    for (int i=0;i<100;i++)
        printf("%d,%lu\n",i,duty_to_cmp_updown((float)i/100.0));

    // Callback a cada TEZ (EMPTY) para actualizar SPWM sin “jitter”
    mcpwm_timer_event_callbacks_t cbs = 
    {
        .on_empty = on_timer_empty_cb,
    };
    mcpwm_timer_register_event_callbacks(timer, &cbs, (void*)update_sem);

    // Enable + start
    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

    /* ----- Bucle SPWM ----- */
    const float w = 2.0f * (float)M_PI * SPWM_FUND_HZ; //w = 2*pi*f
    const float Ts = 1.0f / PWM_CARRIER_HZ;      // actualizamos a 20 kHz (una vez por ciclo PWM)
    float theta = 0.0f;

    int64_t t1 = esp_timer_get_time();   // Tiempo t1 en µs.    



    while (true) {
        
        xSemaphoreTake(update_sem, portMAX_DELAY);
       
        theta += w * Ts;
        if (theta >= 2.0f * (float)M_PI) 
            theta -= 2.0f * (float)M_PI;

        // Tres senos desfasados 120°
        float su = sinf(theta);
        float sv = sinf(theta - 2.0f*(float)M_PI/3.0f);
        float sw = sinf(theta - 4.0f*(float)M_PI/3.0f);

        // SPWM unipolar: duty 0..1
        float du = 0.5f * (1.0f + MOD_INDEX * su);
        float dv = 0.5f * (1.0f + MOD_INDEX * sv);
        float dw = 0.5f * (1.0f + MOD_INDEX * sw);

        mcpwm_comparator_set_compare_value(cmp_u, duty_to_cmp_updown(du));
        mcpwm_comparator_set_compare_value(cmp_v, duty_to_cmp_updown(dv));
        mcpwm_comparator_set_compare_value(cmp_w, duty_to_cmp_updown(dw));

    }
}
