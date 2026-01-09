#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

/*
MCPWM en ESP-IDF sigue esta jerarquía:
   Timer → Operator → pulse_width_us → Generator → GPIO
  
   Timer: Cuenta de 0 a 20000 (uP), Reinicia cada 20 ms. Es el “reloj maestro” del PWM

    ✔ PWM de servo 100 % hardware
    ✔ Precisión en microsegundos
    ✔ Sin timers por software
    ✔ Sin jitter
    ✔ Arquitectura MCPWM moderna (prelude)


*/


//static const char *TAG = "example";

#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define SERVO_PULSE_GPIO             20        // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

uint32_t angle_to_us(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

void app_main(void)
{
    printf("Crea timer y configuralo\n");

    //---------------------------
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = 
    {        
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,         //1MHz  
        .period_ticks  = SERVO_TIMEBASE_PERIOD,                //20ms
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    //---------------------------
    printf("Crea Handler (operador) y configuralo\n"); 
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = 
    {
        .group_id = 0, // operator must be in the same group to the timer
    };
    mcpwm_new_operator(&operator_config, &oper); //Crea el Operador. 
    mcpwm_operator_connect_timer(oper, timer);   //Asigna Timer al Operador.

    //---------------------------
    printf("Crea pulse_width_us y configuralo\n");  //pulse_width_us es la variable que almacena el ancho de pulso. Cuando el tiempo transcurrido = pulse_width_us -> Evento
    mcpwm_cmpr_handle_t pulse_width_us = NULL;
    mcpwm_comparator_config_t pulse_width_us_config = 
    {
        .flags.update_cmp_on_tez = true,  //El valor de anchura de pulso actualizado solo se aplica cuando el timer llega a cero
                                          //esto evita glitches (cambios a mitad de ciclo)
    };
    mcpwm_new_comparator(oper, &pulse_width_us_config, &pulse_width_us);

    //---------------------------
    printf("Crea generador y configuralo\n");       //generator es el nombre que se le da a la salida GPIO
    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = 
    {
        .gen_gpio_num = SERVO_PULSE_GPIO,
    };
    mcpwm_new_generator(oper, &generator_config, &generator);

    // Define el valor de la anchura de pulso inicial
    mcpwm_comparator_set_compare_value(pulse_width_us, 1250);

    printf("Define qué acción hace el generador cuando se cumple el tiempo (evento de tiempo) o cuando se produce un evento de comparación (compare event)\n");
    
    // go high on counter empty
    mcpwm_generator_set_action_on_timer_event(generator,                                                
                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,  //Contaje hacia arriba 
                                                                           MCPWM_TIMER_EVENT_EMPTY,   //Cuando el timer = 0
                                                                           MCPWM_GEN_ACTION_HIGH));   //pon el generador a 1
    // go low on compare threshold
    mcpwm_generator_set_action_on_compare_event(generator, 
                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, //Contaje hacia arriba
                                                                               pulse_width_us,               //cuando el timer vale pulse_width_us
                                                                               MCPWM_GEN_ACTION_LOW));   //pon el generador a 0

    //Pulso HIGH desde t=0 hasta t=compare_value dentro de cada periodo de 20 ms.

    //habilita y arranca timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    int angle = 0;
    int step = 1;
    while (1) {

        uint32_t p = angle_to_us(angle);

        printf("PCM: %d -  %lu\n", angle, p);                  //angulo - ancho de pulso en us

        mcpwm_comparator_set_compare_value(pulse_width_us, p);     //establece el nuevo valor del comparador  
                
        vTaskDelay(pdMS_TO_TICKS(30));
        if ((angle + step) >= 90 || (angle + step) <= -90) 
        {
            step *= -1;
        }
        angle += step;
    }
}
