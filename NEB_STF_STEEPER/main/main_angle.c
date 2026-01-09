#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "stepper_motor_encoder.h"


#define STEP_MOTOR_GPIO_DIR      6
#define STEP_MOTOR_GPIO_STEP     7

#define COEF M0_M1_M2            8
#define NR_PULSOS_VUELTA      (360/7.5)*8

#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 1
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE

#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution

void app_main(void)
{
    //----------INICIALIZA GPIO----------------------------------------    
    gpio_config_t en_dir_gpio_config = 
    {
        .mode         = GPIO_MODE_OUTPUT,
        .intr_type    = GPIO_INTR_DISABLE,            
        .pin_bit_mask = 1ULL << STEP_MOTOR_GPIO_DIR, 
    };
    gpio_config(&en_dir_gpio_config);
     
    //----------CREA CANAL RMT----------------------------------------
    rmt_channel_handle_t motor_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,       // select clock source
        .gpio_num          = STEP_MOTOR_GPIO_STEP,      // GPIO asociado
        .mem_block_symbols = 64,          
        .resolution_hz     = STEP_MOTOR_RESOLUTION_HZ,  // Resolución del timer
        .trans_queue_depth = 10,                        // set the number of transactions that can be pending in the background
    };
    rmt_new_tx_channel(&tx_chan_config, &motor_chan);

    //----------DIRECCIÓN DE GIRO-------------------------------------
    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);

    //----------CURVA DE VELOCIDAD CONSTANTE-------------------------- 
    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution    = STEP_MOTOR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t uniform_motor_encoder = NULL;
    rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder);

    //----------HABILITA EL CANAL RMT---------------------------------
    rmt_enable(motor_chan);

    rmt_transmit_config_t tx_config = 
    {
        .loop_count = 0,
    };

    //----------VEL MAX GIRO------------------------------------------
    const static uint32_t pulses_nr        = 2*NR_PULSOS_VUELTA;  //define la velocidad -> pulses_nr = numero de pulsos por segundo

    while (1) 
    {
        tx_config.loop_count = 0.125*pulses_nr;                                                             //define la duración -> 90deg
        rmt_transmit(motor_chan, uniform_motor_encoder, &pulses_nr, sizeof(pulses_nr), &tx_config);        //VELOCIDAD CONSTANTE
       
        rmt_tx_wait_all_done(motor_chan, -1);         // Espera a que todas las acciones se hayan ejecutado (bloqueante)

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
