#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stepper_motor_encoder.h"


//#define STEP_MOTOR_GPIO_EN                 15 // Lo hacemos a mano
//#define STEP_MOTOR_ENABLE_LEVEL            0  // DRV8825 is enabled on low level

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


    //----------VEL MAX GIRO------------------------------------------
    const static uint32_t pulses_nr        = 2*NR_PULSOS_VUELTA;  //define la velocidad -> pulses_nr = numero de pulsos por segundo

    //----------CURVA DE ACELERACIÓN
    stepper_motor_curve_encoder_config_t accel_encoder_config = 
    {
        .resolution    = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = NR_PULSOS_VUELTA,
        .start_freq_hz = 0.25*pulses_nr,  //velocidad inicial 
        .end_freq_hz   = pulses_nr,      //velocidad final 
    };
    rmt_encoder_handle_t accel_motor_encoder = NULL;
    rmt_new_stepper_motor_curve_encoder(&accel_encoder_config, &accel_motor_encoder);

    //----------CURVA DE VELOCIDAD CONSTANTE-------------------------- 
    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution    = STEP_MOTOR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t uniform_motor_encoder = NULL;
    rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder);

    //----------CURVA DE DECELERACIÓN---------------------------------
    stepper_motor_curve_encoder_config_t decel_encoder_config = {
        .resolution    = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = NR_PULSOS_VUELTA,
        .start_freq_hz = pulses_nr,          //velocidad inicial 
        .end_freq_hz   = 0.25*pulses_nr,     //velocidad final 
    };
    rmt_encoder_handle_t decel_motor_encoder = NULL;
    rmt_new_stepper_motor_curve_encoder(&decel_encoder_config, &decel_motor_encoder);

    //----------HABILITA EL CANAL RMT---------------------------------
    rmt_enable(motor_chan);

    rmt_transmit_config_t tx_config = 
    {
        .loop_count = 0,
    };

    uint32_t accel_samples    = accel_encoder_config.sample_points;
    uint32_t decel_samples    = decel_encoder_config.sample_points;

    while (1) 
    {
        tx_config.loop_count = 0;  
        rmt_transmit(motor_chan, accel_motor_encoder, &accel_samples, sizeof(accel_samples), &tx_config);  //ACELERACIÓN
        
        tx_config.loop_count = 2*pulses_nr;                                                                //define la duración
        rmt_transmit(motor_chan, uniform_motor_encoder, &pulses_nr, sizeof(pulses_nr), &tx_config);        //VELOCIDAD CONSTANTE

        tx_config.loop_count = 0;
        rmt_transmit(motor_chan, decel_motor_encoder, &decel_samples, sizeof(decel_samples), &tx_config);  //DECELERACIÓN
        
        rmt_tx_wait_all_done(motor_chan, -1);         // Espera a que todas las acciones se hayan ejecutado (bloqueante)

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
