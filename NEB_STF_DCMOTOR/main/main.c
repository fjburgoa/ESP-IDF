#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "bdc_motor.h"


// Enable this config,  we will print debug formated string, which in return can be captured and parsed by Serial-Studio
#define SERIAL_STUDIO_DEBUG           CONFIG_SERIAL_STUDIO_DEBUG

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks

#define BDC_MCPWM_GPIO_A              7
#define BDC_MCPWM_GPIO_B              15

void app_main(void)
{

    printf("Create DC motor handler\n");
    bdc_motor_config_t motor_config = 
    {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = 
    {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    
    bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor);
    
    printf("Enable motor\n");
    bdc_motor_enable(motor);
    vTaskDelay(pdMS_TO_TICKS(1000));    

    printf("Forward motor\n");
    bdc_motor_set_speed(motor, 0);     // 0 us - 25%
    bdc_motor_forward(motor);
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    printf("SPEED F = 100\n");
    bdc_motor_set_speed(motor, 100);   //10 us - 25%
    vTaskDelay(pdMS_TO_TICKS(5000));    
    
    printf("SPEED F = 200\n");
    bdc_motor_set_speed(motor, 200);   //20 us - 50%
    vTaskDelay(pdMS_TO_TICKS(5000));    

    printf("SPEED F = 300\n");
    bdc_motor_set_speed(motor, 300);   //30 us - 75%
    vTaskDelay(pdMS_TO_TICKS(5000));    

    printf("SPEED F = 400\n");
    bdc_motor_set_speed(motor, 400);   //40 us - 100%
    vTaskDelay(pdMS_TO_TICKS(5000));       

    printf("reverse motor\n");
    bdc_motor_set_speed(motor, 0);     
    bdc_motor_reverse(motor);
    vTaskDelay(pdMS_TO_TICKS(5000));    

    printf("SPEED R = 100\n");
    bdc_motor_set_speed(motor, 100);
    vTaskDelay(pdMS_TO_TICKS(5000));    
    
    printf("SPEED R = 200\n");
    bdc_motor_set_speed(motor, 200);
    vTaskDelay(pdMS_TO_TICKS(5000));    

    printf("SPEED R = 300\n");
    bdc_motor_set_speed(motor, 300);
    vTaskDelay(pdMS_TO_TICKS(5000));    

    printf("SPEED R = 400\n");
    bdc_motor_set_speed(motor, 400);
    vTaskDelay(pdMS_TO_TICKS(5000));    

    printf("SPEED = Coast\n");
    bdc_motor_coast(motor);            //las dos a 0
    vTaskDelay(pdMS_TO_TICKS(5000));    

    printf("SPEED = Brake\n");   
    bdc_motor_brake(motor);            //las dos a 1
    vTaskDelay(pdMS_TO_TICKS(5000));    
    
    uint32_t velocidad = 0;
    bdc_motor_reverse(motor);

    while (1) 
    {
        //el valor máximo que podemos darle a la velocidad es 399.9999, que corresponden a 40us. 
        //valores más grandes excederían el periodo correspondiente a 25kHz
        

        printf("SPEED F = %lu\n",velocidad);
        bdc_motor_set_speed(motor, velocidad);  //100 us  0.1ms
        velocidad = velocidad + 1;
        if (velocidad > 400)
            velocidad = 0;

        //bdc_motor_set_speed(motor, 100);   //10 us
        //bdc_motor_set_speed(motor, 10);  //1 us
        vTaskDelay(pdMS_TO_TICKS(10));   



    }
}
