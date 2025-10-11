/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
/*
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
*/

#include <stdio.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "led_strip.h"
#include "sdkconfig.h"


/* este programa hace varias cosas:
- gestión de timer
- ADC
- PWM
- blink led
- rgb

(se ha creado con el template de blink porque si no no funcionaba)
 */


//configuración del PWM
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (4) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz

//leds externos
#define led1 4
#define led2 6
#define BLINK_GPIO CONFIG_BLINK_GPIO


uint8_t led_status = 0;

esp_err_t init_led(int led);
esp_err_t init_rgb(void);

esp_err_t blink_led(int led);
esp_err_t blink_rgb(void);

esp_err_t set_adc(int channel);

static const char* TAG = "Ex";

static led_strip_handle_t led_strip;

esp_timer_handle_t periodic_timer;

uint16_t dutycycle = 0;

bool s_led_state = false;

/*-------------------------------------------------------------------------------*/
static void example_pwm_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = dutycycle, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}
/*-------------------------------------------------------------------------------*/
static void periodic_timer_callback(void* arg)
{
    int64_t time_since_boot = esp_timer_get_time();
    
    blink_led(led2);                                        //led 2 blink
   
    int lectura_adc = adc1_get_raw(ADC1_CHANNEL_4);         //lee ADC  

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, lectura_adc*2);  //copiamos el valor de la lectura ADC al led
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
  
    ESP_LOGI(TAG, "timer since boot: %lld us, adc=%d", time_since_boot,lectura_adc); 

    blink_rgb();

}
/*-------------------------------------------------------------------------------*/
esp_err_t blink_rgb(void)
{
    
    s_led_state = !s_led_state;
    if (s_led_state) {        
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);   // Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color         
        led_strip_refresh(led_strip);                    // Refresh the strip to send data 
    } else {
        
        led_strip_clear(led_strip);
    }
    return ESP_OK;
}
/*-------------------------------------------------------------------------------*/
esp_err_t init_rgb(void)
{    
    led_strip_config_t strip_config = {    // LED strip initialization with the GPIO and pixels number
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);

    led_strip_clear(led_strip);

    return ESP_OK;
}
/*-------------------------------------------------------------------------------*/
esp_err_t init_led(int led)
{
    gpio_reset_pin(led);
    gpio_set_direction(led,GPIO_MODE_DEF_OUTPUT);
    return ESP_OK;    
}
/*-------------------------------------------------------------------------------*/
esp_err_t blink_led(int led)
{
    led_status = !led_status;
    gpio_set_level(led,led_status);
    return ESP_OK;
}
/*-------------------------------------------------------------------------------*/
esp_err_t set_adc(int channel)
{
    adc1_config_channel_atten(channel,ADC_ATTEN_DB_12);  //atenuación del canal... hasta cuanto puede medir
    adc1_config_width(ADC_WIDTH_BIT_12);
    return ESP_OK;
}
/*-------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------*/
void app_main(void)
{
    init_led(led1);                                     //led 1 como salida
    init_led(led2);                                     //led 2 como salida
    init_rgb();

    set_adc(ADC1_CHANNEL_4);                            //ADC en pin5 (canal 4)                        

    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &periodic_timer_callback,       //callback            
            .name = "periodic"                          //name
    };

    //configure and start timer
    esp_timer_create(&periodic_timer_args, &periodic_timer);
    esp_timer_start_periodic(periodic_timer, 100000);

    // Set the LEDC peripheral configuration
    example_pwm_init();
}