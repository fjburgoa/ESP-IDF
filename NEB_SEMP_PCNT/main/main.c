
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

static const char *TAG = "example";

#define PCNT_HIGH_LIMIT  15  //contaje máx
#define PCNT_LOW_LIMIT  -10  //contaje min 

#define GPIO_CANAL_A      0  //GPIO0

int flag_int = 0;

#ifdef CON_ISR
    //--------------------------------------------
    static bool ISR_PCNT(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
    {
        flag_int = 1; 
        return false;
    }
#endif

//--------------------------------------------
void app_main(void)
{
    //instala la unidad pcnt
    pcnt_unit_config_t unit_config = 
    {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit  = PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    pcnt_new_unit(&unit_config, &pcnt_unit);

    //configura el filtro anti glitches
    pcnt_glitch_filter_config_t filter_config = 
    {
        .max_glitch_ns = 1000,
    };
    pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);

    //instala los canales PCNT
    pcnt_chan_config_t chan_a_config = 
    {
        .edge_gpio_num = GPIO_CANAL_A,
        .level_gpio_num = -1,   // sin nivel asociado. Si fuera un encoder y hubiera dirección, tendríamos que meter el canal B
    };
    
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
     
    //establece cómo va a ser el contaje en cada flanco de subida/bajada
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_HOLD, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
 
 #ifdef CON_ISR  //si trabajamos con ISR
    //define los contajes que van a disparar la ISR
    int watch_points[] = {0, 5, EXAMPLE_PCNT_HIGH_LIMIT};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
    }

    //define la ISR
    pcnt_event_callbacks_t cbs = {
        .on_reach = ISR_PCNT,
    };
 
    //registra la ISR
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, NULL));
#endif

    //enable pcnt unit
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    //clear pcnt unit
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    //start pcnt unit
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    while (1) 
    {
          int pulse_count = 0;
          ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));         
          printf("Contador: %2d - %lu\n", pulse_count, xTaskGetTickCount()*portTICK_PERIOD_MS);         
          vTaskDelay(pdMS_TO_TICKS(200));
#ifdef CON_ISR  //si trabajamos con ISRS
          if (flag_int)
          {
            flag_int = 0;
            printf("interrupción\n");
          }    
#endif 
    }
}
