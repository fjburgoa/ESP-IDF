
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

#define PCNT_HIGH_LIMIT  10  //contaje máx
#define PCNT_LOW_LIMIT  -10  //contaje min 

#define GPIO_CANAL_A      0  //GPIO0

//--------------------app_main ---------------------------
void app_main(void)
{    
    //-- crea y configura la unidad pcnt -----------------
    pcnt_unit_config_t unit_config =             
    {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit  = PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    pcnt_new_unit(&unit_config, &pcnt_unit);
   

    //-- configura el filtro anti glitches ---------------
    pcnt_glitch_filter_config_t filter_config =  
    {
        .max_glitch_ns = 1000,                   //nano segundos
    };
    pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);

    //instala los canales PCNT ----------------
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    pcnt_chan_config_t chan_a_config =           
    {
        .edge_gpio_num  = GPIO_CANAL_A,
        .level_gpio_num = -1,                    //Sin nivel asociado. Si fuera un 
                                                 //encoder y hubiera dirección,
                                         //tendríamos que meter el canal B
    };
    pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a);
     
    //establece cómo va a ser el contaje en cada flanco de subida/bajada
    pcnt_channel_set_edge_action(pcnt_chan_a, 
             PCNT_CHANNEL_EDGE_ACTION_HOLD,        //mantiene el contaje
             PCNT_CHANNEL_EDGE_ACTION_INCREASE);   //en flanco de bajada 
                                                   //incrementa el contaje  
    pcnt_unit_enable(pcnt_unit);      //habilita pcnt unit
    pcnt_unit_clear_count(pcnt_unit); //borra pcnt unit
    pcnt_unit_start(pcnt_unit);       //arranca pcnt unit

    while (1) 
    {
          int pulse_count = 0;
          pcnt_unit_get_count (pcnt_unit, &pulse_count);         
          printf("Contador: %2d\n", pulse_count);

          vTaskDelay(pdMS_TO_TICKS(100));  //delay de 100ms
    }
}


