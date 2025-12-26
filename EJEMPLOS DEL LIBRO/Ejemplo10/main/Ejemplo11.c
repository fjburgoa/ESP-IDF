
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

#define PCNT_HIGH_LIMIT  10  //contaje máx
#define PCNT_LOW_LIMIT  -10  //contaje min

#define GPIO_CANAL_A      0  //GPIO0

int flag_int = 0;            //Flag interrupción 

//--------------------Rutina ISR-------------------------
static bool ISR_PCNT(pcnt_unit_handle_t unit, 
                     const pcnt_watch_event_data_t *edata, 
                     void *user_ctx)
{
    flag_int = 1;            //setea flag interrupción
    return false;
}
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
   
    //configura el filtro anti glitches ------------------
    pcnt_glitch_filter_config_t filter_config =  
    {
        .max_glitch_ns  = 10000,   //nano segundos (10 ms)
    };
    pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);

    //instala los canales PCNT ---------------------------
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    pcnt_chan_config_t chan_a_config =           
    {
        .edge_gpio_num  = GPIO_CANAL_A,
        .level_gpio_num = -1,               //Sin nivel asociado. Si fuera un 
                                            //encoder y hubiera dirección,
                                            //tendríamos que meter el canal B
    };
    pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a);
     
    //establece cómo va a ser el contaje en cada flanco de subida/bajada
    pcnt_channel_set_edge_action(pcnt_chan_a, 
             PCNT_CHANNEL_EDGE_ACTION_HOLD,        //mantiene el contaje
             PCNT_CHANNEL_EDGE_ACTION_INCREASE);   //en flanco de bajada 
                                                   //incrementa el contaje  
    
    //define los contajes que van a disparar la ISR. Se asume que el eje da media 
    //vuelta cada 5 pulsos y una vuelta entera cada 10 pulsos.
    int watch_points[] = {0,  5};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) 
    {
        pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]);
    }

    pcnt_event_callbacks_t cbs = //define la ISR
    { 
        .on_reach = ISR_PCNT,
    };
 
    //registra la ISR
    pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, NULL);

    pcnt_unit_enable(pcnt_unit);      //habilita pcnt unit
    pcnt_unit_clear_count(pcnt_unit); //borra pcnt unit
    pcnt_unit_start(pcnt_unit);       //arranca pcnt unit

    while (1) 
    {
       int pulse_count = 0;
           
       //como Buena práctica, gestiona la ISR dentro del bucle principal
       if (flag_int)
       {
            flag_int = 0;   
            pcnt_unit_get_count(pcnt_unit, &pulse_count); 
            
            if (pulse_count==5)
                printf("media vuelta completada\n");

            if (pulse_count==0)
                printf("vuelta entera completada\n");
       }    
       vTaskDelay(pdMS_TO_TICKS(10));
    }
}

