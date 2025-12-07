#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "driver/temperature_sensor.h"

float tsens_value;
void app_main(void)
{
    //handler y configuración
    temperature_sensor_handle_t temp_sensor        = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    
    //instala e inicializa el sensor
    temperature_sensor_install(&temp_sensor_config, &temp_sensor);
    temperature_sensor_enable(temp_sensor);

    while (1) 
    {
        temperature_sensor_get_celsius(temp_sensor, &tsens_value); //Medida
        printf("Temperatura %dºC\n", (int)tsens_value);              //Imprime
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
