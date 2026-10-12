
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

void app_main(void)
{
    // Inicializa NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        // Error a la hora de inicializar la zona NVS. Esta debe ser borrada 
        nvs_flash_erase();
        err = nvs_flash_init();            // Repite la inicialización
    }

    printf("Abriendo acceso a NVS\n");

    nvs_handle_t my_handle;
    err = nvs_open("key1", NVS_READWRITE, &my_handle);   //abre el namespace key1
    
    if (err != ESP_OK) 
    {
        printf("Error (%s) abriendo NVS\n", esp_err_to_name(err));
    }else{
        printf("Done\n");
        int32_t restart_counter = 0; // variable contenedora 1
        int32_t start_counter   = 0; // variable contenedora 2
        
        // Lee la primera variable (restart_counter)
        printf("Leyendo variable restart_counter de NVS/key1 ... ");        
        err = nvs_get_i32(my_handle, "restart_counter", &restart_counter);    
        switch (err) 
        {
            case ESP_OK:
                printf("OK, restart_counter = %" PRIu32 "\n", restart_counter);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("Clave no encontrada!\n");
                break;
            default:
                printf("Error (%s) leyendo!\n", esp_err_to_name(err));
        }

        // Lee la segunda variable (start_counter)
        printf("Leyendo variable start_counter de NVS/key1 ... ");          // Read
        err = nvs_get_i32(my_handle, "start_counter", &start_counter);
        
       switch (err) 
       {
            case ESP_OK:
                printf("OK, start_counter = %" PRIu32 "\n", start_counter);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("No se ha podido inicializar!\n");
                break;
            default :
                printf("Error (%s) leyendo!\n", esp_err_to_name(err));
        }

        printf("actualizando únicamente *restart_counter* en la NVS ... ");
        
        err = nvs_set_i32(my_handle, "restart_counter", ++restart_counter);   
        
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
        printf("Committing updates in NVS ... ");

        err = nvs_commit(my_handle);    // No olvidar hacer el commit!

        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        nvs_close(my_handle);           // Cierra la NVS

    }

    while(1){}; // Bucle principal sin tareas adicionales
} 

