#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_spiffs.h"

// --------------------------------------------------
static void copy_file2_in_file1(void)
{
    FILE* f2 = fopen("/spiffs/MiFicheroEjemplo2.txt", "r");  
    if (f2 == NULL) 
    {
        printf("Error al abrir MiFicheroEjemplo2.txt\n");
        return;
    }else
        printf("Leyendo MiFicheroEjemplo2.txt\n");    
    
    FILE* f1 = fopen("/spiffs/MiFicheroEjemplo1.txt", "a+");  
    if (f1 == NULL) 
    {
        printf("Error al abrir MiFicheroEjemplo1.txt\n");
        return;
    }else
        printf("Leyendo MiFicheroEjemplo1.txt\n");

    char buf[1024];
    int numread = fread(buf, sizeof(char), sizeof(buf), f2);
    printf( "Número de caracteres leidos = %d\n", numread );
    
    printf("Copiando MiFicheroEjemplo2.txt en MiFicheroEjemplo1.txt\n");
    fwrite(buf,1,numread,f1);

    fclose(f1);
    fclose(f2);      
}

// --------------------------------------------------
static void read_file1(void)
{
    printf("Leyendo MiFicheroEjemplo1.txt\n");

    FILE* f = fopen("/spiffs/MiFicheroEjemplo1.txt", "r");  
    if (f == NULL) {
        printf("Error al abrir MiFicheroEjemplo1.txt\n");
        return;
    }

    char buf[1024];
    memset(buf, 0, sizeof(buf));
    fread(buf, 1, sizeof(buf), f);
    fclose(f);

    printf("Contenido de MiFicheroEjemplo1.txt:\n %s\n\n", buf);  
}

//-----------------------------------------------------------------------
void app_main(void)
{
    printf("Initializing SPIFFS\n");

    esp_vfs_spiffs_conf_t conf = {
      .base_path       = "/spiffs",
      .partition_label = NULL,
      .max_files       = 5,
      .format_if_mount_failed = false
    };

    // usar los ajustes definidos anteriormente.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) 
    {
        if (ret == ESP_FAIL) 
        {
            printf("Error al montar o formatear el sistema de ficheros \n");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            printf("La partición SPIFFS no se ha encontrado \n");
        } else 
            printf("Error al inicializar SPIFFS (%s)\n", esp_err_to_name(ret));
        
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) 
    {
       printf("Error al obtener la información de la partición SPIFFS (%s)", 
       esp_err_to_name(ret));
    } else 
        printf("Tamaño de la partición: total: %d, usada: %d\n", total, used);
        
    read_file1();		// Leer fichero MiFicheroEjemplo1.txt
    copy_file2_in_file1();	// Copiar MiFicheroEjemplo2.txt a MiFicheroEjemplo1.txt

    while(1){};
}
