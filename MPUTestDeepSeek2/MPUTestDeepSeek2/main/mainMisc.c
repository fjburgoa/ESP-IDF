/*

Lee de varios I2C y los publica en el SH1106

*/


#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "HCM588L3L.h"
#include "SH1106.h"

static const char *TAG = "MISC";

//I2C Configuration
#define I2C_MASTER_SCL_IO           GPIO_NUM_9
#define I2C_MASTER_SDA_IO           GPIO_NUM_8
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000


// Function to initialize I2C -----------------------------------------------
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = 
    {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
 
    return ESP_OK;
}


extern uint8_t compass_bitmap[64*64];


//----------------------------------------------------------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "Starting HMC5883L application");
    
    // Initialize I2C------------------------------------
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return;
    }
    
    // Initialize HMC5883L--------------------------------
    ret = hmc5883l_init();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to initialize HMC5883L");
        return;
    }
    

    char texto[32] = {0};

    sh1106_init();
    sh1106_clear();

    //sh1106_draw_bitmap(32, 0, compass_bitmap, 64, 64);
    fill_test_circle((uint8_t *)compass_bitmap, 64, 64);
    


    for (int i=0;i<256;i++)
    {
        for (int j=0;j<16;j++)
        {
            printf("0x%02x, ",(unsigned int)compass_bitmap[i*16+j]);
        }
        printf("\n");
    }
    
    sh1106_draw_bitmap(32, 0, compass_bitmap, 64, 64);


    while(1)
    {

        bool ready; 

        if (hmc5883l_data_ready(&ready) == ESP_OK && ready) 
        {
            hmc5883l_data_t raw_data; //, calibrated_data;
            
            if (hmc5883l_read_raw(&raw_data) == ESP_OK)         
            {
                memset(texto,' ',sizeof(texto));
                sprintf(texto,"X: %03d Y: %03d Z: %03d",raw_data.x, raw_data.y, raw_data.z);

                //sh1106_draw_text(0, 3, texto);

                //ESP_LOGI(TAG, "Raw: X=%d, Y=%d, Z=%d ", raw_data.x, raw_data.y, raw_data.z);

            }

        } 
        
        vTaskDelay(pdMS_TO_TICKS(300)); // Read every 10ms



    }

}