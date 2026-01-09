idf_component_register(SRCS #"main.c" 
                            "main2.c"  
                       PRIV_REQUIRES esp_wifi nvs_flash wifi_provisioning driver
                       INCLUDE_DIRS ".")
