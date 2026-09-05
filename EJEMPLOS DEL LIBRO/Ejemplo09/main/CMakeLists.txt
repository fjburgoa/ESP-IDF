idf_component_register(SRCS "Ejemplo9.c"
                       #PRIV_REQUIRES driver #deprecated since version v6.0
                       PRIV_REQUIRES esp_driver_gpio esp_driver_gptimer esp_driver_ledc  #required in version v6.0 
                       INCLUDE_DIRS "")
