idf_component_register(SRCS # "Ejemplo10.c"
                              "Ejemplo11.c"  
                       #PRIV_REQUIRES driver esp_driver_pcnt  #deprecated since version v6.0
                       PRIV_REQUIRES esp_driver_gpio esp_driver_pcnt                       
                       INCLUDE_DIRS "")
