#idf_component_register(SRCS "Ejemplo7.c"
#                       PRIV_REQUIRES esp_driver_gpio esp_driver_gptimer 
#                       INCLUDE_DIRS "")

idf_component_register(SRCS "Ejemplo8.c"
                       PRIV_REQUIRES esp_driver_gpio esp_timer
                       INCLUDE_DIRS "")

