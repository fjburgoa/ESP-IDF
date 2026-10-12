idf_component_register(SRCS "Ejemplo32.c"
                       PRIV_REQUIRES esp_driver_gpio esp_adc esp_timer #driver está deprecated desde la versión v6.0
                       INCLUDE_DIRS "")
