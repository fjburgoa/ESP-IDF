
idf_component_register(SRCS "Ejemplo33.c"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_uart #driver está deprecated desde la versión v6.0, es necesario añadir esp_driver_uart
                       INCLUDE_DIRS "")
