idf_component_register(SRCS "Ejemplo16.c"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_tsens 
                                    #driver fue deprecated en versión v6.0, ahora se necesita añadir esp_driver_gpio y esp_driver_tens 
                       INCLUDE_DIRS "")
