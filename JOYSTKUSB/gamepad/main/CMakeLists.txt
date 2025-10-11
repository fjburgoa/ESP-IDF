idf_component_register(
    SRCS "acond_y_medida.c" "main2.c"  
    INCLUDE_DIRS "."
    REQUIRES driver
    PRIV_REQUIRES esp_driver_gpio
    )
