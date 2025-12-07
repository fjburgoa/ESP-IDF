idf_component_register(
    SRCS "borrame.c" #"tusb_hid_example_main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio
    )
