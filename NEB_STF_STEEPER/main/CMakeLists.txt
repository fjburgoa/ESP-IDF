idf_component_register(SRCS "stepper_motor_encoder.c"
                            #"main_angle.c" 
                            "main_rampa.c"
                       PRIV_REQUIRES esp_driver_rmt esp_driver_gpio
                       INCLUDE_DIRS ".")
