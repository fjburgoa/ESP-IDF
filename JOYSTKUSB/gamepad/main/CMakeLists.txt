idf_component_register(
    # SRCS "acond_y_medida.c" "main2.c"  #programa que convierte el ESP32S3 en un joystick, lee el MPU6050 y controla los 10 LEDS
    SRCS "main.c"  #programa que convierte el ESP32S3 en un joystick - manda datos de prueba
    INCLUDE_DIRS "."
    REQUIRES driver
    PRIV_REQUIRES esp_driver_gpio
    )
