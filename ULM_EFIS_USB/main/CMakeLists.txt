idf_component_register(
    SRCS
        "main.c"
        "EFIS_I2C.c"
        "MPU6050.c"
        "BNO055.c"
        "BNO055_driver.c"
        "BNO086.c"
        "BNO086_driver.c"
        "usb_hid.c"
        "settings.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c esp_driver_gpio nvs_flash esp_timer esp_tinyusb
)
