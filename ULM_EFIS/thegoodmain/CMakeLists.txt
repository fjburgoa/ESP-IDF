idf_component_register(
    SRCS
        "main.c"
        "EFIS_I2C.c"
        "wifi_ap.c"
        "webserver.c"
        "websocket.c"
        "BMP280.c"
        "BNO055.c"
        "BNO055_driver.c"
        "BNO055_processing.c"
        "GPS.c"
        "DataLogger.c"
        "MPU6050.c"

    INCLUDE_DIRS
        "include"

    REQUIRES
        esp_wifi
        esp_event
        esp_netif
        esp_http_server
        nvs_flash
        esp_timer
        esp_driver_i2c
        esp_driver_gpio
        esp_driver_uart
        esp_driver_tsens
        spiffs
)

target_add_binary_data(
    ${COMPONENT_LIB}
    "index.html"
    TEXT
)


