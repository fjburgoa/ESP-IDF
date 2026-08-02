#idf_component_register(SRCS #"softap_example_main.c"
#                            #PRIV_REQUIRES esp_wifi nvs_flash 
#                            "softap_with_socket.c"
#                            REQUIRES esp_wifi esp_event esp_netif nvs_flash esp_http_server
#                            INCLUDE_DIRS ".")

idf_component_register(
    SRCS
        "main.c"
        "wifi_ap.c"
        "webserver.c"
        "websocket.c"

    INCLUDE_DIRS
        "include"

    REQUIRES
        esp_wifi
        esp_event
        esp_netif
        esp_http_server
        nvs_flash
        esp_timer
)

target_add_binary_data(
    ${COMPONENT_LIB}
    "index.html"
    TEXT
)


