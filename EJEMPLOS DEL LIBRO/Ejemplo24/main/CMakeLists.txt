set(requires esp-tls nvs_flash esp_event esp_netif esp_http_client driver)
idf_build_get_property(target IDF_TARGET)

idf_component_register(SRCS "Ejemplo24.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES ${requires}
                    EMBED_TXTFILES howsmyssl_com_root_cert.pem)
