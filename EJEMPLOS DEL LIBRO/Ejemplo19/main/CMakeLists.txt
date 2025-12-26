idf_component_register(SRCS "Ejemplo19.c"
                       PRIV_REQUIRES driver nvs_flash
                       INCLUDE_DIRS "")

nvs_create_partition_image(nvs ../mynvs.csv FLASH_IN_PROJECT)
