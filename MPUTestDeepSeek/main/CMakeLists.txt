#idf_component_register(SRCS "acond_y_medida.c" "procesado.c" "i2ctools_example_main.c"
#                            "cmd_i2ctools.c"
#                       INCLUDE_DIRS ".")


idf_component_register(SRCS "main.c" "procesado.c" "acond_y_medida.c"
                        REQUIRES driver
                       INCLUDE_DIRS ".")
