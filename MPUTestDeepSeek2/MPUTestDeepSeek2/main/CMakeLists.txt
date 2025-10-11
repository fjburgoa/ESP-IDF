idf_component_register(SRCS #"procesado.c" "acond_y_medida.c" "mainMPU6050.c"
                                #"mainHMC588L3L_light.c"                                      
                                #"mainHMC588L3L.c"      
                                #"mainSH1106.c"
                                "mainMisc.c" "SH1106.c" "HCM588L3L.c"
                                #"main.c"
                                #"mainDraft.c"
                            REQUIRES driver

                       INCLUDE_DIRS ".")
