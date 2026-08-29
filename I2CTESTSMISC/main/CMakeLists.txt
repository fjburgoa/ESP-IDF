idf_component_register(SRCS    "procesado.c" "acond_y_medida.c" "mainMPU6050.c"  #Accelerometro
                                #"mainHMC588L3L_light.c"       #                               
                                #"mainHMC588L3L.c"      
                                #"mainSH1106.c"
                                #"mainMisc.c" "SH1106.c" "HCM588L3L.c"         #pantalla + Magnetometro
                                #"mainDraft.c"                                 #pantalla intentando una brújula + Magnetometro
                                #"mainGY_BM280.c"                              #barómetro
                                #"CPUAvion.c" "SH1106.c" "BM280.c" "HCM588L3L.c" "MPU6050.c"
                                #"mainBNO055_Simple.c"
                                #"mainBNO055_EFIS.c"
                                #"mainGPS.c" "GPS.c"
                                #"mainGPS_basic.c"
                                #"main_GPS_FAST.c"
                                #"main_FINDI2C_Devices.c"
                                
                               
                            REQUIRES driver

                       INCLUDE_DIRS ".")
