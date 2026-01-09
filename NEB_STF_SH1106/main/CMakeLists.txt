idf_component_register(SRCS 
                        "main.c"   "SH1106.c"
                        #"main_SH1106_and_HCM588L3L.c" "SH1106.c" # "HCM588L3L.c"
                        REQUIRES driver
                        INCLUDE_DIRS ".")
