idf_component_register(SRCS "main.c" #"app_trace_basic_example_main.c"
                       PRIV_REQUIRES driver app_trace esp_timer
                       INCLUDE_DIRS ".")
