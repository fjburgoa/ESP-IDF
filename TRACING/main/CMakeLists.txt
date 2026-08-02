idf_component_register(SRCS #  "trace_test.c" "trace_recorder.c"   
                               "trace_test_jtag.c" "trace_recorder_jtag.c"   
                       PRIV_REQUIRES driver app_trace #esp_adc
                       INCLUDE_DIRS "")
