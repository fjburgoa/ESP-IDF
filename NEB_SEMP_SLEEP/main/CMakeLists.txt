set(srcs #"deep_sleep_example_main.c"
         #                   "gpio_wakeup.c"
         #                   "ext_wakeup.c")
          "main_RTC.c")
set(includes ".")

idf_component_register(SRCS ${srcs}
                       PRIV_REQUIRES nvs_flash ulp esp_driver_gpio esp_timer
                       INCLUDE_DIRS ${includes})
