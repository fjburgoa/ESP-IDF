set(srcs #"esp_hid_joystick_main.c"
         #"esp_hid_mouse_main.c"
         #nus_ble_main_ds.c
         nus_ble_main_gpt.c
         #"esp_hid_gap.c"
         )
set(include_dirs ".")

idf_component_register(SRCS  "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid
                       PRIV_REQUIRES nvs_flash)


                       