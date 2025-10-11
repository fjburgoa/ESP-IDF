#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "os/os_mbuf.h"

static uint16_t hello_attr_handle = 0;

/* UUIDs */
static const ble_uuid128_t hello_service_uuid = BLE_UUID128_INIT(
    0x12,0x34,0x56,0x78,0x90,0xab,0xcd,0xef,
    0x12,0x34,0x56,0x78,0x90,0xab,0xcd,0xef
);
static const ble_uuid128_t hello_char_uuid = BLE_UUID128_INIT(
    0xfe,0xdc,0xba,0x98,0x76,0x54,0x32,0x10,
    0xfe,0xdc,0xba,0x98,0x76,0x54,0x32,0x10
);

/* GATT access callback */
static int gatt_access_hello(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg) {
    hello_attr_handle = attr_handle; // Guardamos el handle aquí
    const char *msg = "Initial Hello";
    os_mbuf_append(ctxt->om, msg, strlen(msg));
    return 0;
}

/* GATT services */
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (ble_uuid_t *)&hello_service_uuid,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = (ble_uuid_t *)&hello_char_uuid,
                .access_cb = gatt_access_hello,
                .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ,
            },
            {0} /* terminator */
        },
    },
    {0} /* terminator */
};

/* GAP/advertising callback */
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch(event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if(event->connect.status == 0)
                printf("Connected!\n");
            else
                printf("Connect failed: %d\n", event->connect.status);
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            printf("Disconnected, reason: %d\n", event->disconnect.reason);
            ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                &(struct ble_gap_adv_params){
                    .conn_mode = BLE_GAP_CONN_MODE_UND,
                    .disc_mode = BLE_GAP_DISC_MODE_GEN,
                    .itvl_min = 0x40,
                    .itvl_max = 0x80,
                }, ble_gap_event, NULL);
            break;
        default: break;
    }
    return 0;
}

/* Advertise */
static void ble_app_advertise(void) {
    ble_svc_gap_device_name_set("nimble-s3");
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min = 0x20,
        .itvl_max = 0x40,
    };
    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if(rc) printf("Advertising failed: %d\n", rc);
    else printf("Advertising started!\n");
}



// static void ble_app_on_sync(void) {
//     ble_app_advertise();
// }

/* Host task */
void ble_host_task(void *param) {
    ble_app_advertise();
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/*
void ble_notify_task(void *param) {
    int count = 1;
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(500));

        // Espera a que haya un handle válido
        if(hello_attr_handle == 0) {
            printf("Characteristic handle not ready\n");
            continue;
        }

        struct ble_gap_conn_desc desc;
        int rc = ble_gap_conn_find(0, &desc); // busca la primera conexión
        if(rc != 0) {
            printf("No connection yet\n");
            continue;
        }

        // Tenemos conexión y handle listo
        char buf[32];
        int len = snprintf(buf, sizeof(buf), "Hello %d", count++);
        printf("Sending: %s\n", buf);

        struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, len);
        if(om) 
        {
            int rc_notify = ble_gattc_notify_custom(desc.conn_handle, hello_attr_handle, om);
            if(rc_notify != 0) {
                printf("Notify failed: %d\n", rc_notify);
                os_mbuf_free_chain(om);
            }
        } else {
            printf("No connection yet or handle not ready\n");
        }
    }
}
*/

/* ---------------- Notification task ---------------- */
void ble_notify_task(void *param) {
    int count = 1;
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(500));

        struct ble_gap_conn_desc desc;
        if(ble_gap_conn_find(0, &desc) == 0 && hello_attr_handle != 0) {
            char buf[32];
            int len = snprintf(buf, sizeof(buf), "Hello %d", count++);
            printf("Sending: %s\n", buf);

            struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, len);
            if(om) {
                int rc = ble_gattc_notify_custom(desc.conn_handle, hello_attr_handle, om);
                if(rc != 0) {
                    printf("Notify failed: %d\n", rc);
                    os_mbuf_free_chain(om);
                }
            }
        } else {
            printf("No connection yet or handle not ready\n");
        }
    }
}

/* Main */
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nimble_port_init();

    //ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.sync_cb = ble_app_advertise;  // Se llamará cuando NimBLE esté listo

    ble_gatts_count_cfg(gatt_svcs);
    int rc = ble_gatts_add_svcs(gatt_svcs);
    if(rc != 0) {
        printf("Error adding services: %d\n", rc);
    } else {
        uint16_t def_handle = 0;
        uint16_t val_handle = 0;
        int rc_chr = ble_gatts_find_chr((ble_uuid_t *)&hello_service_uuid,
                                        (ble_uuid_t *)&hello_char_uuid,
                                        &def_handle,
                                        &val_handle);
        if(rc_chr == 0) {
            hello_attr_handle = val_handle; // guardamos el handle del valor
            printf("Characteristic handle: %u\n", hello_attr_handle);
        } else {
            printf("Failed to find characteristic\n");
        }
    }

    nimble_port_freertos_init(ble_host_task);
    xTaskCreate(ble_notify_task, "ble_notify", 4096, NULL, 5, NULL);
}


//desde TERMINAL
//

//idf.py -p COM14 build flash monitor