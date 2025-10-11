#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE_HELLO";
static uint16_t tx_handle;
static uint16_t conn_handle;

static const ble_uuid128_t service_uuid =
    BLE_UUID128_INIT(0x6E,0x40,0x00,0x01,0xB5,0xA3,0xF3,0x93,
                     0xE0,0xA9,0xE5,0x0E,0x24,0xDC,0xCA,0x9E);
static const ble_uuid128_t tx_uuid =
    BLE_UUID128_INIT(0x6E,0x40,0x00,0x03,0xB5,0xA3,0xF3,0x93,
                     0xE0,0xA9,0xE5,0x0E,0x24,0xDC,0xCA,0x9E);

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &tx_uuid.u,
                .val_handle = &tx_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {0}
        },
    },
    {0}
};

static int gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected");
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected");
        conn_handle = 0;
        ble_svc_gap_device_name_set("ESP32S3_HELLO");
        ble_gap_adv_start(0, NULL, BLE_HS_FOREVER,
                          &(struct ble_gap_adv_params){.conn_mode = BLE_GAP_CONN_MODE_UND,
                                                       .disc_mode = BLE_GAP_DISC_MODE_GEN},
                          gap_event, NULL);
        break;
    }
    return 0;
}

static void ble_app_on_sync(void) {
    uint8_t addr_type;
    ble_hs_id_infer_auto(0, &addr_type);

    ble_svc_gap_device_name_set("ESP32S3_HELLO");
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event, NULL);

    ESP_LOGI(TAG, "Advertising");
}

void hello_task(void *param) {
    while (1) {
        if (conn_handle) {
            const char *msg = "Hello World\n";
            struct os_mbuf *om = ble_hs_mbuf_from_flat(msg, strlen(msg));
            if (ble_gatts_notify_custom(conn_handle, tx_handle, om) != 0) {
                os_mbuf_free_chain(om);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(ble_host_task);
    xTaskCreate(hello_task, "hello_task", 4096, NULL, 5, NULL);
}
