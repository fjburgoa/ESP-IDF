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

static const char *TAG = "NUS_HELLO";

static uint16_t nus_tx_handle;
static uint16_t conn_handle = 0;

static bool tx_ready = false;

static void ble_app_on_sync(void);

/* Nordic UART Service UUIDs */
static const ble_uuid128_t nus_service_uuid =
    BLE_UUID128_INIT(0x6E,0x40,0x00,0x01,0xB5,0xA3,0xF3,0x93,
                     0xE0,0xA9,0xE5,0x0E,0x24,0xDC,0xCA,0x9E);
static const ble_uuid128_t nus_rx_uuid =
    BLE_UUID128_INIT(0x6E,0x40,0x00,0x02,0xB5,0xA3,0xF3,0x93,
                     0xE0,0xA9,0xE5,0x0E,0x24,0xDC,0xCA,0x9E);
static const ble_uuid128_t nus_tx_uuid =
    BLE_UUID128_INIT(0x6E,0x40,0x00,0x03,0xB5,0xA3,0xF3,0x93,
                     0xE0,0xA9,0xE5,0x0E,0x24,0xDC,0xCA,0x9E);

/* RX callback (optional echo/log) */
static int nus_rx_cb(uint16_t conn_handle_param, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_ATT_ACCESS_OP_WRITE) {
        char buf[64];
        ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf) - 1, NULL);
        ESP_LOGI(TAG, "RX: %s", buf);
    }
    return 0;
}

/* GATT service definition */
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nus_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &nus_rx_uuid.u,
                .access_cb = nus_rx_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &nus_tx_uuid.u,
                .val_handle = &nus_tx_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {0}
        },
    },
    {0}
};

/* GAP event handler: manages connections & reconnections */
static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
if (event->connect.status == 0) {
        conn_handle = event->connect.conn_handle;
        ESP_LOGI(TAG, "Connected!");
        
        // Esperamos a que el handle de TX esté listo
        while (nus_tx_handle == 0) {
           vTaskDelay(pdMS_TO_TICKS(50));
        }
        tx_ready = true;

        // ✅ Actualizar parámetros de conexión ahora que hay conn_handle válido
        struct ble_gap_upd_params conn_params = {0};
        conn_params.itvl_min = 0x18;        // 30ms
        conn_params.itvl_max = 0x28;        // 50ms
        conn_params.latency = 0;
        conn_params.supervision_timeout = 0xC8; // 5s
        conn_params.min_ce_len = 0;
        conn_params.max_ce_len = 0;

        int rc = ble_gap_update_params(conn_handle, &conn_params);
        if (rc != 0) {
            ESP_LOGW(TAG, "ble_gap_update_params failed: %d", rc);
        }

    } else {
        ESP_LOGI(TAG, "Connection failed; restarting advertising");
        conn_handle = 0;
        tx_ready = false;
        ble_app_on_sync();
    }
    break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected; restarting advertising");
        conn_handle = 0;
        ble_app_on_sync();
        break;

    default:
        break;
    }
    return 0;
}

/* Advertising & GATT setup after NimBLE sync */
static void ble_app_on_sync(void)
{
    uint8_t addr_type;
    ble_hs_id_infer_auto(0, &addr_type);

    const char *name = "ESP32S3_NUS";
    ble_svc_gap_device_name_set(name);

    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    struct ble_hs_adv_fields fields = {0};
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = 0x40; // ~25ms
    adv_params.itvl_max = 0x60; // ~37ms

    ble_gap_adv_start(addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event, NULL);

    ESP_LOGI(TAG, "Advertising as %s", name);
}

/* Periodic TX task: sends "Hello World" every second */
void nus_tx_task(void *param)
{
    while (1) 
    {
        if (conn_handle != 0 && nus_tx_handle != 0) {
            const char *msg = "Hello World\n";
            struct os_mbuf *om = ble_hs_mbuf_from_flat(msg, strlen(msg));

            int rc = ble_gatts_notify_custom(conn_handle, nus_tx_handle, om);
            if (rc != 0) {
                // Notification failed; free mbuf
                os_mbuf_free_chain(om);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* NimBLE host task */
void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.sync_cb = ble_app_on_sync;

    nimble_port_freertos_init(ble_host_task);

    xTaskCreate(nus_tx_task, "nus_tx_task", 12288, NULL, 5, NULL);
}
