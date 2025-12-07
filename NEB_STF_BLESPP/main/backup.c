#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"

/* NimBLE (ESP-IDF) */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* Declaración manual necesaria en varias versiones de IDF */
void ble_store_config_init(void);



static const char *TAG = "BLE_SPP_MIN";

/* UUIDs del servicio y característica tipo “UART” */
#define SPP_SERVICE_UUID   0xABF0
#define SPP_CHAR_UUID      0xABF1

/* Handle de la característica (para notificar) */
static uint16_t spp_chr_val_handle;

/* Estado de la conexión */
static uint16_t conn_handle_global = BLE_HS_CONN_HANDLE_NONE;
static bool subscribed = false;

/* Tipo de dirección propia */
static uint8_t own_addr_type;

/* -------------------- CALLBACK DE CARACTERÍSTICA ---------------------- */
/* Se llama cuando el cliente lee o escribe la característica */
static int spp_chr_access_cb(uint16_t conn_handle,
                             uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    switch (ctxt->op) {

    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        /* Datos recibidos desde el móvil */
        uint8_t buf[128];
        int len = ctxt->om->om_len;
        if (len > (int)sizeof(buf) - 1) {
            len = sizeof(buf) - 1;
        }

        /* Copiar datos a un buffer plano */
        ble_hs_mbuf_to_flat(ctxt->om, buf, len, NULL);
        buf[len] = 0;  // terminador para imprimir como string

        ESP_LOGI(TAG, "RX desde móvil (%d bytes): %s", len, (char *)buf);
        break;
    }

    case BLE_GATT_ACCESS_OP_READ_CHR:
        /* Si quisieras devolver algo al leer, se rellenaría ctxt->om aquí */
        ESP_LOGI(TAG, "Petición de lectura desde el móvil");
        break;

    default:
        break;
    }

    return 0;
}

/* -------------------- DEFINICIÓN GATT ---------------------- */
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(SPP_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(SPP_CHAR_UUID),
                .access_cb = spp_chr_access_cb,
                .val_handle = &spp_chr_val_handle,
                .flags =
                    BLE_GATT_CHR_F_READ    |
                    BLE_GATT_CHR_F_WRITE   |
                    BLE_GATT_CHR_F_WRITE_NO_RSP |
                    BLE_GATT_CHR_F_NOTIFY  |
                    BLE_GATT_CHR_F_INDICATE,
            },
            {0}  // fin de lista de características
        }
    },
    {0}      // fin de lista de servicios
};

/* -------------------- ENVÍO DE NOTIFICACIONES ---------------------- */
static void spp_send(const char *msg)
{
    if (!subscribed) {
        ESP_LOGW(TAG, "No hay cliente suscrito a NOTIFY");
        return;
    }

    if (conn_handle_global == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "No hay conexión BLE activa");
        return;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(msg, strlen(msg));
    if (!om) {
        ESP_LOGE(TAG, "Error creando mbuf para notificación");
        return;
    }

    int rc = ble_gatts_notify_custom(conn_handle_global, spp_chr_val_handle, om);
    ESP_LOGI(TAG, "Notificación enviada (rc=%d): %s", rc, msg);
}

/* -------------------- ADVERTISING ---------------------- */
static void start_advertising(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));
    memset(&adv_params, 0, sizeof(adv_params));

    /* Flags: general discoverable, sin BR/EDR */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* Nombre del dispositivo */
    const char *name = "ESP32-S3-SPP";
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    /* Anunciar el UUID del servicio */
    fields.uuids16 = (ble_uuid16_t[]){
        BLE_UUID16_INIT(SPP_SERVICE_UUID)
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error en ble_gap_adv_set_fields; rc=%d", rc);
        return;
    }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;   // connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;   // general discoverable

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error en ble_gap_adv_start; rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising iniciado");
}

/* -------------------- EVENTOS GAP ---------------------- */
static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle_global = event->connect.conn_handle;
            ESP_LOGI(TAG, "Conectado, handle=%d", conn_handle_global);
        } else {
            ESP_LOGW(TAG, "Fallo de conexión; status=%d, re-advertising", event->connect.status);
            start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Desconectado; reason=%d", event->disconnect.reason);
        conn_handle_global = BLE_HS_CONN_HANDLE_NONE;
        subscribed = false;
        /* Pequeña pausa y reanudamos advertising */
        vTaskDelay(pdMS_TO_TICKS(50));
        start_advertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        subscribed = event->subscribe.cur_notify || event->subscribe.cur_indicate;
        ESP_LOGI(TAG, "Subscribe: conn=%d, notify=%d, indicate=%d",
                 event->subscribe.conn_handle,
                 event->subscribe.cur_notify,
                 event->subscribe.cur_indicate);
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU actualizado: conn=%d mtu=%d",
                 event->mtu.conn_handle, event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

/* -------------------- CALLBACK DE RESET Y SYNC ---------------------- */
static void on_reset(int reason)
{
    ESP_LOGE(TAG, "Reset de host BLE; reason=%d", reason);
}

static void on_sync(void)
{
    int rc;

    /* Obtener tipo de dirección y MAC propia */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error en ble_hs_id_infer_auto; rc=%d", rc);
        return;
    }

    uint8_t addr_val[6];
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "Dirección BLE: %02X:%02X:%02X:%02X:%02X:%02X",
                 addr_val[5], addr_val[4], addr_val[3],
                 addr_val[2], addr_val[1], addr_val[0]);
    }

    /* Empezar a anunciar con nuestro callback GAP */
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields fields = {0};

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    const char *name = "ESP32-S3-SPP";
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;
    fields.uuids16 = (ble_uuid16_t[]){ BLE_UUID16_INIT(SPP_SERVICE_UUID) };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error en ble_gap_adv_start; rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising iniciado (on_sync)");
}

/* -------------------- TAREA HOST ---------------------- */
static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "Tarea NimBLE host iniciada");
    nimble_port_run();                 // no vuelve hasta que se pare el host
    nimble_port_freertos_deinit();
}

/* -------------------- MAIN ---------------------- */
void app_main(void)
{
    esp_err_t ret;

    /* NVS (requerido por el stack BLE/Wi-Fi) */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Inicializar NimBLE */
    ESP_ERROR_CHECK(nimble_port_init());

    /* Config global del host BLE */
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.gatts_register_cb = NULL;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Servicios básicos GAP/GATT */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    /* Registrar nuestro servicio SPP */
    int rc = ble_gatts_count_cfg(gatt_svcs);
    ESP_ERROR_CHECK(rc);
    rc = ble_gatts_add_svcs(gatt_svcs);
    ESP_ERROR_CHECK(rc);

    /* Config storage por defecto (claves, bonding, etc.) */
    ble_store_config_init();

    /* Lanzar la tarea del host NimBLE */
    nimble_port_freertos_init(ble_host_task);

    /* Bucle principal: enviar cada 2 segundos */
    while (1) {
        spp_send("Hola desde ESP32-S3 via BLE\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
