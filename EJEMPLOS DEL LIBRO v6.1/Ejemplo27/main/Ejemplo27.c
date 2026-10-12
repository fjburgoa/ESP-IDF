
#include <stdio.h>
#include <string.h>

#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// Declaración incluida en NimBLE (manejo de bonding y claves)
void ble_store_config_init(void);

// UUIDs del servicio tipo SPP
#define SPP_SVC_UUID  0xABF0       // Servicio
#define SPP_CHR_UUID  0xABF1       // Característica 

static uint16_t conn_h     = BLE_HS_CONN_HANDLE_NONE; //Handle de conexión activa.
static uint16_t chr_h      = 0;                       //Handle de la caract. GATT   
static bool     notify_en  = false;
static uint8_t  own_addr_type;

//-------------------- Callback de RX  ------------------------
static int chr_cb(uint16_t conn_handle,
                  uint16_t attr_handle,
                  struct ble_gatt_access_ctxt *ctxt,
                  void *arg)
{
    // filtra escrituras (WRITE / WRITE NO RSP).
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) 
    {
        uint8_t buf[128];
        int len = ctxt->om->om_len;
        if (len > (int)sizeof(buf) - 1) len = sizeof(buf) - 1;

        ble_hs_mbuf_to_flat(ctxt->om, buf, len, NULL); //mbuf BLE -> buffer plano.
        buf[len] = 0;

        printf("RX: %s\n", (char *)buf); //imprime lo recibido por el smartphone
    }
    return 0;
}

//------------Definición del servicio GATT -----------------------------
static const struct ble_gatt_svc_def svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(SPP_SVC_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid       = BLE_UUID16_DECLARE(SPP_CHR_UUID),
                .access_cb  = chr_cb,
                .val_handle = &chr_h,
                .flags      = BLE_GATT_CHR_F_READ |
                              BLE_GATT_CHR_F_WRITE |
                              BLE_GATT_CHR_F_WRITE_NO_RSP |
                              BLE_GATT_CHR_F_NOTIFY,
            },
            {0}
        }
    },
    {0}
};

//-------------------- GAP events ---------------------------------------
static int gap_cb(struct ble_gap_event *e, void *arg)
{
    switch (e->type) {

    case BLE_GAP_EVENT_CONNECT:        // CONEXION
        if (e->connect.status == 0) 
        {
            conn_h = e->connect.conn_handle;
            printf("Conectado, handle=%d\n", conn_h);
        } else {
            printf("Fallo de conexión, reintentando...\n");
            conn_h = BLE_HS_CONN_HANDLE_NONE;

            struct ble_gap_adv_params adv = {0};
            adv.conn_mode = BLE_GAP_CONN_MODE_UND;
            adv.disc_mode = BLE_GAP_DISC_MODE_GEN;

            ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                              &adv, gap_cb, NULL);
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:       // DESCONEXION
        printf("Desconectado\n");
        conn_h = BLE_HS_CONN_HANDLE_NONE;
        notify_en = false;

        struct ble_gap_adv_params adv = {0};
        adv.conn_mode = BLE_GAP_CONN_MODE_UND;
        adv.disc_mode = BLE_GAP_DISC_MODE_GEN;

        ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                          &adv, gap_cb, NULL);
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:         // SUSCRIPCIÓN A NOTIFY
        notify_en = e->subscribe.cur_notify;
        printf("Notify=%d\n", notify_en);
        break;

    default:
        break;
    }

    return 0;
}

//-------------------------- Advertising -----------------------------
static void adv_start(void)
{
    struct ble_hs_adv_fields f = {0};
    f.flags = BLE_HS_ADV_F_DISC_GEN |       // visible por otros  
              BLE_HS_ADV_F_BREDR_UNSUP;     // solo BLE (no Bluetooth classic)

    const char *name = "ESP32-S3";          // nombre del dispositivo
    f.name = (uint8_t *)name;
    f.name_len = strlen(name);
    f.name_is_complete = 1;

    f.uuids16 = (ble_uuid16_t[]) { BLE_UUID16_INIT(SPP_SVC_UUID) }; // nombre 
                                                                    // que se va a ver

    f.num_uuids16 = 1;
    f.uuids16_is_complete = 1;

    ble_gap_adv_set_fields(&f);

    struct ble_gap_adv_params adv = {0};
    adv.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(own_addr_type, NULL,   // inicia advertising
                      BLE_HS_FOREVER,
                      &adv, gap_cb, NULL);

    printf("Advertising iniciado\n");
}

//------------Sincronización del Host BLE ---------------------------
static void on_sync(void)
{
    ble_hs_id_infer_auto(0, &own_addr_type);
    adv_start();
}

//-------------------- Tarea NimBLE ---------------------------------
static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}
//-------------------- Envío de datos TX ----------------------------
static void send_spp(const char *msg)
{
    if (!notify_en || conn_h == BLE_HS_CONN_HANDLE_NONE) return;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(msg, strlen(msg));
    if (!om) return;

    ble_gatts_notify_custom(conn_h, chr_h, om);
}
/*-------------------- MAIN --------------------*/
void app_main(void)
{
    nvs_flash_init();           // inicializa memoria
    nimble_port_init();         // inicializa nimble

    // configura el host BLE
    ble_hs_cfg.sync_cb         = on_sync;
    ble_hs_cfg.reset_cb        = NULL;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // Servicios estándar
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Registra el servicio SPP
    ble_gatts_count_cfg(svcs);
    ble_gatts_add_svcs(svcs);

    // Almacena claves
    ble_store_config_init();

    // arranca NimBLE
    nimble_port_freertos_init(host_task);

    while (1) 
    {
        // cada dos segundos envía un mensaje si hay cliente suscrito
        send_spp("Hola BLE\n");
        vTaskDelay(pdMS_TO_TICKS(2000));  
    }
}


