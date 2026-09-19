#include <errno.h>
#include "lwip/sockets.h"
#include "lwip/tcp.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "driver/temperature_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs.h"
#include "websocket.h"
#include "BMP280.h"
#include "BNO086.h"
#include "GPS.h"
#include "config.h"

#if DATALOGGER_ENABLED
#include "DataLogger.h"
#endif

/*
 * El ground track GPS sólo se considera un heading útil por encima de esta
 * velocidad. Por debajo se mantiene el último heading válido.
 */
static const char *TAG = "WEBSOCKET";

/* Buzón de una muestra: el productor sobrescribe datos aún no consumidos. */
static QueueHandle_t s_imu_queue = NULL;

/* Sensor térmico interno empleado únicamente como diagnóstico. */
static temperature_sensor_handle_t s_temp_sensor = NULL;

/* Impide crear más de una tarea de telemetría para este módulo. */
static TaskHandle_t s_dummy_task = NULL;
/* Protege en conjunto los parámetros modificables desde la interfaz web. */
portMUX_TYPE s_settings_mux = portMUX_INITIALIZER_UNLOCKED;

static int32_t s_pitch_offset_deg = PITCH_OFFSET_DEFAULT_DEG;
static uint32_t s_heading_offset_deg = HEADING_OFFSET_DEFAULT_DEG;
static uint32_t s_mount_mode = (uint32_t)BNO086_MOUNT_VERTICAL;
static uint32_t s_imu_mode = 0U;
static uint32_t s_attitude_mode = 0U;
/* Se incrementa con cada cambio para provocar el envío de una trama C. */
static uint32_t s_cfg_version = 1U;
#if DATALOGGER_ENABLED
/* Versión independiente para transmitir el estado L del registrador. */
static uint32_t s_logger_version = 1U;
#endif

/* Último heading GPS válido. Sólo RAM: al arrancar comienza en 0 deg. */
static float s_last_gps_heading_deg = 0.0f;
static bool s_last_gps_heading_valid = false;

/* QNH compartido con la tarea barométrica mediante websocket_get_qnh(). */
float s_qnh_hpa = QNH_DEFAULT_HPA;

typedef struct
{
    /* Servidor y copia autocontenida de la trama que ejecutará httpd. */
    httpd_handle_t server;
    char json[JSON_BUFFER_SIZE];
} websocket_work_t;

/*
 * Sólo puede existir un trabajo rápido pendiente. Si llega otra muestra antes
 * de enviarlo, reemplaza a la anterior: la pantalla recibe siempre la más
 * reciente y no se llena la cola de control del servidor HTTP.
 */
static portMUX_TYPE s_fast_work_mux = portMUX_INITIALIZER_UNLOCKED;
static websocket_work_t s_fast_work;
static bool s_fast_work_queued = false;
static uint32_t s_fast_work_version = 0U;

typedef struct
{
    /* Instantánea coherente de los ajustes protegidos por s_settings_mux. */
    float qnh_hpa;
    int32_t pitch_offset_deg;
    uint32_t heading_offset_deg;
    uint32_t mount_mode;
    uint32_t imu_mode;
    uint32_t attitude_mode;
    uint32_t cfg_version;
#if DATALOGGER_ENABLED
    uint32_t logger_version;
#endif
} settings_snapshot_t;

/** Obtiene atómicamente los ajustes que se incluirán en telemetría. */
static settings_snapshot_t settings_get_snapshot(void)
{
    settings_snapshot_t snapshot;

    portENTER_CRITICAL(&s_settings_mux);
    snapshot.qnh_hpa = s_qnh_hpa;
    snapshot.pitch_offset_deg = s_pitch_offset_deg;
    snapshot.heading_offset_deg = s_heading_offset_deg;
    snapshot.mount_mode = s_mount_mode;
    snapshot.imu_mode = s_imu_mode;
    snapshot.attitude_mode = s_attitude_mode;
    snapshot.cfg_version = s_cfg_version;
#if DATALOGGER_ENABLED
    snapshot.logger_version = s_logger_version;
#endif
    portEXIT_CRITICAL(&s_settings_mux);

    return snapshot;
}
//----------------------------------------------------------------------------------
/** Invalida la versión enviada y fuerza una nueva trama de configuración. */
static void settings_mark_changed(void)
{
    portENTER_CRITICAL(&s_settings_mux);
    ++s_cfg_version;
    portEXIT_CRITICAL(&s_settings_mux);
}
#if DATALOGGER_ENABLED
//----------------------------------------------------------------------------------
/** Fuerza el envío de una trama L con el estado actualizado del datalogger. */
static void logger_mark_changed(void)
{
    portENTER_CRITICAL(&s_settings_mux);
    ++s_logger_version;
    portEXIT_CRITICAL(&s_settings_mux);
}
#endif
//----------------------------------------------------------------------------------
/** Guarda un entero con signo y confirma inmediatamente la transacción NVS. */
static esp_err_t settings_save_i32(const char *key, int32_t value)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo abrir NVS: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_i32(handle, key, value);

    if (err == ESP_OK)
        err = nvs_commit(handle);

    nvs_close(handle);

    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error guardando %s en NVS: %s", key, esp_err_to_name(err));

    return err;
}
//----------------------------------------------------------------------------------
/** Guarda un entero sin signo y confirma inmediatamente la transacción NVS. */
static esp_err_t settings_save_u32(const char *key, uint32_t value)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo abrir NVS: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_u32(handle, key, value);

    if (err == ESP_OK)
        err = nvs_commit(handle);

    nvs_close(handle);

    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error guardando %s en NVS: %s", key, esp_err_to_name(err));

    return err;
}
//----------------------------------------------------------------------------------
/** Recupera NVS, limita los valores y aplica el montaje guardado al BNO086. */
static void settings_load(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "NVS sin configuración previa; se usan valores por defecto");
        return;
    }

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "No se pudo leer NVS: %s", esp_err_to_name(err));
        return;
    }

    /* El QNH se almacena escalado para no persistir directamente un float. */
    int32_t qnh_x100 = (int32_t)lroundf(QNH_DEFAULT_HPA * 100.0f);
    int32_t pitch_offset = PITCH_OFFSET_DEFAULT_DEG;
    uint32_t heading_offset = HEADING_OFFSET_DEFAULT_DEG;
    uint32_t mount_mode = (uint32_t)BNO086_MOUNT_VERTICAL;
    uint32_t imu_mode = 0U;

    (void)nvs_get_i32(handle, NVS_KEY_QNH_X100, &qnh_x100);
    (void)nvs_get_i32(handle, NVS_KEY_PITCH_OFFSET, &pitch_offset);
    (void)nvs_get_u32(handle, NVS_KEY_HEAD_OFFSET, &heading_offset);
    (void)nvs_get_u32(handle, NVS_KEY_MOUNT_MODE, &mount_mode);
    (void)nvs_get_u32(handle, NVS_KEY_IMU_MODE, &imu_mode);
    uint32_t attitude_mode = ATTITUDE_DEFAULT_MODE;
    (void)nvs_get_u32(handle, NVS_KEY_ATTITUDE_MODE, &attitude_mode);

    nvs_close(handle);

    float qnh = (float)qnh_x100 / 100.0f;

    if ((qnh < QNH_MIN_HPA) || (qnh > QNH_MAX_HPA))
        qnh = QNH_DEFAULT_HPA;

    if (pitch_offset < PITCH_OFFSET_MIN_DEG)
        pitch_offset = PITCH_OFFSET_MIN_DEG;
    else if (pitch_offset > PITCH_OFFSET_MAX_DEG)
        pitch_offset = PITCH_OFFSET_MAX_DEG;

    heading_offset %= 360U;

    if (mount_mode > (uint32_t)BNO086_MOUNT_HORIZONTAL)
    {
        mount_mode = (uint32_t)BNO086_MOUNT_VERTICAL;
    }
    /*
     * El BNO086 usa siempre Game Rotation Vector para actitud.
     * Los antiguos modos AMG/IMUPLUS/NDOF y los seis modos experimentales
     * ya no aplican.
     */
    imu_mode = 0U;
    attitude_mode = 0U;

    if (mount_mode > (uint32_t)BNO086_MOUNT_HORIZONTAL)
    {
        mount_mode = (uint32_t)BNO086_MOUNT_VERTICAL;
    }

    esp_err_t mount_err =
        BNO086_set_mount_mode((bno086_mount_mode_t)mount_mode);

    if (mount_err != ESP_OK)
    {
        ESP_LOGW(TAG,
                 "No se pudo recuperar el modo de montaje: %s",
                 esp_err_to_name(mount_err));
        mount_mode = (uint32_t)BNO086_MOUNT_VERTICAL;
    }

    portENTER_CRITICAL(&s_settings_mux);
    s_qnh_hpa = qnh;
    s_pitch_offset_deg = pitch_offset;
    s_heading_offset_deg = heading_offset;
    s_mount_mode = mount_mode;
    s_imu_mode = imu_mode;
    s_attitude_mode = attitude_mode;
    portEXIT_CRITICAL(&s_settings_mux);

    ESP_LOGI(
        TAG,
        "Configuración recuperada: QNH=%.2f hPa, pitch offset=%" PRId32
        " deg, heading offset=%" PRIu32 " deg, montaje=%s, IMU=BNO086/GAME_RV",
        (double)qnh,
        pitch_offset,
        heading_offset,
        mount_mode == (uint32_t)BNO086_MOUNT_HORIZONTAL ? "H" : "V");
}
//----------------------------------------------------------------------------------
/** Aplica, limita, persiste y registra un incremento de QNH. */
static void qnh_change(float increment_hpa)
{
    float new_value;

    portENTER_CRITICAL(&s_settings_mux);

    s_qnh_hpa += increment_hpa;

    if (s_qnh_hpa < QNH_MIN_HPA)
    {
        s_qnh_hpa = QNH_MIN_HPA;
    }
    else if (s_qnh_hpa > QNH_MAX_HPA)
    {
        s_qnh_hpa = QNH_MAX_HPA;
    }

    new_value = s_qnh_hpa;
    portEXIT_CRITICAL(&s_settings_mux);

    const int32_t qnh_x100 = (int32_t)lroundf(new_value * 100.0f);
    (void)settings_save_i32(NVS_KEY_QNH_X100, qnh_x100);

    ESP_LOGI(TAG, "QNH actualizado: %.2f hPa", (double)new_value);
}
//----------------------------------------------------------------------------------
/** Modifica el cero visual de pitch dentro del intervalo configurado. */
static void pitch_offset_change(int32_t increment_deg)
{
    int32_t new_value;

    portENTER_CRITICAL(&s_settings_mux);

    s_pitch_offset_deg += increment_deg;

    if (s_pitch_offset_deg < PITCH_OFFSET_MIN_DEG)
    {
        s_pitch_offset_deg = PITCH_OFFSET_MIN_DEG;
    }
    else if (s_pitch_offset_deg > PITCH_OFFSET_MAX_DEG)
    {
        s_pitch_offset_deg = PITCH_OFFSET_MAX_DEG;
    }

    new_value = s_pitch_offset_deg;
    portEXIT_CRITICAL(&s_settings_mux);

    (void)settings_save_i32(NVS_KEY_PITCH_OFFSET, new_value);

    ESP_LOGI(TAG, "Offset de pitch actualizado: %" PRId32 " deg", new_value);
}
//----------------------------------------------------------------------------------
/** Modifica el curso manual y lo normaliza al intervalo [0, 360). */
static void heading_offset_change(int32_t increment_deg)
{
    uint32_t new_value;

    portENTER_CRITICAL(&s_settings_mux);

    int32_t value = (int32_t)s_heading_offset_deg + increment_deg;

    while (value < 0)
    {
        value += 360;
    }

    while (value >= 360)
    {
        value -= 360;
    }

    s_heading_offset_deg = (uint32_t)value;
    new_value = s_heading_offset_deg;

    portEXIT_CRITICAL(&s_settings_mux);

    (void)settings_save_u32(NVS_KEY_HEAD_OFFSET, new_value);

    ESP_LOGI(TAG, "Heading manual actualizado: %" PRIu32 " deg", new_value);
}
//----------------------------------------------------------------------------------
/** Aplica el montaje V/H al BNO086 y lo conserva en NVS. */
static void mount_mode_change(bno086_mount_mode_t mode)
{
    esp_err_t err = BNO086_set_mount_mode(mode);

    if (err != ESP_OK)
    {
        ESP_LOGW(
            TAG,
            "No se pudo cambiar el modo de montaje: %s",
            esp_err_to_name(err));
        return;
    }

    portENTER_CRITICAL(&s_settings_mux);
    s_mount_mode = (uint32_t)mode;
    portEXIT_CRITICAL(&s_settings_mux);

    (void)settings_save_u32(
        NVS_KEY_MOUNT_MODE,
        (uint32_t)mode);

    ESP_LOGI(
        TAG,
        "Modo de montaje actualizado: %s",
        mode == BNO086_MOUNT_HORIZONTAL ? "H" : "V");
}
//----------------------------------------------------------------------------------
/** Reinicia los extremos retenidos por el G-meter. */
static void g_peak_reset(void)
{
    BNO086_reset_accel_peaks();
    ESP_LOGI(TAG, "Indicador de G reseteado: min/max = G actual");
}
//----------------------------------------------------------------------------------
/**
 * Atiende el handshake GET y las tramas de control. Activa TCP_NODELAY antes
 * de devolver el socket al flujo normal del WebSocket.
 */
static esp_err_t websocket_handler(httpd_req_t *req)
{
    /* Descriptor TCP correspondiente a esta sesión HTTP/WebSocket. */
    const int fd = httpd_req_to_sockfd(req);
    const int enable = 1;

    /*
     * Desactivar el algoritmo de Nagle.
     * La opción permanece activa durante toda la vida del socket.
     */
    if (setsockopt(
            fd,
            IPPROTO_TCP,
            TCP_NODELAY,
            &enable,
            sizeof(enable)) < 0)
    {
        ESP_LOGW(
            TAG,
            "No se pudo activar TCP_NODELAY: fd=%d errno=%d",
            fd,
            errno);
    }

    /*
     * Petición inicial de establecimiento del WebSocket.
     */
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(
            TAG,
            "Cliente WebSocket conectado: fd=%d, TCP_NODELAY activo",
            fd);

        return ESP_OK;
    }

    /*
     * Obtener primero el tamaño y el tipo de la trama recibida.
     */
    httpd_ws_frame_t frame = {0};

    esp_err_t err =
        httpd_ws_recv_frame(req, &frame, 0U);

    if (err != ESP_OK)
    {
        ESP_LOGW(
            TAG,
            "Error obteniendo trama WebSocket: %s",
            esp_err_to_name(err));

        return err;
    }

    /*
     * Las tramas sin carga útil no contienen comandos.
     */
    if (frame.len == 0U)
    {
        return ESP_OK;
    }

    /*
     * Los comandos del navegador se reciben como texto.
     */
    if (frame.type != HTTPD_WS_TYPE_TEXT)
    {
        ESP_LOGW(
            TAG,
            "Trama WebSocket ignorada: tipo=%d",
            (int)frame.type);

        return ESP_OK;
    }

    /* Se reserva un byte adicional para el terminador de la cadena C. */
    uint8_t *payload =
        calloc(1U, frame.len + 1U);

    if (payload == NULL)
    {
        ESP_LOGE(
            TAG,
            "Sin memoria para recibir trama WebSocket");

        return ESP_ERR_NO_MEM;
    }

    frame.payload = payload;

    err = httpd_ws_recv_frame(
        req,
        &frame,
        frame.len);

    if (err != ESP_OK)
    {
        ESP_LOGW(
            TAG,
            "Error recibiendo trama WebSocket: %s",
            esp_err_to_name(err));

        free(payload);
        return err;
    }

    /*
     * calloc() ha reservado un byte adicional para el terminador.
     */
    payload[frame.len] = '\0';

    ESP_LOGI(
        TAG,
        "Mensaje del navegador: %s",
        (char *)payload);

    if (strcmp((char *)payload, "QNH_UP") == 0)
    {
        qnh_change(QNH_STEP_HPA);
        settings_mark_changed();
    }
    else if (strcmp((char *)payload, "QNH_DOWN") == 0)
    {
        qnh_change(-QNH_STEP_HPA);
        settings_mark_changed();
    }
    else if (strcmp((char *)payload, "PITCH_OFFSET_UP") == 0)
    {
        pitch_offset_change(PITCH_OFFSET_STEP_DEG);
        settings_mark_changed();
    }
    else if (strcmp((char *)payload, "PITCH_OFFSET_DOWN") == 0)
    {
        pitch_offset_change(-PITCH_OFFSET_STEP_DEG);
        settings_mark_changed();
    }
    else if (strcmp((char *)payload, "HEADING_OFFSET_UP") == 0)
    {
        heading_offset_change(
            (int32_t)HEADING_OFFSET_STEP_DEG);

        settings_mark_changed();
    }
    else if (strcmp((char *)payload, "HEADING_OFFSET_DOWN") == 0)
    {
        heading_offset_change(
            -(int32_t)HEADING_OFFSET_STEP_DEG);

        settings_mark_changed();
    }
    else if (strcmp((char *)payload, "MOUNT_MODE_V") == 0)
    {
        mount_mode_change(
            BNO086_MOUNT_VERTICAL);

        settings_mark_changed();
    }
    else if (strcmp((char *)payload, "MOUNT_MODE_H") == 0)
    {
        mount_mode_change(
            BNO086_MOUNT_HORIZONTAL);

        settings_mark_changed();
    }
#if DATALOGGER_ENABLED
    else if (strcmp((char *)payload, "LOGGER_START") == 0)
    {
        const esp_err_t logger_err = DataLogger_begin_recording();

        if (logger_err != ESP_OK)
        {
            ESP_LOGW(TAG,
                     "No se pudo iniciar el datalogger: %s",
                     esp_err_to_name(logger_err));
        }

        logger_mark_changed();
    }
    else if (strcmp((char *)payload, "LOGGER_STOP") == 0)
    {
        const esp_err_t logger_err = DataLogger_stop_recording();

        if (logger_err != ESP_OK)
        {
            ESP_LOGW(TAG,
                     "No se pudo detener el datalogger: %s",
                     esp_err_to_name(logger_err));
        }

        logger_mark_changed();
    }
#endif
    else if (strcmp((char *)payload, "G_RESET") == 0)
    {
        g_peak_reset();
    }
    else if (strcmp((char *)payload, "CFG_GET") == 0)
    {
        /*
         * Fuerza el envío de una trama de configuración
         * en el siguiente ciclo rápido.
         */
        settings_mark_changed();
#if DATALOGGER_ENABLED
        logger_mark_changed();
#endif
    }
    else
    {
        ESP_LOGW(
            TAG,
            "Comando WebSocket desconocido: %s",
            (char *)payload);
    }

    free(payload);

    return ESP_OK;
}
//----------------------------------------------------------------------------------
/** Envía texto a todos los sockets que siguen siendo clientes WebSocket. */
static void websocket_broadcast_json(httpd_handle_t server, const char *json)
{
    if ((server == NULL) || (json == NULL))
    {
        return;
    }

    int client_fds[MAX_WS_CLIENTS];
    size_t client_count = MAX_WS_CLIENTS;

    if (httpd_get_client_list(server,
                              &client_count,
                              client_fds) == ESP_OK)
    {
        httpd_ws_frame_t frame = {
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)json,
            .len = strlen(json),
        };

        for (size_t i = 0; i < client_count; ++i)
        {
            const int fd = client_fds[i];

            if (httpd_ws_get_fd_info(server, fd) ==
                HTTPD_WS_CLIENT_WEBSOCKET)
            {
                esp_err_t err = httpd_ws_send_frame_async(
                    server,
                    fd,
                    &frame);

                if (err != ESP_OK)
                {
                    ESP_LOGD(TAG,
                             "Error enviando a fd=%d: %s",
                             fd,
                             esp_err_to_name(err));
                }
            }
        }
    }
}

//----------------------------------------------------------------------------------
/** Trabajo httpd para una trama lenta; libera su copia al terminar. */
static void websocket_broadcast_work(void *arg)
{
    websocket_work_t *work = arg;

    if (work == NULL)
    {
        return;
    }

    websocket_broadcast_json(work->server, work->json);

    free(work);
}

//----------------------------------------------------------------------------------
/**
 * Trabajo de baja latencia: envía la versión copiada y vuelve a programarse
 * únicamente si el productor la sustituyó mientras se estaba transmitiendo.
 */
static void websocket_fast_broadcast_work(void *arg)
{
    (void)arg;

    /* Copia local para no retener el spinlock durante la operación de red. */
    websocket_work_t work;
    uint32_t sent_version;

    portENTER_CRITICAL(&s_fast_work_mux);
    work = s_fast_work;
    sent_version = s_fast_work_version;
    portEXIT_CRITICAL(&s_fast_work_mux);

    websocket_broadcast_json(work.server, work.json);

    bool requeue = false;

    portENTER_CRITICAL(&s_fast_work_mux);

    if (s_fast_work_version == sent_version)
    {
        s_fast_work_queued = false;
    }
    else
    {
        requeue = true;
    }

    portEXIT_CRITICAL(&s_fast_work_mux);

    if (requeue &&
        (httpd_queue_work(work.server,
                          websocket_fast_broadcast_work,
                          NULL) != ESP_OK))
    {
        portENTER_CRITICAL(&s_fast_work_mux);
        s_fast_work_queued = false;
        portEXIT_CRITICAL(&s_fast_work_mux);
    }
}
//----------------------------------------------------------------------------------
/** Cuenta solamente las sesiones que completaron el upgrade WebSocket. */
static size_t websocket_count_clients(httpd_handle_t server)
{
    if (server == NULL)
    {
        return 0U;
    }

    int client_fds[MAX_WS_CLIENTS];
    size_t client_count = MAX_WS_CLIENTS;

    if (httpd_get_client_list(server, &client_count, client_fds) != ESP_OK)
    {
        return 0U;
    }

    size_t websocket_clients = 0U;

    for (size_t i = 0; i < client_count; ++i)
    {
        if (httpd_ws_get_fd_info(server, client_fds[i]) ==
            HTTPD_WS_CLIENT_WEBSOCKET)
        {
            ++websocket_clients;
        }
    }

    return websocket_clients;
}

//----------------------------------------------------------------------------------
/** Instala y habilita el sensor térmico interno de diagnóstico. */
static esp_err_t internal_temperature_init(void)
{
    temperature_sensor_config_t config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(INTERNAL_TEMP_MIN_C, INTERNAL_TEMP_MAX_C);

    esp_err_t err = temperature_sensor_install(&config, &s_temp_sensor);

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "No se pudo instalar el sensor interno de temperatura: %s", esp_err_to_name(err));
        s_temp_sensor = NULL;
        return err;
    }

    err = temperature_sensor_enable(s_temp_sensor);

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG,
                 "No se pudo habilitar el sensor interno de temperatura: %s",
                 esp_err_to_name(err));
        temperature_sensor_uninstall(s_temp_sensor);
        s_temp_sensor = NULL;
        return err;
    }

    return ESP_OK;
}

//----------------------------------------------------------------------------------

/** Registra la temperatura interna cuando el sensor está disponible. */
static void internal_temperature_log(void)
{

    if (s_temp_sensor == NULL)
    {
        return;
    }

    float temperature_c = 0.0f;

    esp_err_t err = temperature_sensor_get_celsius(s_temp_sensor, &temperature_c);

    if (err == ESP_OK)
        ESP_LOGI(TAG, "Temperatura interna ESP32-S3: %.1f C", (double)temperature_c);
    else
        ESP_LOGW(TAG, "Error leyendo temperatura interna: %s", esp_err_to_name(err));
}

//----------------------------------------------------------------------------------
/** Formatea y agenda una trama lenta independiente de navegación/configuración. */
static void websocket_queue_json(httpd_handle_t server, const char *format, ...)
{
    websocket_work_t *work = calloc(1U, sizeof(*work));

    if (work == NULL)
        return;

    work->server = server;

    va_list args;
    va_start(args, format);
    vsnprintf(work->json, sizeof(work->json), format, args);
    va_end(args);

    esp_err_t err = httpd_queue_work(server, websocket_broadcast_work, work);

    if (err != ESP_OK)
        free(work);
}

//----------------------------------------------------------------------------------
/**
 * Actualiza el único slot rápido. Si ya hay un trabajo pendiente, reemplaza
 * sus datos en vez de acumular otra muestra de actitud.
 */
static void websocket_queue_fast_json(httpd_handle_t server, const char *format, ...)
{
    char json[JSON_BUFFER_SIZE] = {0};

    va_list args;
    va_start(args, format);
    vsnprintf(json, sizeof(json), format, args);
    va_end(args);

    bool queue_work = false;

    portENTER_CRITICAL(&s_fast_work_mux);
    s_fast_work.server = server;
    memcpy(s_fast_work.json, json, sizeof(s_fast_work.json));
    ++s_fast_work_version;

    if (!s_fast_work_queued)
    {
        s_fast_work_queued = true;
        queue_work = true;
    }

    portEXIT_CRITICAL(&s_fast_work_mux);

    if (queue_work &&
        (httpd_queue_work(server,
                          websocket_fast_broadcast_work,
                          NULL) != ESP_OK))
    {
        portENTER_CRITICAL(&s_fast_work_mux);
        s_fast_work_queued = false;
        portEXIT_CRITICAL(&s_fast_work_mux);
    }
}

/**
 * Consume la IMU y produce F al recibir muestra, N a 2 Hz y C al cambiar los
 * ajustes. El diseño prioriza siempre el dato más reciente.
 */
static void telemetry_task(void *arg)
{
    httpd_handle_t server = (httpd_handle_t)arg;

    bno086_data_t imu = {0};

    /* Navegación, barometría y GPS no requieren la frecuencia de la actitud. */
    const TickType_t navigation_period = pdMS_TO_TICKS(500U); /* 2 Hz */

    TickType_t last_navigation_time = xTaskGetTickCount();

    uint32_t last_cfg_version = 0U;
#if DATALOGGER_ENABLED
    uint32_t last_logger_version = 0U;
    TickType_t last_logger_status_time = xTaskGetTickCount();
#endif

    for (;;)
    {
        /*
         * Esperar una muestra nueva del BNO086.
         *
         * Al tratarse de una cola de longitud 1 con xQueueOverwrite(),
         * si el consumidor se retrasa recibirá siempre la muestra más
         * reciente, no una muestra antigua.
         */
        const BaseType_t sample_received = xQueueReceive(s_imu_queue, &imu, pdMS_TO_TICKS(100U));

        /*
         * Aunque no haya una muestra nueva, el timeout permite comprobar
         * cambios de configuración y mantener operativa la tarea.
         */
        if (websocket_count_clients(server) == 0U)
        {
            continue;
        }

        /* Instantáneas locales coherentes durante todo el formateo JSON. */
        const gps_data_t gps = GPS_get_data();

        const settings_snapshot_t settings = settings_get_snapshot();

        /*
         * Enviar F únicamente cuando se haya recibido una muestra
         * nueva del BNO086.
         */
        if (sample_received == pdPASS)
        {
            const float roll = imu.valid ? imu.roll_deg : 0.0f;

            const float pitch = imu.valid ? imu.pitch_deg : 0.0f;

            /* No se acepta Ground Track mientras el vehículo casi no se mueve. */
            const bool gps_heading_sample_valid =
                gps.fix_valid &&
                isfinite(gps.ground_track_deg) &&
                isfinite(gps.ground_speed_knots) &&
                (gps.ground_speed_knots >=
                 GPS_HEADING_MIN_SPEED_KT);

            if (gps_heading_sample_valid)
            {
                float heading = gps.ground_track_deg;

                while (heading < 0.0f)
                {
                    heading += 360.0f;
                }

                while (heading >= 360.0f)
                {
                    heading -= 360.0f;
                }

                s_last_gps_heading_deg = heading;
                s_last_gps_heading_valid = true;
            }

#if HEADING_GPS
            const float yaw =
                s_last_gps_heading_valid
                    ? s_last_gps_heading_deg
                    : 0.0f;
#else
            const float yaw =
                imu.valid
                    ? imu.heading_deg
                    : 0.0f;
#endif

            const float turn_rate_dps = imu.valid ? imu.yaw_rate_dps : 0.0f;

            const float slip_ball_deg = imu.valid ? imu.slip_ball_deg : 0.0f;

            const float g_current = imu.valid ? imu.g_current : 0.0f;

            const float g_max =
                imu.valid
                    ? imu.g_max
                    : 0.0f;

            const float g_min =
                imu.valid
                    ? imu.g_min
                    : 0.0f;

            /*
             * F: datos dinámicos enviados inmediatamente después
             * de recibir una muestra nueva.
             */
            /* F reemplaza cualquier actitud que siga pendiente de envío. */
            websocket_queue_fast_json(
                server,
                "[\"F\",%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f]",
                -(double)roll,
                (double)pitch,
                (double)yaw,
                (double)turn_rate_dps,
                (double)slip_ball_deg,
                (double)g_current,
                (double)g_max,
                (double)g_min);
        }

        /*
         * N: navegación y altimetría a 2 Hz, controlada por tiempo.
         */
        const TickType_t now = xTaskGetTickCount();

        if ((now - last_navigation_time) >=
            navigation_period)
        {
            /* Evita ráfagas de recuperación después de una desconexión. */
            last_navigation_time = now;

            /* N transporta variables lentas y el timestamp UTC compacto. */
            websocket_queue_json(
                server,
                "[\"N\",%.1f,%.2f,%.1f,%u,"
                "%.1f,%.6f,%.6f,%.2f,%" PRIu32 "]",
                (double)altitude,
                (double)vertical_speed,
                (double)temperature,
                gps.fix_valid ? 1U : 0U,
                (double)(gps.fix_valid
                             ? gps.altitude_m
                             : 0.0f),
                gps.fix_valid
                    ? gps.latitude_deg
                    : 0.0,
                gps.fix_valid
                    ? gps.longitude_deg
                    : 0.0,
                (double)(gps.fix_valid
                             ? gps.ground_speed_knots
                             : 0.0f),
                gps.utc_timestamp);
        }

        /*
         * C: enviar solamente cuando cambia la configuración.
         */
        if (settings.cfg_version != last_cfg_version)
        {
            last_cfg_version = settings.cfg_version;

            /* C solo se genera al conectar o después de cambiar un ajuste. */
            websocket_queue_json(
                server,
                "[\"C\",%.1f,%" PRId32
                ",%" PRIu32 ",%" PRIu32 "]",
                (double)settings.qnh_hpa,
                settings.pitch_offset_deg,
                settings.heading_offset_deg,
                settings.mount_mode);
        }

#if DATALOGGER_ENABLED
        const datalogger_status_t logger = DataLogger_get_status();

        /* L: al cambiar de estado y cada 500 ms mientras está grabando. */
        if ((settings.logger_version != last_logger_version) ||
            (logger.recording &&
             ((now - last_logger_status_time) >= pdMS_TO_TICKS(500U))))
        {
            last_logger_version = settings.logger_version;
            last_logger_status_time = now;

            websocket_queue_json(
                server,
                "[\"L\",%u,%" PRIu32 ",%" PRIu32 ",%u,%" PRIu32 "]",
                logger.recording ? 1U : 0U,
                logger.samples,
                logger.capacity,
                logger.wrapped ? 1U : 0U,
                logger.total_samples);
        }
#endif
    }
}
//----------------------------------------------------------------------------------
/** Registra `/ws` como endpoint GET con upgrade WebSocket. */
esp_err_t websocket_register_uri(httpd_handle_t server)
{
    if (server == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const httpd_uri_t uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = websocket_handler,
        .user_ctx = NULL,
        .is_websocket = true,
    };

    return httpd_register_uri_handler(server, &uri);
}
//----------------------------------------------------------------------------------
/**
 * Crea el buzón IMU, lo registra en el BNO086, recupera NVS y arranca la tarea
 * encargada de construir las tramas de telemetría.
 */
esp_err_t websocket_start_dummy_stream(httpd_handle_t server)
{
    if (server == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_dummy_task != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    /* Longitud uno: xQueueOverwrite conserva siempre la muestra más reciente. */
    s_imu_queue = xQueueCreate(1U, sizeof(bno086_data_t));

    if (s_imu_queue == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    BNO086_set_output_queue(s_imu_queue);

    settings_load();

    /*
     * Sensor térmico interno del ESP32-S3.
     * Se usa únicamente como diagnóstico y se imprime una vez por segundo.
     */
    (void)internal_temperature_init();

    BaseType_t ok = xTaskCreate(
        telemetry_task,
        "telemetry",
        TELEMETRY_TASK_STACK_SIZE,
        server,
        TELEMETRY_TASK_PRIORITY,
        &s_dummy_task);

    if (ok != pdPASS)
    {
        BNO086_set_output_queue(NULL);
        vQueueDelete(s_imu_queue);
        s_imu_queue = NULL;
        s_dummy_task = NULL;

        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG,
             "Telemetría sincronizada con BNO086; navegación a 2 Hz");

    return ESP_OK;
}
//----------------------------------------------------------------------------------
/** Entrega al BMP280 una copia atómica del QNH seleccionado. */
float websocket_get_qnh(void)
{
    float qnh;

    portENTER_CRITICAL(&s_settings_mux);
    qnh = s_qnh_hpa;
    portEXIT_CRITICAL(&s_settings_mux);

    return qnh;
}
