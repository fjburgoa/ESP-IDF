#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/temperature_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "websocket.h"
#include "BMP280.h"
#include "BNO055.h"
#include "GPS.h"
#include "config.h"

#if DATALOGGER_ENABLED
#include "DataLogger.h"
#endif

/* Añadir también a BMP280.h cuando se consolide la interfaz del driver. */
extern float vertical_speed;

#define TELEMETRY_PERIOD_MS 100U // 10 Hz
#define MAX_WS_CLIENTS 4U
#define JSON_BUFFER_SIZE 1600U

#define QNH_STEP_HPA 0.05f
#define QNH_MIN_HPA 970.00f
#define QNH_MAX_HPA 1150.00f
#define QNH_DEFAULT_HPA 1013.25f

#define PITCH_OFFSET_DEFAULT_DEG 0
#define PITCH_OFFSET_STEP_DEG 1
#define PITCH_OFFSET_MIN_DEG -90
#define PITCH_OFFSET_MAX_DEG 90

#define HEADING_OFFSET_DEFAULT_DEG 0U
#define HEADING_OFFSET_STEP_DEG 1U
#define HEADING_OFFSET_MAX_DEG 359U

/*
 * El ground track GPS sólo se considera un heading útil por encima de esta
 * velocidad. Por debajo se mantiene el último heading válido.
 */
#define GPS_HEADING_MIN_SPEED_KT 5.0f

#define NVS_NAMESPACE "flight_cfg"
#define NVS_KEY_QNH_X100 "qnh_x100"
#define NVS_KEY_PITCH_OFFSET "pitch_off"
#define NVS_KEY_HEAD_OFFSET "head_off"
#define NVS_KEY_MOUNT_MODE "mount_mode"

static const char *TAG = "WEBSOCKET";

static temperature_sensor_handle_t s_temp_sensor = NULL;

static TaskHandle_t s_dummy_task = NULL;
portMUX_TYPE s_settings_mux = portMUX_INITIALIZER_UNLOCKED;

static int32_t s_pitch_offset_deg = PITCH_OFFSET_DEFAULT_DEG;
static uint32_t s_heading_offset_deg = HEADING_OFFSET_DEFAULT_DEG;
static uint32_t s_mount_mode = (uint32_t)BNO055_MOUNT_VERTICAL;

/* Último heading GPS válido. Sólo RAM: al arrancar comienza en 0 deg. */
static float s_last_gps_heading_deg = 0.0f;
static bool s_last_gps_heading_valid = false;

float s_qnh_hpa = QNH_DEFAULT_HPA;

typedef struct
{
    httpd_handle_t server;
    char json[JSON_BUFFER_SIZE];
} websocket_work_t;

typedef struct
{
    float qnh_hpa;
    int32_t pitch_offset_deg;
    uint32_t heading_offset_deg;
    uint32_t mount_mode;
} settings_snapshot_t;

static settings_snapshot_t settings_get_snapshot(void)
{
    settings_snapshot_t snapshot;

    portENTER_CRITICAL(&s_settings_mux);
    snapshot.qnh_hpa = s_qnh_hpa;
    snapshot.pitch_offset_deg = s_pitch_offset_deg;
    snapshot.heading_offset_deg = s_heading_offset_deg;
    snapshot.mount_mode = s_mount_mode;
    portEXIT_CRITICAL(&s_settings_mux);

    return snapshot;
}
//----------------------------------------------------------------------------------
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

    int32_t qnh_x100 = (int32_t)lroundf(QNH_DEFAULT_HPA * 100.0f);
    int32_t pitch_offset = PITCH_OFFSET_DEFAULT_DEG;
    uint32_t heading_offset = HEADING_OFFSET_DEFAULT_DEG;
    uint32_t mount_mode = (uint32_t)BNO055_MOUNT_VERTICAL;

    (void)nvs_get_i32(handle, NVS_KEY_QNH_X100, &qnh_x100);
    (void)nvs_get_i32(handle, NVS_KEY_PITCH_OFFSET, &pitch_offset);
    (void)nvs_get_u32(handle, NVS_KEY_HEAD_OFFSET, &heading_offset);
    (void)nvs_get_u32(handle, NVS_KEY_MOUNT_MODE, &mount_mode);

    nvs_close(handle);

    float qnh = (float)qnh_x100 / 100.0f;

    if ((qnh < QNH_MIN_HPA) || (qnh > QNH_MAX_HPA))
        qnh = QNH_DEFAULT_HPA;

    if (pitch_offset < PITCH_OFFSET_MIN_DEG)
        pitch_offset = PITCH_OFFSET_MIN_DEG;
    else if (pitch_offset > PITCH_OFFSET_MAX_DEG)
        pitch_offset = PITCH_OFFSET_MAX_DEG;

    heading_offset %= 360U;

    if (mount_mode > (uint32_t)BNO055_MOUNT_HORIZONTAL)
    {
        mount_mode = (uint32_t)BNO055_MOUNT_VERTICAL;
    }

    /*
     * Se aplica primero al BNO055 para que toda la fusión NDOF quede expresada
     * en los ejes lógicos del avión antes de publicar el modo en telemetría.
     */
    esp_err_t mount_err =
        BNO055_set_mount_mode((bno055_mount_mode_t)mount_mode);

    if (mount_err != ESP_OK)
    {
        ESP_LOGW(TAG,
                 "No se pudo recuperar el modo de montaje: %s",
                 esp_err_to_name(mount_err));
        mount_mode = (uint32_t)BNO055_MOUNT_VERTICAL;
    }

    portENTER_CRITICAL(&s_settings_mux);
    s_qnh_hpa = qnh;
    s_pitch_offset_deg = pitch_offset;
    s_heading_offset_deg = heading_offset;
    s_mount_mode = mount_mode;
    portEXIT_CRITICAL(&s_settings_mux);

    ESP_LOGI(TAG,
             "Configuración recuperada: QNH=%.2f hPa, pitch offset=%" PRId32
             " deg, heading offset=%" PRIu32 " deg, montaje=%s",
             (double)qnh,
             pitch_offset,
             heading_offset,
             mount_mode == (uint32_t)BNO055_MOUNT_HORIZONTAL ? "H" : "V");
}
//----------------------------------------------------------------------------------
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
static void mount_mode_change(bno055_mount_mode_t mode)
{
    esp_err_t err = BNO055_set_mount_mode(mode);

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG,
                 "No se pudo cambiar el modo de montaje: %s",
                 esp_err_to_name(err));
        return;
    }

    portENTER_CRITICAL(&s_settings_mux);
    s_mount_mode = (uint32_t)mode;
    portEXIT_CRITICAL(&s_settings_mux);

    (void)settings_save_u32(NVS_KEY_MOUNT_MODE, (uint32_t)mode);

    ESP_LOGI(TAG,
             "Modo de montaje actualizado: %s",
             mode == BNO055_MOUNT_HORIZONTAL ? "H" : "V");
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
static void g_peak_reset(void)
{
    BNO055_reset_accel_peaks();
    ESP_LOGI(TAG, "Indicador de G reseteado: min/max = G actual");
}
//----------------------------------------------------------------------------------
static esp_err_t websocket_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "Cliente WebSocket conectado, fd=%d", httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);

    if ((err != ESP_OK) || (frame.len == 0U))
    {
        return err;
    }

    uint8_t *payload = calloc(1U, frame.len + 1U);

    if (payload == NULL)
        return ESP_ERR_NO_MEM;

    frame.payload = payload;
    err = httpd_ws_recv_frame(req, &frame, frame.len);

    if (err == ESP_OK)
    {
        payload[frame.len] = '\0';
        ESP_LOGI(TAG, "Mensaje del navegador: %s", (char *)payload);

        if (strcmp((char *)payload, "QNH_UP") == 0)
        {
            qnh_change(QNH_STEP_HPA);
        }
        else if (strcmp((char *)payload, "QNH_DOWN") == 0)
        {
            qnh_change(-QNH_STEP_HPA);
        }
        else if (strcmp((char *)payload, "PITCH_OFFSET_UP") == 0)
        {
            pitch_offset_change(PITCH_OFFSET_STEP_DEG);
        }
        else if (strcmp((char *)payload, "PITCH_OFFSET_DOWN") == 0)
        {
            pitch_offset_change(-PITCH_OFFSET_STEP_DEG);
        }
        else if (strcmp((char *)payload, "HEADING_OFFSET_UP") == 0)
        {
            heading_offset_change((int32_t)HEADING_OFFSET_STEP_DEG);
        }
        else if (strcmp((char *)payload, "HEADING_OFFSET_DOWN") == 0)
        {
            heading_offset_change(-(int32_t)HEADING_OFFSET_STEP_DEG);
        }
        else if (strcmp((char *)payload, "MOUNT_MODE_V") == 0)
        {
            mount_mode_change(BNO055_MOUNT_VERTICAL);
        }
        else if (strcmp((char *)payload, "MOUNT_MODE_H") == 0)
        {
            mount_mode_change(BNO055_MOUNT_HORIZONTAL);
        }
        else if (strcmp((char *)payload, "G_RESET") == 0)
        {
            g_peak_reset();
        }

        else if (strcmp((char *)payload, "RECORD_START") == 0)
        {
#if DATALOGGER_ENABLED
            esp_err_t logger_err = DataLogger_begin_recording();
            if (logger_err != ESP_OK)
                ESP_LOGW(TAG, "No se pudo iniciar la grabación: %s", esp_err_to_name(logger_err));
#else
            ESP_LOGI(TAG, "RECORD_START ignorado: DataLogger desactivado");
#endif
        }
        else if (strcmp((char *)payload, "RECORD_STOP") == 0)
        {
#if DATALOGGER_ENABLED
            esp_err_t logger_err = DataLogger_stop_recording();
            if (logger_err != ESP_OK)
                ESP_LOGW(TAG, "No se pudo detener limpiamente la grabación: %s", esp_err_to_name(logger_err));
#else
            ESP_LOGI(TAG, "RECORD_STOP ignorado: DataLogger desactivado");
#endif
        }
    }

    free(payload);
    return err;
}
//----------------------------------------------------------------------------------
static void websocket_broadcast_work(void *arg)
{
    websocket_work_t *work = arg;

    if ((work == NULL) || (work->server == NULL))
    {
        free(work);
        return;
    }

    int client_fds[MAX_WS_CLIENTS];
    size_t client_count = MAX_WS_CLIENTS;

    if (httpd_get_client_list(work->server,
                              &client_count,
                              client_fds) == ESP_OK)
    {
        httpd_ws_frame_t frame = {
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)work->json,
            .len = strlen(work->json),
        };

        for (size_t i = 0; i < client_count; ++i)
        {
            const int fd = client_fds[i];

            if (httpd_ws_get_fd_info(work->server, fd) ==
                HTTPD_WS_CLIENT_WEBSOCKET)
            {
                esp_err_t err = httpd_ws_send_frame_async(
                    work->server,
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

    free(work);
}
//----------------------------------------------------------------------------------
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
static esp_err_t internal_temperature_init(void)
{
    temperature_sensor_config_t config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);

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
static void internal_temperature_log(void)
{
    /*
    if (s_temp_sensor == NULL)
    {
        return;
    }

    float temperature_c = 0.0f;

    esp_err_t err = temperature_sensor_get_celsius(s_temp_sensor, &temperature_c);


        if (err == ESP_OK)
            ESP_LOGI(TAG,  "Temperatura interna ESP32-S3: %.1f C", (double)temperature_c);
        else
            ESP_LOGW(TAG,  "Error leyendo temperatura interna: %s", esp_err_to_name(err));
    */
}

//----------------------------------------------------------------------------------
static void telemetry_task(void *arg)
{
    httpd_handle_t server = (httpd_handle_t)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(TELEMETRY_PERIOD_MS);
    uint32_t counter = 0U;
    uint32_t temp_log_divider = 0U;

    for (;;)
    {
        /*
         * Diagnóstico térmico independiente de la existencia de clientes.
         * TELEMETRY_PERIOD_MS = 100 ms -> una lectura cada 10 ciclos = 1 s.
         */
        if (++temp_log_divider >= (1000U / TELEMETRY_PERIOD_MS))
        {
            temp_log_divider = 0U;
            internal_temperature_log();
        }

        /*
         * Si no hay ningún cliente WebSocket conectado no se construye el JSON
         * ni se encola trabajo de transmisión.
         */
        if (websocket_count_clients(server) == 0U)
        {
            vTaskDelayUntil(&last_wake, period);
            continue;
        }

        const float t = (float)esp_timer_get_time() / 1000000.0f;

        /*
         * Instantánea atómica de los datos procesados por la tarea BNO055.
         * Si todavía no existe una medida válida se transmiten ceros.
         */
        const bno055_data_t imu = BNO055_get_data();

        /*
         * GPS independiente de BMP280/BNO055. De momento no se fusionan
         * datos entre sensores.
         */
        const gps_data_t gps = GPS_get_data();

        const float roll = imu.valid ? imu.roll_deg : 0.0f;
        const float pitch = imu.valid ? imu.pitch_deg : 0.0f;

        /*
         * Girodireccional:
         *
         * El rumbo mostrado procede exclusivamente del ground track GPS.
         * Para evitar valores erráticos a muy baja velocidad, sólo se acepta
         * una nueva muestra cuando:
         *
         *   - existe FIX válido,
         *   - ground_track_deg es finito,
         *   - ground_speed_knots es finita,
         *   - GS >= GPS_HEADING_MIN_SPEED_KT.
         *
         * Si la muestra deja de ser válida se conserva el último heading GPS.
         * Si desde el arranque todavía no ha habido ninguno válido, se usa 0°.
         */
        const bool gps_heading_sample_valid =
            gps.fix_valid &&
            isfinite(gps.ground_track_deg) &&
            isfinite(gps.ground_speed_knots) &&
            (gps.ground_speed_knots >= GPS_HEADING_MIN_SPEED_KT);

        if (gps_heading_sample_valid)
        {
            float heading = gps.ground_track_deg;

            while (heading < 0.0f)
                heading += 360.0f;

            while (heading >= 360.0f)
                heading -= 360.0f;

            s_last_gps_heading_deg = heading;
            s_last_gps_heading_valid = true;
        }

        // const float yaw = s_last_gps_heading_valid ? s_last_gps_heading_deg : 0.0f;   //  ###GPS###
        const float yaw = imu.valid ? imu.heading_deg : 0.0f; //  ###DEMO###

        /*
         * altitude y vertical_speed proceden de BMP280.c.
         * Durante la prueba, altitude contiene la señal triangular generada
         * por main.c y vertical_speed es calculada por el filtro del BMP280.
         */

        /*
         * Avión del coordinador:
         * velocidad de cambio de rumbo compensada por roll y pitch.
         */
        const float turn_rate_dps =
            imu.valid ? imu.yaw_rate_dps : 0.0f;

        /*
         * Bola del coordinador:
         * aceleración lateral medida y filtrada en BNO055Task().
         */
        const float slip_ball_deg =
            imu.valid ? imu.slip_ball_deg : 0.0f;

        /*
         * Magnitud total de aceleración específica:
         *
         * g_current = sqrt(gx^2 + gy^2 + gz^2).
         * g_max y g_min son los extremos retenidos desde el último reset.
         */
        const float g_current = imu.valid ? imu.g_current : 0.0f;

        const float g_max = imu.valid ? imu.g_max : 0.0f;

        const float g_min =
            imu.valid ? imu.g_min : 0.0f;

        const settings_snapshot_t settings = settings_get_snapshot();

        /*
         * Estado del registro SPIFFS mostrado en la interfaz web.
         */

#if DATALOGGER_ENABLED
        const datalogger_status_t logger = DataLogger_get_status();
#else
        /* Mantiene intacta la estructura JSON esperada por index.html. */
        const struct
        {
            bool recording;
            bool file_available;
            uint32_t samples;
            size_t file_size_bytes;
        } logger = {false, false, 0U, 0U};
#endif

        websocket_work_t *work = calloc(1U, sizeof(*work));

        if (work != NULL)
        {
            work->server = server;

            snprintf(
                work->json,
                sizeof(work->json),

                "{"

                /*----------------------------------------------------------*/
                /* Diagnóstico general                                      */
                /*----------------------------------------------------------*/
                "\"counter\":%" PRIu32 "," // Contador de mensajes
                "\"time_s\":%.3f,"         // Tiempo desde el arranque

                /*----------------------------------------------------------*/
                /* Horizonte artificial                                     */
                /*----------------------------------------------------------*/
                "\"roll\":%.2f,"  // Ángulo de alabeo
                "\"pitch\":%.2f," // Ángulo de cabeceo

                /*----------------------------------------------------------*/
                /* Girodireccional: heading GPS retenido                    */
                /*----------------------------------------------------------*/
                "\"yaw\":%.2f," // Rumbo mostrado

                /*----------------------------------------------------------*/
                /* Altímetro y variómetro                                   */
                /*----------------------------------------------------------*/
                "\"altitude\":%.2f,"       // Altitud en metros
                "\"vertical_speed\":%.2f," // Velocidad vertical en m/s
                "\"temperature\":%.2f,"    // Temperatura del BMP280
                /*----------------------------------------------------------*/
                /* GPS / GNSS                                               */
                /*----------------------------------------------------------*/
                "\"gps_valid\":%s,"            // FIX GPS válido
                "\"gps_altitude_m\":%.2f,"     // Altitud GPS [m]
                "\"gps_latitude_deg\":%.7f,"   // Latitud decimal
                "\"gps_longitude_deg\":%.7f,"  // Longitud decimal
                "\"ground_speed_knots\":%.2f," // Ground Speed [kt]
                "\"ground_speed_kmh\":%.2f,"   // Ground Speed [km/h]
                "\"gps_track_deg\":%.2f,"      // Track GPS sobre el suelo [deg]
                "\"gps_utc_day\":%u,"          // Día UTC
                "\"gps_utc_month\":%u,"        // Mes UTC
                "\"gps_utc_year\":%u,"         // Año UTC
                "\"gps_utc_hour\":%u,"         // Hora UTC
                "\"gps_utc_minute\":%u,"       // Minuto UTC
                "\"gps_utc_second\":%u,"       // Segundo UTC

                /*----------------------------------------------------------*/
                /* DataLogger SPIFFS                                        */
                /*----------------------------------------------------------*/
                "\"recording\":%s,"            // Grabación activa
                "\"log_available\":%s,"        // Existe fichero descargable
                "\"log_samples\":%" PRIu32 "," // Muestras de la sesión actual
                "\"log_size_bytes\":%u,"       // Tamaño actual del CSV

                /*----------------------------------------------------------*/
                /* Ajustes persistentes del usuario                         */
                /*----------------------------------------------------------*/
                "\"qnh\":%.2f,"                       // Ajuste QNH del altímetro
                "\"pitch_offset_deg\":%" PRId32 ","   // Offset del horizonte
                "\"heading_offset_deg\":%" PRIu32 "," // Heading manual persistente
                "\"mount_mode\":%" PRIu32 ","         // 0=V, 1=H

                /*----------------------------------------------------------*/
                /* Coordinador de viraje y bola                             */
                /*----------------------------------------------------------*/
                "\"turn_rate_dps\":%.2f," // Régimen de giro, deg/s
                "\"slip_ball_deg\":%.2f," // Posición física de la bola

                /*----------------------------------------------------------*/
                /* G-meter de tres agujas                                   */
                /*----------------------------------------------------------*/
                "\"g_current\":%.2f," // Aguja blanca: G instantánea
                "\"g_max\":%.2f,"     // Aguja azul: máximo retenido
                "\"g_min\":%.2f,"     // Aguja verde: mínimo retenido

                /*----------------------------------------------------------*/
                /* Diagnóstico del acelerómetro BNO055                     */
                /*----------------------------------------------------------*/
                "\"accel_x_g\":%.3f,"     // Aceleración X en G
                "\"accel_y_g\":%.3f,"     // Aceleración Y en G
                "\"accel_z_g\":%.3f,"     // Aceleración Z en G
                "\"accel_total_g\":%.3f," // Módulo total de aceleración

                /*----------------------------------------------------------*/
                /* Diagnóstico del giróscopo BNO055                        */
                /*----------------------------------------------------------*/
                "\"gyro_x_dps\":%.2f,"   // Velocidad angular X
                "\"gyro_y_dps\":%.2f,"   // Velocidad angular Y
                "\"gyro_z_dps\":%.2f,"   // Velocidad angular Z del cuerpo
                "\"yaw_rate_dps\":%.2f," // Cambio de rumbo compensado

                /*----------------------------------------------------------*/
                /* Estado de validez de la IMU                              */
                /*----------------------------------------------------------*/
                "\"imu_valid\":%s" // true cuando hay datos válidos

                "}",
                counter++,
                (double)t,
                -(double)roll,
                (double)pitch,
                (double)yaw,
                (double)altitude,
                (double)vertical_speed,
                (double)temperature,
                gps.fix_valid ? "true" : "false",
                (double)(gps.fix_valid ? gps.altitude_m : 0.0f),
                gps.fix_valid ? gps.latitude_deg : 0.0,
                gps.fix_valid ? gps.longitude_deg : 0.0,
                (double)(gps.fix_valid ? gps.ground_speed_knots : 0.0f),
                (double)(gps.fix_valid ? gps.ground_speed_kmh : 0.0f),
                (double)(gps.fix_valid ? gps.ground_track_deg : 0.0f), // gps
                gps.utc_day,
                gps.utc_month,
                gps.utc_year,
                gps.utc_hour,
                gps.utc_minute,
                gps.utc_second,
                logger.recording ? "true" : "false",
                logger.file_available ? "true" : "false",
                logger.samples,
                (unsigned)logger.file_size_bytes,
                (double)settings.qnh_hpa,
                settings.pitch_offset_deg,
                settings.heading_offset_deg,
                settings.mount_mode,
                (double)turn_rate_dps,
                (double)slip_ball_deg,
                (double)g_current,
                (double)g_max,
                (double)g_min,
                (double)imu.accel_x_g,
                (double)imu.accel_y_g,
                (double)imu.accel_z_g,
                (double)imu.accel_total_g,
                (double)imu.gyro_x_dps,
                (double)imu.gyro_y_dps,
                (double)imu.gyro_z_dps,
                (double)imu.yaw_rate_dps,
                imu.valid ? "true" : "false");

            esp_err_t err = httpd_queue_work(
                server,
                websocket_broadcast_work,
                work);

            if (err != ESP_OK)
            {
                free(work);
            }
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
//----------------------------------------------------------------------------------
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

    settings_load();

    /*
     * Sensor térmico interno del ESP32-S3.
     * Se usa únicamente como diagnóstico y se imprime una vez por segundo.
     */
    (void)internal_temperature_init();

    BaseType_t ok = xTaskCreate(
        telemetry_task,
        "telemetry",
        4096,
        server,
        5,
        &s_dummy_task);

    if (ok != pdPASS)
    {
        s_dummy_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Telemetría iniciada a %u Hz", 1000U / TELEMETRY_PERIOD_MS);

    return ESP_OK;
}
//----------------------------------------------------------------------------------
float websocket_get_qnh(void)
{
    float qnh;

    portENTER_CRITICAL(&s_settings_mux);
    qnh = s_qnh_hpa;
    portEXIT_CRITICAL(&s_settings_mux);

    return qnh;
}