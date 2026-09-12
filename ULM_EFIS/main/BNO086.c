/**
 * @file BNO086.c
 * @brief Procesado de alto nivel del BNO086 para ULM-EFIS WiFi.
 */

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "BNO086.h"
#include "BNO086_driver.h"
#include "config.h"

#define RAD_TO_DEG 57.29577951308232f
#define STANDARD_GRAVITY_MS2 9.80665f

static const char *TAG = "BNO086";

/* Cola de longitud uno registrada por websocket.c para publicar la última IMU. */
static QueueHandle_t s_output_queue = NULL;

/* Handle usado por la ISR para despertar exclusivamente a la tarea del sensor. */
static TaskHandle_t s_task = NULL;
/* Protege la instantánea pública, el modo de montaje y el handle de la cola. */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* Última muestra completa disponible para lectores que no utilizan la cola. */
static bno086_data_t s_data;
/* Orientación mecánica aplicada a todos los vectores y al cuaternión. */
static bno086_mount_mode_t s_mount_mode = BNO086_MOUNT_VERTICAL;

/* Indica que los extremos del G-meter ya contienen una primera medida válida. */
static bool s_g_initialized = false;

/* Estado interno persistente del filtro de régimen de giro. */
static float s_yaw_rate_filtered_dps = 0.0f;
// static float s_slip_ball_filtered_d = 0.0f;

static bool s_yaw_rate_filter_initialized = false;
// static bool s_slip_ball_initialized = false;

/* -------------------------------------------------------------------------- */
/* Utilidades                                                                  */
/* -------------------------------------------------------------------------- */

/** Convierte dos bytes little-endian de SH-2 en un entero con signo. */
static int16_t read_i16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
/*----------------------------------------------------------*/
/** Limita value al intervalo cerrado [low, high]. */
static float clampf(float value, float low, float high)
{
    if (value < low)
        return low;

    if (value > high)
        return high;

    return value;
}

/* -------------------------------------------------------------------------- */
/* Cambio de ejes                                                              */
/* -------------------------------------------------------------------------- */

/**
 * Convierte un vector de aceleración expresado en ejes del sensor a los ejes
 * utilizados por el EFIS. Se usa también para gravedad y aceleración lineal.
 */
static void acceleration_to_aircraft_axes(
    const bno086_vector3f_t *raw,
    bno086_vector3f_t *aircraft)
{
    const bno086_mount_mode_t mount = BNO086_get_mount_mode();

    if (mount == BNO086_MOUNT_VERTICAL)
    {
        /*
         * Configuración VALIDADA experimentalmente:
         *
         * X_avion = -Y_sensor
         * Y_avion = -X_sensor
         * Z_avion = -Z_sensor
         */
        aircraft->x = -raw->y;
        aircraft->y = -raw->x;
        aircraft->z = -raw->z;
    }
    else
    {
        /*
         * Montaje H: placa girada 90 grados respecto de V.
         */
        aircraft->x = -raw->x;
        aircraft->y = raw->y;
        aircraft->z = -raw->z;
    }
}
/*----------------------------------------------------------*/
/** Convierte las velocidades angulares del sensor a p, q y r del avión. */
static void gyro_to_aircraft_axes(const bno086_vector3f_t *raw, bno086_vector3f_t *aircraft)
{
    const bno086_mount_mode_t mount = BNO086_get_mount_mode();

    if (mount == BNO086_MOUNT_VERTICAL)
    {
        /*
         * Configuración VALIDADA experimentalmente:
         *
         * p = +Y_sensor
         * q = +X_sensor
         * r = -Z_sensor
         */
        aircraft->x = raw->y;
        aircraft->y = raw->x;
        aircraft->z = -raw->z;
    }
    else
    {
        aircraft->x = raw->x;
        aircraft->y = -raw->y;
        aircraft->z = -raw->z;
    }
}
/*----------------------------------------------------------*/
/**
 * Aplica al cuaternión el mismo convenio de ejes que al giróscopo y lo
 * renormaliza para limitar el error introducido por la cuantificación Q14.
 */
static void quaternion_to_aircraft_axes(
    const bno086_quaternionf_t *raw,
    bno086_quaternionf_t *aircraft)
{
    const bno086_mount_mode_t mount = BNO086_get_mount_mode();

    aircraft->w = raw->w;

    if (mount == BNO086_MOUNT_VERTICAL)
    {
        /*
         * Rotación propia deducida del remapeo validado del giróscopo.
         */
        aircraft->x = raw->y;
        aircraft->y = raw->x;
        aircraft->z = -raw->z;
    }
    else
    {
        aircraft->x = raw->x;
        aircraft->y = -raw->y;
        aircraft->z = -raw->z;
    }

    const float norm = sqrtf(
        aircraft->w * aircraft->w +
        aircraft->x * aircraft->x +
        aircraft->y * aircraft->y +
        aircraft->z * aircraft->z);

    if (norm > 0.0f)
    {
        aircraft->w /= norm;
        aircraft->x /= norm;
        aircraft->y /= norm;
        aircraft->z /= norm;
    }
}
/*----------------------------------------------------------*/
/** Extrae los ángulos de Euler en grados a partir del cuaternión normalizado. */
static void quaternion_to_euler(const bno086_quaternionf_t *q, float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    const float sin_roll_cos_pitch = 2.0f * (q->w * q->x + q->y * q->z);

    const float cos_roll_cos_pitch = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);

    const float roll_rad = atan2f(sin_roll_cos_pitch, cos_roll_cos_pitch);

    float sin_pitch = 2.0f * (q->w * q->y - q->z * q->x);

    sin_pitch = clampf(sin_pitch, -1.0f, 1.0f);

    const float pitch_rad = asinf(sin_pitch);

    const float sin_yaw_cos_pitch = 2.0f * (q->w * q->z + q->x * q->y);

    const float cos_yaw_cos_pitch = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);

    const float yaw_rad = atan2f(sin_yaw_cos_pitch, cos_yaw_cos_pitch);

    *roll_deg = -roll_rad * RAD_TO_DEG;
    *pitch_deg = pitch_rad * RAD_TO_DEG;
    *yaw_deg = yaw_rad * RAD_TO_DEG;
}

/* -------------------------------------------------------------------------- */
/* Magnitudes derivadas para el EFIS                                          */
/* -------------------------------------------------------------------------- */

/*----------------------------------------------------------*/

/**
 * Calcula la velocidad angular alrededor de la vertical local proyectando el
 * vector 3D del giróscopo sobre el vector unitario de gravedad.
 */
static float compute_vertical_turn_rate_dps(
    const bno086_vector3f_t *gyro_dps,
    const bno086_vector3f_t *gravity_ms2)
{
    const float g_norm = sqrtf(
        gravity_ms2->x * gravity_ms2->x +
        gravity_ms2->y * gravity_ms2->y +
        gravity_ms2->z * gravity_ms2->z);

    if (!isfinite(g_norm) || (g_norm < 1.0f))
    {
        return gyro_dps->z;
    }

    /* Componentes del vector unitario que define la vertical local. */
    const float gx = gravity_ms2->x / g_norm;
    const float gy = gravity_ms2->y / g_norm;
    const float gz = gravity_ms2->z / g_norm;

    const float turn_rate = -(gyro_dps->x * gx + gyro_dps->y * gy + gyro_dps->z * gz);

    // printf("%.2f %.2f %.2f\n", gx, gy, gz);

    return turn_rate;
}
/*----------------------------------------------------------*/

/** Filtro paso bajo exponencial del bastón, con zona muerta solo en la salida. */
static float filter_yaw_rate(float input_dps, float dt_s)
{
    if (!isfinite(input_dps))
    {
        return s_yaw_rate_filtered_dps;
    }

    if (!s_yaw_rate_filter_initialized)
    {
        s_yaw_rate_filtered_dps = input_dps;
        s_yaw_rate_filter_initialized = true;
        return s_yaw_rate_filtered_dps;
    }

    const float alpha = 1.0f - expf(-dt_s / TURN_RATE_FILTER_TAU_S);

    s_yaw_rate_filtered_dps += alpha * (input_dps - s_yaw_rate_filtered_dps);

    return (fabsf(s_yaw_rate_filtered_dps) < TURN_RATE_DEADBAND_DPS) ? 0.0f : s_yaw_rate_filtered_dps;
}

/*----------------------------------------------------------*/
/* Estado independiente del filtro de la bola. */
static float s_slip_ball_filtered_deg = 0.0f;
static bool s_slip_ball_filter_initialized = false;

/*----------------------------------------------------------*/
/** Filtro paso bajo exponencial aplicado al ángulo equivalente de resbale. */
static float filter_slip_ball(float slip_ball_deg, float dt_s)
{
    if (!isfinite(slip_ball_deg))
    {
        return (fabsf(s_slip_ball_filtered_deg) <
                SLIP_BALL_DEADBAND_DEG)
                   ? 0.0f
                   : s_slip_ball_filtered_deg;
    }

    if (!s_slip_ball_filter_initialized)
    {
        s_slip_ball_filtered_deg = slip_ball_deg;
        s_slip_ball_filter_initialized = true;
    }
    else if ((dt_s > 0.0f) && isfinite(dt_s))
    {
        const float alpha =
            1.0f - expf(-dt_s / SLIP_BALL_FILTER_TAU_S);

        s_slip_ball_filtered_deg +=
            alpha *
            (slip_ball_deg - s_slip_ball_filtered_deg);
    }

    /*
     * La zona muerta solo afecta a la salida;
     * el estado interno continúa evolucionando.
     */
    return (fabsf(s_slip_ball_filtered_deg) < SLIP_BALL_DEADBAND_DEG) ? 0.0f : s_slip_ball_filtered_deg;
}

/*----------------------------------------------------------*/
/**
 * Convierte la aceleración lineal lateral Y, ya libre de gravedad, en el
 * ángulo equivalente utilizado para representar la bola.
 */
static float compute_slip_ball_deg(const bno086_vector3f_t *linear_acceleration_ms2)
{
    /*
     * SH-2 ya entrega LINEAR ACCELERATION con la gravedad eliminada.
     * Por tanto no volvemos a estimar ni a restar la gravedad.
     *
     * Y es el eje lateral del avión.
     */

    float lateral_g = linear_acceleration_ms2->y / STANDARD_GRAVITY_MS2;

    float ball_deg = -atanf(lateral_g) * RAD_TO_DEG;

    ball_deg *= SLIP_BALL_GAIN;

    return clampf(ball_deg, -SLIP_BALL_LIMIT_DEG, SLIP_BALL_LIMIT_DEG);
}
/*----------------------------------------------------------*/
/** Calcula G total y actualiza los extremos retenidos desde el último reset. */
static void process_g_meter(bno086_data_t *data)
{
    const float gx = data->acceleration_ms2.x / STANDARD_GRAVITY_MS2;
    const float gy = data->acceleration_ms2.y / STANDARD_GRAVITY_MS2;
    const float gz = data->acceleration_ms2.z / STANDARD_GRAVITY_MS2;

    data->accel_x_g = gx;
    data->accel_y_g = gy;
    data->accel_z_g = gz;

    data->accel_total_g =
        sqrtf(gx * gx + gy * gy + gz * gz);

    data->g_current = data->accel_total_g;

    if (!s_g_initialized)
    {
        data->g_min = data->g_current;
        data->g_max = data->g_current;
        s_g_initialized = true;
        return;
    }

    if (data->g_current < data->g_min)
        data->g_min = data->g_current;

    if (data->g_current > data->g_max)
        data->g_max = data->g_current;
}

/* -------------------------------------------------------------------------- */
/* SH-2                                                                        */
/* -------------------------------------------------------------------------- */

/** Devuelve el tamaño SH-2 esperado para cada informe habilitado. */
static size_t report_size(uint8_t report_id)
{
    switch (report_id)
    {
    case BNO086_REPORT_ACCELEROMETER:
    case BNO086_REPORT_GYROSCOPE:
    case BNO086_REPORT_LINEAR_ACCELERATION:
    case BNO086_REPORT_GRAVITY:
        return 10U;

    case BNO086_REPORT_GAME_ROTATION_VECTOR:
        return 12U;

    default:
        return 0U;
    }
}

/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/

/**
 * Escala un informe SH-2, cambia sus ejes y actualiza el campo correspondiente
 * de la muestra acumulada, incluida su precisión y bandera de validez.
 */
static void process_sensor_report(bno086_data_t *sample, const uint8_t *report)
{
    const uint8_t report_id = report[0];
    const uint8_t accuracy = report[2] & 0x03U;

    if ((report_id == BNO086_REPORT_ACCELEROMETER) ||
        (report_id == BNO086_REPORT_GYROSCOPE) ||
        (report_id == BNO086_REPORT_LINEAR_ACCELERATION) ||
        (report_id == BNO086_REPORT_GRAVITY))
    {
        const int16_t raw_x = read_i16_le(&report[4]);
        const int16_t raw_y = read_i16_le(&report[6]);
        const int16_t raw_z = read_i16_le(&report[8]);

        if (report_id == BNO086_REPORT_GYROSCOPE)
        {
            const bno086_vector3f_t scaled =
                {
                    .x = (float)raw_x / 512.0f * RAD_TO_DEG,
                    .y = (float)raw_y / 512.0f * RAD_TO_DEG,
                    .z = (float)raw_z / 512.0f * RAD_TO_DEG,
            };

            gyro_to_aircraft_axes(&scaled, &sample->gyro_dps);

            sample->gyro_accuracy = accuracy;
            sample->gyro_valid = true;
            // printf("Gyro\n");
        }
        else
        {
            const bno086_vector3f_t scaled =
                {
                    .x = (float)raw_x / 256.0f,
                    .y = (float)raw_y / 256.0f,
                    .z = (float)raw_z / 256.0f,
            };

            if (report_id == BNO086_REPORT_ACCELEROMETER)
            {
                acceleration_to_aircraft_axes(&scaled, &sample->acceleration_ms2);

                sample->acceleration_accuracy = accuracy;
                sample->acceleration_valid = true;
                // printf("Acc\n");
            }
            else if (report_id == BNO086_REPORT_LINEAR_ACCELERATION)
            {
                acceleration_to_aircraft_axes(&scaled, &sample->linear_acceleration_ms2);

                sample->linear_accel_accuracy = accuracy;
                sample->linear_accel_valid = true;
                // printf("LinAcc\n");
            }
            else
            {
                acceleration_to_aircraft_axes(&scaled, &sample->gravity_ms2);

                sample->gravity_accuracy = accuracy;
                sample->gravity_valid = true;

                // printf("Other\n");
            }
        }
    }
    else if (report_id == BNO086_REPORT_GAME_ROTATION_VECTOR)
    {
        const bno086_quaternionf_t raw =
            {
                .x = (float)read_i16_le(&report[4]) / 16384.0f,
                .y = (float)read_i16_le(&report[6]) / 16384.0f,
                .z = (float)read_i16_le(&report[8]) / 16384.0f,
                .w = (float)read_i16_le(&report[10]) / 16384.0f,
        };

        quaternion_to_aircraft_axes(&raw, &sample->quaternion);

        // a partir del cuaternio, extrae pitch, roll y heading
        quaternion_to_euler(&sample->quaternion, &sample->roll_deg, &sample->pitch_deg, &sample->heading_deg);

        // printf("GameRot\n");
        sample->rotation_accuracy = accuracy;
        sample->rotation_valid = true;
    }
}
/*----------------------------------------------------------*/
#define REPORT_MASK_ACCELERATION (1U << 0)
#define REPORT_MASK_GYROSCOPE (1U << 1)
#define REPORT_MASK_LINEAR_ACCEL (1U << 2)
#define REPORT_MASK_GRAVITY (1U << 3)
#define REPORT_MASK_GAME_RV (1U << 4)

#define REQUIRED_REPORTS        \
    (REPORT_MASK_ACCELERATION | \
     REPORT_MASK_GYROSCOPE |    \
     REPORT_MASK_LINEAR_ACCEL | \
     REPORT_MASK_GRAVITY |      \
     REPORT_MASK_GAME_RV)

/**
 * Procesa todos los informes contenidos en un paquete SHTP.
 *
 * @return Máscara REPORT_MASK_* de los informes encontrados en el paquete.
 */
static uint32_t process_packet(bno086_data_t *sample, const uint8_t *packet, uint16_t packet_len)
{
    if (packet_len < 9U)
        return 0U;

    if (packet[2] != BNO086_SHTP_CHANNEL_REPORTS)
        return 0U;

    const uint8_t *payload = &packet[4];
    const size_t payload_len = packet_len - 4U;

    if (payload[0] != BNO086_SHTP_REPORT_BASE_TIMESTAMP)
        return 0U;

    size_t offset = 5U;
    uint32_t reports = 0U;

    while (offset < payload_len)
    {
        const uint8_t report_id = payload[offset];
        const size_t size = report_size(report_id);

        if ((size == 0U) || ((offset + size) > payload_len))
        {
            break;
        }

        process_sensor_report(sample, &payload[offset]);

        switch (report_id)
        {
        case BNO086_REPORT_ACCELEROMETER:
            reports |= REPORT_MASK_ACCELERATION;
            break;

        case BNO086_REPORT_GYROSCOPE:
            reports |= REPORT_MASK_GYROSCOPE;
            break;

        case BNO086_REPORT_LINEAR_ACCELERATION:
            reports |= REPORT_MASK_LINEAR_ACCEL;
            break;

        case BNO086_REPORT_GRAVITY:
            reports |= REPORT_MASK_GRAVITY;
            break;

        case BNO086_REPORT_GAME_ROTATION_VECTOR:
            reports |= REPORT_MASK_GAME_RV;
            break;

        default:
            break;
        }

        offset += size;
    }

    return reports;
}

/* -------------------------------------------------------------------------- */
/* ISR                                                                       */
/* -------------------------------------------------------------------------- */

/** ISR mínima de H_INTN: notifica a BNO086Task y solicita cambio de contexto. */
static void IRAM_ATTR bno086_int_isr(void *arg)
{
    (void)arg;

    if (s_task == NULL)
        return;

    BaseType_t higher_priority_task_woken = pdFALSE;

    vTaskNotifyGiveFromISR(s_task, &higher_priority_task_woken);

    if (higher_priority_task_woken == pdTRUE)
        portYIELD_FROM_ISR();
}

/*----------------------------------------------------------*/
/*----------------------------------------------------------*/
/*----------------------------------------------------------*/
/**
 * Tarea de adquisición dirigida por H_INTN. Acumula informes hasta construir
 * una muestra completa, calcula las magnitudes derivadas y publica la última.
 */
static void BNO086Task(void *arg)
{
    (void)arg;

    uint8_t packet[BNO086_PACKET_BUFFER_SIZE];

    /* Periodo nominal utilizado por los filtros discretos. */
    const float dt_s = (float)BNO086_REPORT_INTERVAL_US / 1000000.0f;

    uint32_t consecutive_errors = 0U;
    /* Informes frescos recibidos desde la última publicación completa. */
    uint32_t pending_reports = 0U;

    /*
     * Esta estructura acumula los diferentes informes hasta
     * completar una muestra.
     */
    bno086_data_t sample = BNO086_get_data();

    while (1)
    {
        /*
         * H_INTN es activo a nivel bajo. Si está alto, la tarea
         * espera a que la ISR detecte un flanco descendente.
         */
        if (gpio_get_level(BNO086_INT_GPIO) != 0)
        {
            ulTaskNotifyTake(
                pdTRUE,
                portMAX_DELAY);
        }

        /*
         * Leer todos los paquetes pendientes mientras H_INTN
         * permanezca activo.
         */
        while (gpio_get_level(BNO086_INT_GPIO) == 0)
        {
            uint16_t packet_len = 0U;

            const esp_err_t err =
                bno086_driver_receive_packet(
                    packet,
                    sizeof(packet),
                    &packet_len);

            if (err == ESP_OK)
            {
                consecutive_errors = 0U;

                /*
                 * process_packet() debe devolver una máscara con
                 * los informes encontrados en este paquete.
                 */
                pending_reports |=
                    process_packet(
                        &sample,
                        packet,
                        packet_len);

                /*
                 * No calcular ni publicar hasta haber recibido todos
                 * los informes necesarios.
                 */
                if ((pending_reports & REQUIRED_REPORTS) ==
                    REQUIRED_REPORTS)
                {
                    /*
                     * La muestra contiene todos los datos utilizados
                     * por los cálculos derivados.
                     */
                    sample.valid =
                        sample.acceleration_valid &&
                        sample.gyro_valid &&
                        sample.linear_accel_valid &&
                        sample.gravity_valid &&
                        sample.rotation_valid;

                    /*
                     * Factor de carga.
                     */
                    process_g_meter(&sample);

                    /*
                     * Régimen de giro.
                     */
                    const float yaw_rate_raw_dps =
                        compute_vertical_turn_rate_dps(
                            &sample.gyro_dps,
                            &sample.gravity_ms2);

                    sample.yaw_rate_dps =
                        filter_yaw_rate(
                            yaw_rate_raw_dps,
                            dt_s);

                    /*
                     * Bola de resbale.
                     */
                    const float slip_ball_raw_deg =
                        compute_slip_ball_deg(
                            &sample.linear_acceleration_ms2);

                    sample.slip_ball_deg =
                        filter_slip_ball(
                            slip_ball_raw_deg,
                            dt_s);

                    /*
                     * Publicar atómicamente la muestra completa.
                     */
                    portENTER_CRITICAL(&s_mux);

                    s_data = sample;
                    QueueHandle_t output_queue =
                        s_output_queue;

                    portEXIT_CRITICAL(&s_mux);

                    /*
                     * La cola tiene longitud uno: la muestra nueva
                     * sustituye cualquier muestra pendiente.
                     */
                    if (output_queue != NULL)
                    {
                        xQueueOverwrite(
                            output_queue,
                            &sample);
                    }

                    /*
                     * Comenzar la formación de la siguiente muestra.
                     */
                    pending_reports = 0U;
                }
            }
            else if (err == ESP_ERR_NOT_FOUND)
            {
                /*
                 * H_INTN puede desactivarse entre la comprobación
                 * del GPIO y la lectura I2C.
                 */
                break;
            }
            else
            {
                consecutive_errors++;

                ESP_LOGW(
                    TAG,
                    "Error SHTP (%u/%u): %s",
                    (unsigned)consecutive_errors,
                    (unsigned)BNO086_MAX_CONSECUTIVE_ERRORS,
                    esp_err_to_name(err));

                if ((err == ESP_ERR_TIMEOUT) ||
                    (consecutive_errors >=
                     BNO086_MAX_CONSECUTIVE_ERRORS))
                {
                    const esp_err_t recovery_err =
                        bno086_driver_recover();

                    /*
                     * Después de recuperar el sensor no deben
                     * conservarse informes parciales anteriores.
                     */
                    pending_reports = 0U;

                    if (recovery_err != ESP_OK)
                    {
                        ESP_LOGE(
                            TAG,
                            "No se pudo recuperar BNO086: %s",
                            esp_err_to_name(recovery_err));

                        vTaskDelay(
                            pdMS_TO_TICKS(250U));
                    }
                    else
                    {
                        consecutive_errors = 0U;
                    }

                    break;
                }

                vTaskDelay(
                    pdMS_TO_TICKS(2U));
            }
        }
    }
}

/* -------------------------------------------------------------------------- */
/* API pública                                                                 */
/* -------------------------------------------------------------------------- */

/** Obtiene una copia atómica de la última muestra publicada. */
bno086_data_t BNO086_get_data(void)
{
    bno086_data_t copy;

    portENTER_CRITICAL(&s_mux);
    copy = s_data;
    portEXIT_CRITICAL(&s_mux);

    return copy;
}

/*----------------------------------------------------------*/
/** Registra o elimina la cola de longitud uno utilizada por telemetría. */
void BNO086_set_output_queue(QueueHandle_t queue)
{
    portENTER_CRITICAL(&s_mux);
    s_output_queue = queue;
    portEXIT_CRITICAL(&s_mux);
}

/*----------------------------------------------------------*/
/** Reinicia los extremos retenidos del G-meter alrededor de la G actual. */
void BNO086_reset_accel_peaks(void)
{
    portENTER_CRITICAL(&s_mux);

    s_data.g_min = s_data.g_current;
    s_data.g_max = s_data.g_current;
    s_g_initialized = s_data.valid;

    portEXIT_CRITICAL(&s_mux);
}
/*----------------------------------------------------------*/
/** Valida y cambia de forma atómica la orientación mecánica del módulo. */
esp_err_t BNO086_set_mount_mode(bno086_mount_mode_t mode)
{
    if ((mode != BNO086_MOUNT_VERTICAL) && (mode != BNO086_MOUNT_HORIZONTAL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_mux);
    s_mount_mode = mode;
    portEXIT_CRITICAL(&s_mux);

    ESP_LOGI(
        TAG,
        "Modo de montaje BNO086: %s",
        mode == BNO086_MOUNT_HORIZONTAL ? "H" : "V");

    return ESP_OK;
}

/*----------------------------------------------------------*/
/** Devuelve de forma atómica el montaje V/H seleccionado. */
bno086_mount_mode_t BNO086_get_mount_mode(void)
{
    bno086_mount_mode_t mode;

    portENTER_CRITICAL(&s_mux);
    mode = s_mount_mode;
    portEXIT_CRITICAL(&s_mux);

    return mode;
}

/*----------------------------------------------------------*/
/** Inicializa driver, ISR y tarea de adquisición del BNO086. */
esp_err_t BNO086_start(void)
{
    if (s_task != NULL)
        return ESP_ERR_INVALID_STATE;

    ESP_RETURN_ON_ERROR(
        bno086_driver_init(),
        TAG,
        "No se pudo inicializar BNO086");

    const BaseType_t ok = xTaskCreate(BNO086Task, "bno086", BNO086_TASK_STACK_SIZE, NULL, BNO086_TASK_PRIORITY, &s_task);

    if (ok != pdPASS)
    {
        s_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    /*
     * El servicio ISR puede estar ya instalado por otro módulo. En ese caso
     * ESP_ERR_INVALID_STATE no es un error fatal.
     */
    esp_err_t err = gpio_install_isr_service(0);
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE))
    {
        vTaskDelete(s_task);
        s_task = NULL;
        return err;
    }

    err = gpio_isr_handler_add(BNO086_INT_GPIO, bno086_int_isr, NULL);

    if (err != ESP_OK)
    {
        vTaskDelete(s_task);
        s_task = NULL;
        return err;
    }

    /* Evita perder un INT que ya estuviera activo al instalar la ISR. */
    xTaskNotifyGive(s_task);

    ESP_LOGI(
        TAG,
        "BNO086 iniciado por H_INTN: INT=GPIO%d RESET_N=GPIO%d",
        (int)BNO086_INT_GPIO,
        (int)BNO086_RESET_GPIO);

    return ESP_OK;
}
