
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/i2c_master.h"

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO 8
#define I2C_SCL_GPIO 9
#define I2C_FREQUENCY_HZ 400000

#define BNO086_I2C_ADDRESS 0x4B

#define SHTP_CHANNEL_EXECUTABLE 1
#define SHTP_CHANNEL_CONTROL 2
#define SHTP_CHANNEL_REPORTS 3

#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE 0xFD

#define SENSOR_REPORT_ACCELEROMETER 0x01
#define SENSOR_REPORT_GYROSCOPE 0x02
#define SENSOR_REPORT_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORT_GRAVITY 0x06
#define SENSOR_REPORT_GAME_ROTATION_VECTOR 0x08

#define REPORT_INTERVAL_US 40000U
#define SHTP_MAX_PACKET_SIZE 128
#define SHTP_I2C_CHUNK_DATA_SIZE 28

#define STANDARD_GRAVITY_MS2 9.80665f
#define RAD_TO_DEG 57.29577951308232f

static const char *TAG = "BNO086";

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_bno;

static uint8_t s_seq[6];

static float ax, ay, az;
static float gx, gy, gz;

static float gravity_x, gravity_y, gravity_z;
static float linear_x, linear_y, linear_z;

static float quat_i, quat_j, quat_k, quat_real;
static float roll_deg, pitch_deg, yaw_deg;

static uint8_t acc_accuracy;
static uint8_t gyro_accuracy;
static uint8_t gravity_accuracy;
static uint8_t linear_accuracy;
static uint8_t quat_accuracy;

static bool have_acc;
static bool have_gyro;
static bool have_gravity;
static bool have_linear;
static bool have_quat;

static int16_t read_i16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void write_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static const char *accuracy_str(uint8_t a)
{
    switch (a)
    {
    case 0:
        return "UNRELIABLE";
    case 1:
        return "LOW";
    case 2:
        return "MEDIUM";
    case 3:
        return "HIGH";
    default:
        return "?";
    }
}

static void quaternion_to_euler_aircraft(float sensor_i,
                                         float sensor_j,
                                         float sensor_k,
                                         float sensor_real,
                                         float *roll_out_deg,
                                         float *pitch_out_deg,
                                         float *yaw_out_deg)
{
    /*
     * El montaje validado experimentalmente intercambia X <-> Y y
     * cambia el signo de Z para las velocidades angulares:
     *
     *     X_avion = +Y_sensor
     *     Y_avion = +X_sensor
     *     Z_avion = -Z_sensor
     *
     * Esta transformación es una rotación propia (det = +1).
     * El quaternion equivalente a esa rotación fija de ejes se aplica
     * a la orientación reportada por SH-2.
     *
     * Para esta primera prueba interesa sobre todo verificar que:
     *   roll  responde al giro físico sobre X_avion,
     *   pitch responde al giro físico sobre Y_avion,
     *   yaw   responde al giro físico sobre Z_avion.
     */

    /*
     * Quaternion de cambio de base correspondiente a:
     * R = [ 0  1  0
     *       1  0  0
     *       0  0 -1 ]
     *
     * Es una rotación de 180 grados alrededor del eje (1,1,0)/sqrt(2):
     * q_mount = (x,y,z,w) = (sqrt(1/2), sqrt(1/2), 0, 0)
     */
    const float s = 0.7071067811865476f;

    /*
     * q_aircraft = q_mount * q_sensor * conjugate(q_mount)
     *
     * La expresión está desarrollada de forma explícita para no añadir
     * infraestructura innecesaria en este programa de prueba.
     */
    float qi = sensor_j;
    float qj = sensor_i;
    float qk = -sensor_k;
    float qr = sensor_real;

    /*
     * Normalización defensiva.
     */
    float norm = sqrtf(qi * qi + qj * qj + qk * qk + qr * qr);

    if (norm > 0.0f)
    {
        qi /= norm;
        qj /= norm;
        qk /= norm;
        qr /= norm;
    }

    (void)s;

    /*
     * Convención Tait-Bryan Z-Y-X:
     * roll  alrededor de X,
     * pitch alrededor de Y,
     * yaw   alrededor de Z.
     */
    float sin_roll_cos_pitch =
        2.0f * (qr * qi + qj * qk);

    float cos_roll_cos_pitch =
        1.0f - 2.0f * (qi * qi + qj * qj);

    float roll_rad =
        atan2f(sin_roll_cos_pitch, cos_roll_cos_pitch);

    float sin_pitch =
        2.0f * (qr * qj - qk * qi);

    if (sin_pitch > 1.0f)
    {
        sin_pitch = 1.0f;
    }
    else if (sin_pitch < -1.0f)
    {
        sin_pitch = -1.0f;
    }

    float pitch_rad = asinf(sin_pitch);

    float sin_yaw_cos_pitch =
        2.0f * (qr * qk + qi * qj);

    float cos_yaw_cos_pitch =
        1.0f - 2.0f * (qj * qj + qk * qk);

    float yaw_rad =
        atan2f(sin_yaw_cos_pitch, cos_yaw_cos_pitch);

    *roll_out_deg = roll_rad * RAD_TO_DEG;
    *pitch_out_deg = pitch_rad * RAD_TO_DEG;
    *yaw_out_deg = yaw_rad * RAD_TO_DEG;
}

static esp_err_t i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK)
    {
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BNO086_I2C_ADDRESS,
        .scl_speed_hz = I2C_FREQUENCY_HZ,
    };

    return i2c_master_bus_add_device(s_bus, &dev_cfg, &s_bno);
}

static esp_err_t shtp_send(uint8_t channel,
                           const uint8_t *payload,
                           uint16_t payload_len)
{
    uint8_t packet[SHTP_MAX_PACKET_SIZE];
    uint16_t packet_len = payload_len + 4U;

    if (channel >= 6 || packet_len > sizeof(packet))
    {
        return ESP_ERR_INVALID_ARG;
    }

    packet[0] = (uint8_t)(packet_len & 0xFF);
    packet[1] = (uint8_t)((packet_len >> 8) & 0x7F);
    packet[2] = channel;
    packet[3] = s_seq[channel]++;

    memcpy(&packet[4], payload, payload_len);

    return i2c_master_transmit(s_bno, packet, packet_len, 100);
}

static esp_err_t shtp_receive(uint8_t *packet,
                              size_t capacity,
                              uint16_t *packet_len)
{
    uint8_t header[4];

    esp_err_t err = i2c_master_receive(
        s_bno,
        header,
        sizeof(header),
        20);

    if (err != ESP_OK)
    {
        return err;
    }

    uint16_t total_len =
        (uint16_t)header[0] |
        ((uint16_t)header[1] << 8);

    total_len &= 0x7FFF;

    if (total_len == 0)
    {
        *packet_len = 0;
        return ESP_ERR_NOT_FOUND;
    }

    if (total_len < 4)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    /*
     * Guardamos la primera cabecera SHTP.
     */
    size_t stored = 0;

    if (capacity >= 4)
    {
        memcpy(packet, header, 4);
        stored = 4;
    }

    /*
     * En el BNO08x, una lectura I2C parcial NO continúa simplemente
     * donde terminó la anterior. Cada nueva transacción vuelve a
     * comenzar con una cabecera SHTP de 4 bytes.
     *
     * Por eso se lee el payload por bloques:
     *
     *     [header 4 bytes] + [hasta 28 bytes de payload]
     *
     * La cabecera de cada bloque se descarta. Esto permite vaciar
     * también los paquetes grandes de arranque/advertisement, que
     * pueden superar nuestro buffer local de 128 bytes.
     */
    uint16_t payload_remaining = total_len - 4U;

    while (payload_remaining > 0)
    {
        uint16_t payload_chunk = payload_remaining;

        if (payload_chunk > SHTP_I2C_CHUNK_DATA_SIZE)
        {
            payload_chunk = SHTP_I2C_CHUNK_DATA_SIZE;
        }

        uint8_t chunk[4 + SHTP_I2C_CHUNK_DATA_SIZE];

        err = i2c_master_receive(
            s_bno,
            chunk,
            payload_chunk + 4U,
            50);

        if (err != ESP_OK)
        {
            return err;
        }

        size_t free_space = 0;

        if (stored < capacity)
        {
            free_space = capacity - stored;
        }

        size_t bytes_to_store = payload_chunk;

        if (bytes_to_store > free_space)
        {
            bytes_to_store = free_space;
        }

        if (bytes_to_store > 0)
        {
            memcpy(
                &packet[stored],
                &chunk[4],
                bytes_to_store);

            stored += bytes_to_store;
        }

        payload_remaining -= payload_chunk;
    }

    /*
     * Si el paquete era mayor que el buffer, ya se ha vaciado por
     * completo del BNO086. Para los paquetes de sensores normales
     * 128 bytes son más que suficientes.
     */
    if (total_len > capacity)
    {
        ESP_LOGD(
            TAG,
            "Paquete SHTP de %u bytes truncado a %u bytes",
            total_len,
            (unsigned)stored);
    }

    *packet_len = (uint16_t)stored;

    return ESP_OK;
}

static esp_err_t bno086_soft_reset(void)
{
    const uint8_t reset_cmd = 0x01;

    esp_err_t err = shtp_send(
        SHTP_CHANNEL_EXECUTABLE,
        &reset_cmd,
        1);

    if (err != ESP_OK)
    {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(300));

    uint8_t packet[SHTP_MAX_PACKET_SIZE];
    uint16_t len;

    for (int i = 0; i < 20; i++)
    {
        esp_err_t rx_err = shtp_receive(
            packet,
            sizeof(packet),
            &len);

        if (rx_err == ESP_ERR_NOT_FOUND ||
            rx_err == ESP_ERR_TIMEOUT)
        {
            break;
        }

        if (rx_err != ESP_OK)
        {
            ESP_LOGW(
                TAG,
                "Error vaciando paquetes tras reset: %s",
                esp_err_to_name(rx_err));

            break;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    return ESP_OK;
}

static esp_err_t bno086_enable_report(uint8_t report_id,
                                      uint32_t interval_us)
{
    uint8_t payload[17] = {0};

    payload[0] = SHTP_REPORT_SET_FEATURE;
    payload[1] = report_id;

    write_u32_le(&payload[5], interval_us);

    return shtp_send(
        SHTP_CHANNEL_CONTROL,
        payload,
        sizeof(payload));
}

static void process_sensor_report(const uint8_t *report)
{
    uint8_t report_id = report[0];
    uint8_t accuracy = report[2] & 0x03;

    if (report_id == SENSOR_REPORT_ACCELEROMETER ||
        report_id == SENSOR_REPORT_GYROSCOPE ||
        report_id == SENSOR_REPORT_LINEAR_ACCELERATION ||
        report_id == SENSOR_REPORT_GRAVITY)
    {
        int16_t raw_x = read_i16_le(&report[4]);
        int16_t raw_y = read_i16_le(&report[6]);
        int16_t raw_z = read_i16_le(&report[8]);

        if (report_id == SENSOR_REPORT_ACCELEROMETER)
        {
            /*
             * Transformación validada experimentalmente para este montaje.
             */
            ay = -(float)raw_x / 256.0f; /* Q8, m/s² */
            ax = -(float)raw_y / 256.0f;
            az = -(float)raw_z / 256.0f;

            acc_accuracy = accuracy;
            have_acc = true;
        }
        else if (report_id == SENSOR_REPORT_GYROSCOPE)
        {
            /*
             * Transformación validada experimentalmente para este montaje.
             */
            gy = (float)raw_x / 512.0f; /* Q9, rad/s */
            gx = (float)raw_y / 512.0f;
            gz = -(float)raw_z / 512.0f;

            gyro_accuracy = accuracy;
            have_gyro = true;
        }
        else if (report_id == SENSOR_REPORT_LINEAR_ACCELERATION)
        {
            /*
             * Vector lineal: misma transformación usada para ACC.
             */
            linear_y = -(float)raw_x / 256.0f; /* Q8, m/s² */
            linear_x = -(float)raw_y / 256.0f;
            linear_z = -(float)raw_z / 256.0f;

            linear_accuracy = accuracy;
            have_linear = true;
        }
        else if (report_id == SENSOR_REPORT_GRAVITY)
        {
            /*
             * Vector gravedad: misma transformación usada para ACC.
             */
            gravity_y = -(float)raw_x / 256.0f; /* Q8, m/s² */
            gravity_x = -(float)raw_y / 256.0f;
            gravity_z = -(float)raw_z / 256.0f;

            gravity_accuracy = accuracy;
            have_gravity = true;
        }
    }
    else if (report_id == SENSOR_REPORT_GAME_ROTATION_VECTOR)
    {
        /*
         * Game Rotation Vector:
         * i, j, k, real en Q14.
         */
        int16_t raw_i = read_i16_le(&report[4]);
        int16_t raw_j = read_i16_le(&report[6]);
        int16_t raw_k = read_i16_le(&report[8]);
        int16_t raw_real = read_i16_le(&report[10]);

        quat_i = (float)raw_i / 16384.0f;
        quat_j = (float)raw_j / 16384.0f;
        quat_k = (float)raw_k / 16384.0f;
        quat_real = (float)raw_real / 16384.0f;

        quaternion_to_euler_aircraft(
            quat_i,
            quat_j,
            quat_k,
            quat_real,
            &roll_deg,
            &pitch_deg,
            &yaw_deg);

        quat_accuracy = accuracy;
        have_quat = true;
    }
}

static size_t sensor_report_size(uint8_t report_id)
{
    switch (report_id)
    {
    case SENSOR_REPORT_ACCELEROMETER:
    case SENSOR_REPORT_GYROSCOPE:
    case SENSOR_REPORT_LINEAR_ACCELERATION:
    case SENSOR_REPORT_GRAVITY:
        return 10U;

    case SENSOR_REPORT_GAME_ROTATION_VECTOR:
        return 12U;

    default:
        return 0U;
    }
}

static void process_packet(const uint8_t *packet, uint16_t packet_len)
{
    if (packet_len < 9)
    {
        return;
    }

    if (packet[2] != SHTP_CHANNEL_REPORTS)
    {
        return;
    }

    const uint8_t *payload = &packet[4];
    size_t payload_len = packet_len - 4U;

    if (payload[0] != SHTP_REPORT_BASE_TIMESTAMP)
    {
        return;
    }

    size_t offset = 5U;

    while (offset < payload_len)
    {
        uint8_t report_id = payload[offset];
        size_t report_size = sensor_report_size(report_id);

        if (report_size == 0U)
        {
            break;
        }

        if ((offset + report_size) > payload_len)
        {
            break;
        }

        process_sensor_report(&payload[offset]);
        offset += report_size;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Prueba BNO086 - ESP-IDF v6.1");
    ESP_LOGI(TAG, "SDA=GPIO%d, SCL=GPIO%d, direccion=0x%02X",
             I2C_SDA_GPIO,
             I2C_SCL_GPIO,
             BNO086_I2C_ADDRESS);

    ESP_ERROR_CHECK(i2c_init());
    ESP_ERROR_CHECK(i2c_master_probe(s_bus, BNO086_I2C_ADDRESS, 100));

    ESP_LOGI(TAG, "BNO086 detectado");

    ESP_ERROR_CHECK(bno086_soft_reset());

    ESP_ERROR_CHECK(
        bno086_enable_report(
            SENSOR_REPORT_ACCELEROMETER,
            REPORT_INTERVAL_US));

    ESP_ERROR_CHECK(
        bno086_enable_report(
            SENSOR_REPORT_GYROSCOPE,
            REPORT_INTERVAL_US));

    ESP_ERROR_CHECK(
        bno086_enable_report(
            SENSOR_REPORT_LINEAR_ACCELERATION,
            REPORT_INTERVAL_US));

    ESP_ERROR_CHECK(
        bno086_enable_report(
            SENSOR_REPORT_GRAVITY,
            REPORT_INTERVAL_US));

    ESP_ERROR_CHECK(
        bno086_enable_report(
            SENSOR_REPORT_GAME_ROTATION_VECTOR,
            REPORT_INTERVAL_US));

    ESP_LOGI(TAG, "Modo: I2C + SHTP/SH-2");
    ESP_LOGI(TAG, "Reports: ACC + GYR + GRAVITY + LINEAR + GAME ROTATION VECTOR");
    ESP_LOGI(TAG, "Frecuencia solicitada: 25 Hz");
    ESP_LOGI(TAG, "El BNO086 no usa AMG/IMUPLUS/NDOF como el BNO055");

    uint8_t packet[SHTP_MAX_PACKET_SIZE];
    uint16_t packet_len;

    TickType_t last_print = xTaskGetTickCount();

    while (1)
    {
        esp_err_t err = shtp_receive(
            packet,
            sizeof(packet),
            &packet_len);

        if (err == ESP_OK)
        {
            process_packet(packet, packet_len);
        }
        else if (err != ESP_ERR_NOT_FOUND &&
                 err != ESP_ERR_TIMEOUT)
        {
            ESP_LOGW(TAG, "Error SHTP: %s", esp_err_to_name(err));
        }

        if ((xTaskGetTickCount() - last_print) >= pdMS_TO_TICKS(100))
        {
            last_print = xTaskGetTickCount();

            if (have_acc && have_gyro && have_gravity && have_linear && have_quat)
            {
                float norm_g =
                    sqrtf(ax * ax + ay * ay + az * az) /
                    STANDARD_GRAVITY_MS2;

                printf(
                    "ACC X=%+6.2f Y=%+6.2f Z=%+6.2f |A|=%4.2fg | "
                    "GRV X=%+6.2f Y=%+6.2f Z=%+6.2f | "
                    "LIN X=%+6.2f Y=%+6.2f Z=%+6.2f | "
                    "GYR X=%+5.1f Y=%+5.1f Z=%+5.1f deg/s | "
                    "RPY R=%+6.1f P=%+6.1f Y=%+6.1f deg\n",
                    ax,
                    ay,
                    az,
                    norm_g,
                    gravity_x,
                    gravity_y,
                    gravity_z,
                    linear_x,
                    linear_y,
                    linear_z,
                    gx * RAD_TO_DEG,
                    gy * RAD_TO_DEG,
                    gz * RAD_TO_DEG,
                    roll_deg,
                    pitch_deg,
                    yaw_deg);
            }
            else
            {
                ESP_LOGI(TAG, "Esperando ACC/GYR/GRAVITY/LINEAR/GAME_RV...");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
