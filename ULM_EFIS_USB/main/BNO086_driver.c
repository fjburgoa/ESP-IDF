/**
 * @file BNO086_driver.c
 * @brief Transporte I2C + protocolo SHTP/SH-2 del BNO086.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "BNO086_driver.h"
#include "EFIS_I2C.h"
#include "config.h"

#define SHTP_CHANNEL_EXECUTABLE 1U
#define SHTP_CHANNEL_CONTROL    2U

#define SHTP_REPORT_SET_FEATURE 0xFDU

#define SHTP_CHANNEL_COUNT 6U
#define SHTP_TX_MAX_PACKET_SIZE 128U

/*
 * En I2C cada lectura parcial vuelve a incluir la cabecera SHTP.
 * 28 bytes de payload + 4 bytes de cabecera = 32 bytes por transacción.
 */
#define SHTP_I2C_CHUNK_DATA_SIZE 28U

static const char *TAG = "BNO086_DRV";

static i2c_master_dev_handle_t s_device = NULL;
static uint8_t s_sequence[SHTP_CHANNEL_COUNT];

static void write_u32_le(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static esp_err_t shtp_send(
    uint8_t channel,
    const uint8_t *payload,
    uint16_t payload_len)
{
    uint8_t packet[SHTP_TX_MAX_PACKET_SIZE];

    const uint16_t packet_len = payload_len + 4U;

    if ((payload == NULL) ||
        (channel >= SHTP_CHANNEL_COUNT) ||
        (packet_len > sizeof(packet)))
    {
        return ESP_ERR_INVALID_ARG;
    }

    packet[0] = (uint8_t)(packet_len & 0xFFU);
    packet[1] = (uint8_t)((packet_len >> 8) & 0x7FU);
    packet[2] = channel;
    packet[3] = s_sequence[channel]++;

    memcpy(&packet[4], payload, payload_len);

    return i2c_master_transmit(
        s_device,
        packet,
        packet_len,
        EFIS_I2C_TIMEOUT_MS);
}

/* -------------------------------------------------------------------------- */
/* Recepción SHTP                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t bno086_driver_receive_packet(
    uint8_t *packet,
    size_t capacity,
    uint16_t *packet_len)
{
    if ((packet == NULL) ||
        (packet_len == NULL) ||
        (capacity < 4U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t header[4];

    esp_err_t err = i2c_master_receive(
        s_device,
        header,
        sizeof(header),
        BNO086_I2C_READ_TIMEOUT_MS);

    if (err != ESP_OK)
    {
        return err;
    }

    uint16_t total_len =
        (uint16_t)header[0] |
        ((uint16_t)header[1] << 8);

    total_len &= 0x7FFFU;

    /*
     * El BNO086 devuelve cabecera 0000 cuando no hay datos disponibles.
     */
    if (total_len == 0U)
    {
        *packet_len = 0U;
        return ESP_ERR_NOT_FOUND;
    }

    if (total_len < 4U)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    memcpy(packet, header, 4U);

    size_t stored = 4U;
    uint16_t payload_remaining = total_len - 4U;

    while (payload_remaining > 0U)
    {
        uint16_t payload_chunk = payload_remaining;

        if (payload_chunk > SHTP_I2C_CHUNK_DATA_SIZE)
        {
            payload_chunk = SHTP_I2C_CHUNK_DATA_SIZE;
        }

        uint8_t chunk[4U + SHTP_I2C_CHUNK_DATA_SIZE];

        /*
         * Cada nueva lectura I2C comienza otra vez por la cabecera SHTP.
         * Por eso pedimos 4 + N y descartamos chunk[0..3].
         */
        err = i2c_master_receive(
            s_device,
            chunk,
            payload_chunk + 4U,
            BNO086_I2C_READ_TIMEOUT_MS);

        if (err != ESP_OK)
        {
            return err;
        }

        size_t bytes_to_store = payload_chunk;

        if ((stored + bytes_to_store) > capacity)
        {
            if (stored < capacity)
            {
                bytes_to_store = capacity - stored;
            }
            else
            {
                bytes_to_store = 0U;
            }
        }

        if (bytes_to_store > 0U)
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
     * Aunque un paquete de arranque sea mayor que el buffer local,
     * ya se ha consumido completamente del BNO086.
     */
    if (total_len > capacity)
    {
        ESP_LOGD(
            TAG,
            "Paquete SHTP %u bytes; almacenados %u",
            (unsigned)total_len,
            (unsigned)stored);
    }

    *packet_len = (uint16_t)stored;

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Configuración SH-2                                                          */
/* -------------------------------------------------------------------------- */

static esp_err_t bno086_soft_reset(void)
{
    const uint8_t reset_command = 0x01U;

    ESP_RETURN_ON_ERROR(
        shtp_send(
            SHTP_CHANNEL_EXECUTABLE,
            &reset_command,
            sizeof(reset_command)),
        TAG,
        "No se pudo enviar soft reset");

    vTaskDelay(pdMS_TO_TICKS(BNO086_RESET_DELAY_MS));

    /*
     * Tras el reset aparecen paquetes de advertisement/arranque.
     * Se consumen antes de activar los reports.
     */
    uint8_t packet[BNO086_PACKET_BUFFER_SIZE];
    uint16_t packet_len = 0U;

    for (uint32_t i = 0U; i < BNO086_STARTUP_FLUSH_PACKETS; ++i)
    {
        const esp_err_t err = bno086_driver_receive_packet(
            packet,
            sizeof(packet),
            &packet_len);

        if ((err == ESP_ERR_NOT_FOUND) ||
            (err == ESP_ERR_TIMEOUT))
        {
            break;
        }

        if (err != ESP_OK)
        {
            ESP_LOGW(
                TAG,
                "Error vaciando SHTP tras reset: %s",
                esp_err_to_name(err));

            break;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    return ESP_OK;
}

static esp_err_t bno086_enable_report(
    uint8_t report_id,
    uint32_t interval_us)
{
    uint8_t payload[17] = {0};

    payload[0] = SHTP_REPORT_SET_FEATURE;
    payload[1] = report_id;

    /*
     * Report interval en microsegundos, little-endian.
     */
    write_u32_le(&payload[5], interval_us);

    return shtp_send(
        SHTP_CHANNEL_CONTROL,
        payload,
        sizeof(payload));
}

esp_err_t bno086_driver_init(void)
{
    if (s_device != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(
        efis_i2c_probe(BNO086_I2C_ADDRESS),
        TAG,
        "BNO086 no detectado");

    ESP_RETURN_ON_ERROR(
        efis_i2c_add_device(
            BNO086_I2C_ADDRESS,
            &s_device),
        TAG,
        "No se pudo registrar BNO086");

    ESP_LOGI(
        TAG,
        "BNO086 detectado en 0x%02X",
        BNO086_I2C_ADDRESS);

    ESP_RETURN_ON_ERROR(
        bno086_soft_reset(),
        TAG,
        "Error durante soft reset");

    ESP_RETURN_ON_ERROR(
        bno086_enable_report(
            BNO086_REPORT_ACCELEROMETER,
            BNO086_REPORT_INTERVAL_US),
        TAG,
        "No se pudo activar Accelerometer");

    ESP_RETURN_ON_ERROR(
        bno086_enable_report(
            BNO086_REPORT_GYROSCOPE,
            BNO086_REPORT_INTERVAL_US),
        TAG,
        "No se pudo activar Gyroscope");

    ESP_RETURN_ON_ERROR(
        bno086_enable_report(
            BNO086_REPORT_LINEAR_ACCELERATION,
            BNO086_REPORT_INTERVAL_US),
        TAG,
        "No se pudo activar Linear Acceleration");

    ESP_RETURN_ON_ERROR(
        bno086_enable_report(
            BNO086_REPORT_GRAVITY,
            BNO086_REPORT_INTERVAL_US),
        TAG,
        "No se pudo activar Gravity");

    ESP_RETURN_ON_ERROR(
        bno086_enable_report(
            BNO086_REPORT_GAME_ROTATION_VECTOR,
            BNO086_REPORT_INTERVAL_US),
        TAG,
        "No se pudo activar Game Rotation Vector");

    ESP_LOGI(
        TAG,
        "SH-2: ACC + GYR + LINEAR + GRAVITY + GAME_RV @ %u Hz",
        (unsigned)(1000000U / BNO086_REPORT_INTERVAL_US));

    return ESP_OK;
}
