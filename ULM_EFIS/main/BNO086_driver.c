/**
 * @file BNO086_driver.c
 * @brief Driver I2C/SHTP/SH-2 del BNO086.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "BNO086_driver.h"
#include "EFIS_I2C.h"
#include "config.h"

#define SHTP_CHANNEL_CONTROL 2U
#define SHTP_REPORT_SET_FEATURE 0xFDU
#define SHTP_CHANNEL_COUNT 6U
#define SHTP_TX_MAX_PACKET_SIZE 128U

/*
 * En I2C cada lectura parcial del BNO086 vuelve a comenzar con
 * los 4 bytes de cabecera SHTP.
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

static esp_err_t bno086_wait_int_low(uint32_t timeout_ms)
{
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

    while (gpio_get_level(BNO086_INT_GPIO) != 0)
    {
        if ((xTaskGetTickCount() - start) >= timeout)
        {
            return ESP_ERR_TIMEOUT;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return ESP_OK;
}

static esp_err_t bno086_configure_gpio(void)
{
    const gpio_config_t int_cfg = {
        .pin_bit_mask = 1ULL << BNO086_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    ESP_RETURN_ON_ERROR(
        gpio_config(&int_cfg),
        TAG,
        "No se pudo configurar H_INTN");

    const gpio_config_t reset_cfg = {
        .pin_bit_mask = 1ULL << BNO086_RESET_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_RETURN_ON_ERROR(
        gpio_config(&reset_cfg),
        TAG,
        "No se pudo configurar RESET_N");

    return ESP_OK;
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
        (packet_len > sizeof(packet)) ||
        (s_device == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    packet[0] = (uint8_t)(packet_len & 0xFFU);
    packet[1] = (uint8_t)((packet_len >> 8) & 0x7FU);
    packet[2] = channel;
    packet[3] = s_sequence[channel]++;

    memcpy(&packet[4], payload, payload_len);

    ESP_RETURN_ON_ERROR(
        efis_i2c_lock(EFIS_I2C_MUTEX_TIMEOUT_MS),
        TAG,
        "Timeout esperando bus I2C");

    const esp_err_t err = i2c_master_transmit(
        s_device,
        packet,
        packet_len,
        EFIS_I2C_TIMEOUT_MS);

    efis_i2c_unlock();

    return err;
}

esp_err_t bno086_driver_receive_packet(
    uint8_t *packet,
    size_t capacity,
    uint16_t *packet_len)
{
    if ((packet == NULL) ||
        (packet_len == NULL) ||
        (capacity < 4U) ||
        (s_device == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * El BNO086 no se sondea: solo se inicia una lectura cuando H_INTN
     * indica que el dispositivo tiene datos pendientes.
     */
    if (gpio_get_level(BNO086_INT_GPIO) != 0)
    {
        *packet_len = 0U;
        return ESP_ERR_NOT_FOUND;
    }

    ESP_RETURN_ON_ERROR(
        efis_i2c_lock(EFIS_I2C_MUTEX_TIMEOUT_MS),
        TAG,
        "Timeout esperando bus I2C");

    uint8_t header[4];

    esp_err_t err = i2c_master_receive(
        s_device,
        header,
        sizeof(header),
        BNO086_I2C_READ_TIMEOUT_MS);

    if (err != ESP_OK)
    {
        efis_i2c_unlock();
        return err;
    }

    uint16_t total_len =
        (uint16_t)header[0] |
        ((uint16_t)header[1] << 8);

    total_len &= 0x7FFFU;

    if (total_len == 0U)
    {
        efis_i2c_unlock();
        *packet_len = 0U;
        return ESP_ERR_NOT_FOUND;
    }

    if (total_len < 4U)
    {
        efis_i2c_unlock();
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

        err = i2c_master_receive(
            s_device,
            chunk,
            payload_chunk + 4U,
            BNO086_I2C_READ_TIMEOUT_MS);

        if (err != ESP_OK)
        {
            efis_i2c_unlock();
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

    efis_i2c_unlock();

    *packet_len = (uint16_t)stored;
    return ESP_OK;
}

static esp_err_t bno086_flush_startup_packets(void)
{
    uint8_t packet[BNO086_PACKET_BUFFER_SIZE];
    uint16_t packet_len = 0U;

    for (uint32_t i = 0U; i < BNO086_STARTUP_FLUSH_PACKETS; ++i)
    {
        if (gpio_get_level(BNO086_INT_GPIO) != 0)
        {
            return ESP_OK;
        }

        const esp_err_t err = bno086_driver_receive_packet(
            packet,
            sizeof(packet),
            &packet_len);

        if (err == ESP_ERR_NOT_FOUND)
        {
            return ESP_OK;
        }

        if (err != ESP_OK)
        {
            return err;
        }

        vTaskDelay(pdMS_TO_TICKS(BNO086_STARTUP_PACKET_GAP_MS));
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
    write_u32_le(&payload[5], interval_us);

    return shtp_send(
        SHTP_CHANNEL_CONTROL,
        payload,
        sizeof(payload));
}

static esp_err_t bno086_enable_all_reports(void)
{
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

    return ESP_OK;
}

static esp_err_t bno086_hardware_reset_and_configure(bool reset_i2c_bus)
{
    /* Mantener el BNO086 completamente parado antes de recuperar el bus. */
    gpio_set_level(BNO086_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(BNO086_RESET_LOW_MS));

    if (reset_i2c_bus)
    {
        ESP_RETURN_ON_ERROR(
            efis_i2c_reset_bus(),
            TAG,
            "No se pudo recuperar el bus I2C");
    }

    memset(s_sequence, 0, sizeof(s_sequence));

    gpio_set_level(BNO086_RESET_GPIO, 1);

    ESP_RETURN_ON_ERROR(
        bno086_wait_int_low(BNO086_BOOT_TIMEOUT_MS),
        TAG,
        "H_INTN no se activo tras RESET_N");

    /*
     * Vaciamos únicamente mientras H_INTN permanezca bajo. No se hacen
     * lecturas especulativas/polling del BNO086.
     */
    const esp_err_t flush_err = bno086_flush_startup_packets();
    if ((flush_err != ESP_OK) && (flush_err != ESP_ERR_NOT_FOUND))
    {
        ESP_LOGW(
            TAG,
            "Error vaciando paquetes SHTP de arranque: %s",
            esp_err_to_name(flush_err));
        return flush_err;
    }

    ESP_RETURN_ON_ERROR(
        bno086_enable_all_reports(),
        TAG,
        "No se pudieron configurar los reports SH-2");

    return ESP_OK;
}

esp_err_t bno086_driver_init(void)
{
    if (s_device != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(
        bno086_configure_gpio(),
        TAG,
        "No se pudieron configurar GPIO del BNO086");

    /* El BNO086 permanece en reset mientras se registra en el bus. */
    gpio_set_level(BNO086_RESET_GPIO, 0);

    ESP_RETURN_ON_ERROR(
        efis_i2c_add_device_ex(
            BNO086_I2C_ADDRESS,
            BNO086_I2C_FREQ_HZ,
            BNO086_SCL_WAIT_US,
            &s_device),
        TAG,
        "No se pudo registrar BNO086");

    esp_err_t err = ESP_FAIL;

    for (uint32_t attempt = 1U; attempt <= 3U; ++attempt)
    {
        err = bno086_hardware_reset_and_configure(attempt > 1U);

        if (err == ESP_OK)
        {
            break;
        }

        ESP_LOGW(
            TAG,
            "Intento de inicio BNO086 %u/3 fallido: %s",
            (unsigned)attempt,
            esp_err_to_name(err));

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (err != ESP_OK)
    {
        return err;
    }

    ESP_LOGI(
        TAG,
        "BNO086 detectado en 0x%02X, INT=GPIO%d, RESET_N=GPIO%d, I2C=%u Hz",
        BNO086_I2C_ADDRESS,
        (int)BNO086_INT_GPIO,
        (int)BNO086_RESET_GPIO,
        (unsigned)BNO086_I2C_FREQ_HZ);

    ESP_LOGI(
        TAG,
        "SH-2: ACC + GYR + LINEAR + GRAVITY + GAME_RV @ %u Hz",
        (unsigned)(1000000U / BNO086_REPORT_INTERVAL_US));

    return ESP_OK;
}

esp_err_t bno086_driver_recover(void)
{
    if (s_device == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG, "Recuperando BNO086 y bus I2C...");

    vTaskDelay(pdMS_TO_TICKS(BNO086_RECOVERY_DELAY_MS));

    const esp_err_t err = bno086_hardware_reset_and_configure(true);

    if (err == ESP_OK)
    {
        ESP_LOGW(TAG, "BNO086 recuperado correctamente");
    }

    return err;
}
