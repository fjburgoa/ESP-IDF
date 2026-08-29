/**
 * @file GPS.c
 * @brief Driver GNSS para ESP32-S3: UART, UBX, NMEA GGA/RMC y publicación.
 *
 * CONEXIONES
 * ----------
 *
 *   ESP32-S3                   GPS
 *   --------                   ---
 *   GPIO47  UART1 RX  <------- TX
 *   GPIO45  UART1 TX  -------> RX
 *   GND               -------- GND
 *   3V3/5V            -------- VCC, según el módulo
 *
 * CONFIGURACIÓN OBJETIVO
 * ----------------------
 *
 *   - UART: 115200 baud
 *   - Frecuencia de navegación: 5 Hz
 *   - Sentencias NMEA activas: GGA y RMC
 *
 * La configuración se intenta guardar, pero el driver vuelve a aplicarla en
 * cada arranque para soportar módulos sin EEPROM/Flash o receptores compatibles
 * que no implementen persistencia UBX.
 */

#include "GPS.h"

#include <ctype.h>
#include <inttypes.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"

#include "config.h"

/* -------------------------------------------------------------------------- */
/* UART y tarea                                                               */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* UBX                                                                        */
/* -------------------------------------------------------------------------- */

#define UBX_SYNC_1 0xB5U
#define UBX_SYNC_2 0x62U

#define UBX_CLASS_ACK 0x05U
#define UBX_ID_ACK_NAK 0x00U
#define UBX_ID_ACK_ACK 0x01U

#define UBX_CLASS_CFG 0x06U
#define UBX_ID_CFG_PRT 0x00U
#define UBX_ID_CFG_MSG 0x01U
#define UBX_ID_CFG_CFG 0x09U
#define UBX_ID_CFG_RATE 0x08U

#define NMEA_CLASS 0xF0U
#define NMEA_ID_GGA 0x00U
#define NMEA_ID_GLL 0x01U
#define NMEA_ID_GSA 0x02U
#define NMEA_ID_GSV 0x03U
#define NMEA_ID_RMC 0x04U
#define NMEA_ID_VTG 0x05U
#define NMEA_ID_GRS 0x06U
#define NMEA_ID_GST 0x07U
#define NMEA_ID_ZDA 0x08U
#define NMEA_ID_GBS 0x09U
#define NMEA_ID_DTM 0x0AU

/* -------------------------------------------------------------------------- */

static const char *TAG = "GPS";

static gps_data_t s_data = {0};
static portMUX_TYPE s_data_mux = portMUX_INITIALIZER_UNLOCKED;

/* -------------------------------------------------------------------------- */
/* Utilidades generales                                                       */
/* -------------------------------------------------------------------------- */

static void gps_publish_diagnostic(
    uint32_t baud_rate,
    bool connected)
{
    portENTER_CRITICAL(&s_data_mux);

    s_data.baud_rate = baud_rate;
    s_data.connected = connected;

    portEXIT_CRITICAL(&s_data_mux);
}

/* -------------------------------------------------------------------------- */

static esp_err_t gps_uart_init(uint32_t baud_rate)
{
    const uart_config_t config = {
        .baud_rate = (int)baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(
        GPS_UART_PORT,
        GPS_UART_RX_BUFFER_SIZE,
        GPS_UART_TX_BUFFER_SIZE,
        0,
        NULL,
        0);

    if (err != ESP_OK)
    {
        return err;
    }

    err = uart_param_config(
        GPS_UART_PORT,
        &config);

    if (err != ESP_OK)
    {
        uart_driver_delete(GPS_UART_PORT);
        return err;
    }

    err = uart_set_pin(
        GPS_UART_PORT,
        GPS_UART_TX_GPIO,
        GPS_UART_RX_GPIO,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);

    if (err != ESP_OK)
    {
        uart_driver_delete(GPS_UART_PORT);
        return err;
    }

    gps_publish_diagnostic(baud_rate, false);

    ESP_LOGI(
        TAG,
        "UART%d: TX=GPIO%d RX=GPIO%d, %" PRIu32 " baud",
        GPS_UART_PORT,
        GPS_UART_TX_GPIO,
        GPS_UART_RX_GPIO,
        baud_rate);

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

static esp_err_t gps_uart_set_baud(uint32_t baud_rate)
{
    esp_err_t err = uart_set_baudrate(
        GPS_UART_PORT,
        baud_rate);

    if (err == ESP_OK)
    {
        uart_flush_input(GPS_UART_PORT);
        gps_publish_diagnostic(baud_rate, s_data.connected);

        ESP_LOGI(
            TAG,
            "UART cambiada a %" PRIu32 " baud",
            baud_rate);
    }

    return err;
}

/* -------------------------------------------------------------------------- */

static bool gps_is_printable_nmea(uint8_t c)
{
    return ((c >= 0x20U) && (c <= 0x7EU)) ||
           (c == '\r') ||
           (c == '\n');
}

/* -------------------------------------------------------------------------- */

static bool gps_line_looks_like_nmea(const char *line)
{
    if ((line == NULL) || (strlen(line) < 7U))
    {
        return false;
    }

    return (line[0] == '$') &&
           isalpha((unsigned char)line[1]) &&
           isalpha((unsigned char)line[2]) &&
           isalpha((unsigned char)line[3]) &&
           isalpha((unsigned char)line[4]) &&
           isalpha((unsigned char)line[5]) &&
           (line[6] == ',');
}

/* -------------------------------------------------------------------------- */

static bool gps_nmea_checksum_valid(const char *line)
{
    if ((line == NULL) || (line[0] != '$'))
    {
        return false;
    }

    const char *asterisk = strchr(line, '*');

    if ((asterisk == NULL) ||
        (asterisk[1] == '\0') ||
        (asterisk[2] == '\0'))
    {
        return false;
    }

    uint8_t checksum = 0U;

    for (const char *p = line + 1; p < asterisk; ++p)
    {
        checksum ^= (uint8_t)*p;
    }

    char received_text[3] = {
        asterisk[1],
        asterisk[2],
        '\0'};

    char *end = NULL;
    const unsigned long received =
        strtoul(received_text, &end, 16);

    return (end != received_text) &&
           (checksum == (uint8_t)received);
}

/* -------------------------------------------------------------------------- */
/* UBX                                                                        */
/* -------------------------------------------------------------------------- */

static void gps_ubx_checksum(
    const uint8_t *data,
    size_t length,
    uint8_t *ck_a,
    uint8_t *ck_b)
{
    uint8_t a = 0U;
    uint8_t b = 0U;

    for (size_t i = 0U; i < length; ++i)
    {
        a = (uint8_t)(a + data[i]);
        b = (uint8_t)(b + a);
    }

    *ck_a = a;
    *ck_b = b;
}

/* -------------------------------------------------------------------------- */

static esp_err_t gps_ubx_send(
    uint8_t message_class,
    uint8_t message_id,
    const uint8_t *payload,
    uint16_t payload_length)
{
    uint8_t frame[64];

    if (((payload_length > 0U) && (payload == NULL)) ||
        ((size_t)payload_length + 8U > sizeof(frame)))
    {
        return ESP_ERR_INVALID_ARG;
    }

    frame[0] = UBX_SYNC_1;
    frame[1] = UBX_SYNC_2;
    frame[2] = message_class;
    frame[3] = message_id;
    frame[4] = (uint8_t)(payload_length & 0xFFU);
    frame[5] = (uint8_t)(payload_length >> 8);

    if (payload_length > 0U)
    {
        memcpy(&frame[6], payload, payload_length);
    }

    uint8_t ck_a = 0U;
    uint8_t ck_b = 0U;

    gps_ubx_checksum(
        &frame[2],
        (size_t)payload_length + 4U,
        &ck_a,
        &ck_b);

    frame[6U + payload_length] = ck_a;
    frame[7U + payload_length] = ck_b;

    const size_t frame_length =
        (size_t)payload_length + 8U;

    const int written = uart_write_bytes(
        GPS_UART_PORT,
        frame,
        frame_length);

    if (written != (int)frame_length)
    {
        return ESP_FAIL;
    }

    return uart_wait_tx_done(
        GPS_UART_PORT,
        pdMS_TO_TICKS(GPS_UART_TX_TIMEOUT_MS));
}

/* -------------------------------------------------------------------------- */

static bool gps_ubx_wait_ack(
    uint8_t expected_class,
    uint8_t expected_id,
    uint32_t timeout_ms)
{
    enum
    {
        WAIT_SYNC_1,
        WAIT_SYNC_2,
        WAIT_CLASS,
        WAIT_ID,
        WAIT_LEN_L,
        WAIT_LEN_H,
        WAIT_PAYLOAD,
        WAIT_CK_A,
        WAIT_CK_B
    } state = WAIT_SYNC_1;

    uint8_t message_class = 0U;
    uint8_t message_id = 0U;
    uint16_t payload_length = 0U;
    uint16_t payload_position = 0U;
    uint8_t payload[16] = {0};

    uint8_t ck_a = 0U;
    uint8_t ck_b = 0U;
    uint8_t received_ck_a = 0U;

    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start) < timeout)
    {
        uint8_t byte = 0U;

        if (uart_read_bytes(
                GPS_UART_PORT,
                &byte,
                1,
                pdMS_TO_TICKS(GPS_UART_READ_TIMEOUT_MS)) != 1)
        {
            continue;
        }

        switch (state)
        {
        case WAIT_SYNC_1:
            if (byte == UBX_SYNC_1)
            {
                state = WAIT_SYNC_2;
            }
            break;

        case WAIT_SYNC_2:
            state = (byte == UBX_SYNC_2) ? WAIT_CLASS : WAIT_SYNC_1;
            break;

        case WAIT_CLASS:
            message_class = byte;
            ck_a = byte;
            ck_b = ck_a;
            state = WAIT_ID;
            break;

        case WAIT_ID:
            message_id = byte;
            ck_a = (uint8_t)(ck_a + byte);
            ck_b = (uint8_t)(ck_b + ck_a);
            state = WAIT_LEN_L;
            break;

        case WAIT_LEN_L:
            payload_length = byte;
            ck_a = (uint8_t)(ck_a + byte);
            ck_b = (uint8_t)(ck_b + ck_a);
            state = WAIT_LEN_H;
            break;

        case WAIT_LEN_H:
            payload_length |= (uint16_t)byte << 8;
            ck_a = (uint8_t)(ck_a + byte);
            ck_b = (uint8_t)(ck_b + ck_a);
            payload_position = 0U;

            if (payload_length > sizeof(payload))
            {
                state = WAIT_SYNC_1;
            }
            else
            {
                state = (payload_length == 0U) ? WAIT_CK_A : WAIT_PAYLOAD;
            }
            break;

        case WAIT_PAYLOAD:
            payload[payload_position++] = byte;
            ck_a = (uint8_t)(ck_a + byte);
            ck_b = (uint8_t)(ck_b + ck_a);

            if (payload_position >= payload_length)
            {
                state = WAIT_CK_A;
            }
            break;

        case WAIT_CK_A:
            received_ck_a = byte;
            state = WAIT_CK_B;
            break;

        case WAIT_CK_B:
        {
            const bool checksum_ok =
                (received_ck_a == ck_a) &&
                (byte == ck_b);

            if (checksum_ok &&
                (message_class == UBX_CLASS_ACK) &&
                (payload_length >= 2U) &&
                (payload[0] == expected_class) &&
                (payload[1] == expected_id))
            {
                return message_id == UBX_ID_ACK_ACK;
            }

            state = WAIT_SYNC_1;
            break;
        }

        default:
            state = WAIT_SYNC_1;
            break;
        }
    }

    return false;
}

/* -------------------------------------------------------------------------- */

static bool gps_ubx_send_and_ack(
    uint8_t message_id,
    const uint8_t *payload,
    uint16_t payload_length)
{
    uart_flush_input(GPS_UART_PORT);

    if (gps_ubx_send(
            UBX_CLASS_CFG,
            message_id,
            payload,
            payload_length) != ESP_OK)
    {
        return false;
    }

    return gps_ubx_wait_ack(
        UBX_CLASS_CFG,
        message_id,
        GPS_ACK_TIMEOUT_MS);
}

/* -------------------------------------------------------------------------- */

static bool gps_ubx_set_rate(void)
{
    const uint8_t payload[6] = {
        (uint8_t)(GPS_TARGET_RATE_MS & 0xFFU),
        (uint8_t)(GPS_TARGET_RATE_MS >> 8),
        0x01U,
        0x00U,
        0x00U,
        0x00U, /* UTC */
    };

    return gps_ubx_send_and_ack(
        UBX_ID_CFG_RATE,
        payload,
        sizeof(payload));
}

/* -------------------------------------------------------------------------- */

static bool gps_ubx_set_message_rate(
    uint8_t nmea_id,
    uint8_t uart1_rate)
{
    const uint8_t payload[8] = {
        NMEA_CLASS,
        nmea_id,
        0U,
        uart1_rate,
        0U,
        0U,
        0U,
        0U,
    };

    return gps_ubx_send_and_ack(
        UBX_ID_CFG_MSG,
        payload,
        sizeof(payload));
}

/* -------------------------------------------------------------------------- */

static bool gps_ubx_configure_gga_rmc_only(void)
{
    typedef struct
    {
        uint8_t id;
        uint8_t rate;
        const char *name;
    } message_cfg_t;

    static const message_cfg_t messages[] = {
        {NMEA_ID_GGA, 1U, "GGA"},
        {NMEA_ID_RMC, 1U, "RMC"},
        {NMEA_ID_GLL, 0U, "GLL"},
        {NMEA_ID_GSA, 0U, "GSA"},
        {NMEA_ID_GSV, 0U, "GSV"},
        {NMEA_ID_VTG, 0U, "VTG"},
        {NMEA_ID_GRS, 0U, "GRS"},
        {NMEA_ID_GST, 0U, "GST"},
        {NMEA_ID_ZDA, 0U, "ZDA"},
        {NMEA_ID_GBS, 0U, "GBS"},
        {NMEA_ID_DTM, 0U, "DTM"},
    };

    for (size_t i = 0U;
         i < sizeof(messages) / sizeof(messages[0]);
         ++i)
    {
        if (!gps_ubx_set_message_rate(
                messages[i].id,
                messages[i].rate))
        {
            ESP_LOGW(
                TAG,
                "CFG-MSG no confirmado para %s",
                messages[i].name);

            return false;
        }

        ESP_LOGI(
            TAG,
            "NMEA %s -> rate=%u",
            messages[i].name,
            messages[i].rate);
    }

    return true;
}

/* -------------------------------------------------------------------------- */

static esp_err_t gps_ubx_change_receiver_baud(
    uint32_t baud_rate)
{
    uint8_t payload[20] = {0};

    payload[0] = 0x01U; /* UART1 */

    /* 8N1: 0x000008D0 */
    payload[4] = 0xD0U;
    payload[5] = 0x08U;

    payload[8] = (uint8_t)(baud_rate & 0xFFU);
    payload[9] = (uint8_t)((baud_rate >> 8) & 0xFFU);
    payload[10] = (uint8_t)((baud_rate >> 16) & 0xFFU);
    payload[11] = (uint8_t)((baud_rate >> 24) & 0xFFU);

    /* Entrada UBX+NMEA+RTCM; salida UBX+NMEA. */
    payload[12] = 0x07U;
    payload[14] = 0x03U;

    uart_flush_input(GPS_UART_PORT);

    esp_err_t err = gps_ubx_send(
        UBX_CLASS_CFG,
        UBX_ID_CFG_PRT,
        payload,
        sizeof(payload));

    if (err != ESP_OK)
    {
        return err;
    }

    /*
     * CFG-PRT puede cambiar la velocidad antes de que resulte posible leer
     * el ACK. Por eso se conmuta inmediatamente la UART del ESP32-S3.
     */
    vTaskDelay(pdMS_TO_TICKS(GPS_BAUD_CHANGE_PRE_DELAY_MS));

    err = gps_uart_set_baud(baud_rate);

    if (err != ESP_OK)
    {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(GPS_BAUD_CHANGE_POST_DELAY_MS));
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

static bool gps_ubx_save_configuration(void)
{
    /*
     * UBX-CFG-CFG:
     *   clearMask = 0
     *   saveMask  = 0x0000FFFF
     *   loadMask  = 0
     *   deviceMask= BBR + Flash + EEPROM + SPI Flash (0x17)
     *
     * Algunos módulos solo disponen de BBR o no admiten persistencia.
     */
    const uint8_t payload[13] = {
        0x00U,
        0x00U,
        0x00U,
        0x00U,
        0xFFU,
        0xFFU,
        0x00U,
        0x00U,
        0x00U,
        0x00U,
        0x00U,
        0x00U,
        0x17U,
    };

    return gps_ubx_send_and_ack(
        UBX_ID_CFG_CFG,
        payload,
        sizeof(payload));
}

/* -------------------------------------------------------------------------- */
/* Detección y verificación                                                   */
/* -------------------------------------------------------------------------- */

static bool gps_wait_for_nmea(
    uint32_t timeout_ms,
    char *first_line,
    size_t first_line_size)
{
    uint8_t data[GPS_READ_BUFFER_SIZE];
    char line[GPS_LINE_BUFFER_SIZE];
    size_t position = 0U;

    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start) < timeout)
    {
        const int received = uart_read_bytes(
            GPS_UART_PORT,
            data,
            sizeof(data),
            pdMS_TO_TICKS(GPS_NMEA_READ_TIMEOUT_MS));

        if (received <= 0)
        {
            continue;
        }

        for (int i = 0; i < received; ++i)
        {
            const uint8_t c = data[i];

            if ((c == '\r') || (c == '\n'))
            {
                if (position > 0U)
                {
                    line[position] = '\0';

                    if (gps_line_looks_like_nmea(line) &&
                        gps_nmea_checksum_valid(line))
                    {
                        if ((first_line != NULL) &&
                            (first_line_size > 0U))
                        {
                            snprintf(
                                first_line,
                                first_line_size,
                                "%s",
                                line);
                        }

                        uart_flush_input(GPS_UART_PORT);
                        return true;
                    }

                    position = 0U;
                }

                continue;
            }

            if (c == '$')
            {
                position = 0U;
            }

            if (gps_is_printable_nmea(c) &&
                position < sizeof(line) - 1U)
            {
                line[position++] = (char)c;
            }
        }
    }

    return false;
}

/* -------------------------------------------------------------------------- */

static void gps_configure_receiver(void)
{
    ESP_LOGI(TAG, "Configurando navegación a 5 Hz");

    if (!gps_ubx_set_rate())
    {
        ESP_LOGW(TAG, "El receptor no confirmó UBX-CFG-RATE."
                      "Se conserva la configuración original.");
        return;
    }

    ESP_LOGI(TAG, "Configurando salida NMEA GGA + RMC");

    if (!gps_ubx_configure_gga_rmc_only())
    {
        ESP_LOGW(TAG, "No se pudo completar la selección de mensajes NMEA.");
    }

#if !GPS_115200
    ESP_LOGI(TAG, "Cambiando receptor a 115200 baud");

    if (gps_ubx_change_receiver_baud(GPS_TARGET_BAUD_RATE) != ESP_OK)
    {
        ESP_LOGW(TAG, "Falló el cambio de velocidad. "
                      "Se intentará continuar a 9600 baud.");

        gps_uart_set_baud(GPS_INITIAL_BAUD_RATE);
        return;
    }
    char line[GPS_LINE_BUFFER_SIZE];

    if (!gps_wait_for_nmea(
            GPS_VERIFY_TIMEOUT_MS,
            line,
            sizeof(line)))
    {
        ESP_LOGW(
            TAG,
            "No se verificó NMEA a 115200 baud. "
            "Se intentará volver a 9600 baud.");

        gps_uart_set_baud(GPS_INITIAL_BAUD_RATE);
        return;
    }
#endif

    if (gps_ubx_save_configuration())
        ESP_LOGI(TAG, "Configuración guardada por el receptor");
    else
    {
        ESP_LOGW(
            TAG,
            "El receptor no confirmó el guardado. "
            "La configuración se reaplicará en cada arranque.");
    }
}

/* -------------------------------------------------------------------------- */
/* Parser NMEA                                                                */
/* -------------------------------------------------------------------------- */

static size_t gps_split_fields(
    char *line,
    char **fields,
    size_t maximum_fields)
{
    size_t count = 0U;

    if ((line == NULL) ||
        (fields == NULL) ||
        (maximum_fields == 0U))
    {
        return 0U;
    }

    fields[count++] = line;

    for (char *p = line;
         (*p != '\0') && (count < maximum_fields);
         ++p)
    {
        if ((*p == ',') || (*p == '*'))
        {
            *p = '\0';
            fields[count++] = p + 1;
        }
    }

    return count;
}

/* -------------------------------------------------------------------------- */

static bool gps_parse_utc_time(
    const char *text,
    uint8_t *hour,
    uint8_t *minute,
    uint8_t *second,
    uint16_t *millisecond)
{
    if ((text == NULL) || (strlen(text) < 6U))
    {
        return false;
    }

    char hh[3] = {text[0], text[1], '\0'};
    char mm[3] = {text[2], text[3], '\0'};
    char ss[3] = {text[4], text[5], '\0'};

    *hour = (uint8_t)strtoul(hh, NULL, 10);
    *minute = (uint8_t)strtoul(mm, NULL, 10);
    *second = (uint8_t)strtoul(ss, NULL, 10);
    *millisecond = 0U;

    const char *dot = strchr(text, '.');

    if (dot != NULL)
    {
        uint16_t factor = 100U;

        for (const char *p = dot + 1;
             (*p != '\0') && (factor > 0U);
             ++p)
        {
            if (!isdigit((unsigned char)*p))
            {
                break;
            }

            *millisecond +=
                (uint16_t)(*p - '0') * factor;

            factor /= 10U;
        }
    }

    return true;
}

/* -------------------------------------------------------------------------- */

/*
 * Convierte una fecha/hora UTC de calendario a Unix timestamp sin depender
 * del huso horario ni de timegm()/mktime().
 *
 * El algoritmo calcula primero el número de días transcurridos desde
 * 1970-01-01 y después añade hora, minuto y segundo.
 */
static bool gps_utc_to_timestamp(
    uint16_t year,
    uint8_t month,
    uint8_t day,
    uint8_t hour,
    uint8_t minute,
    uint8_t second,
    uint32_t *timestamp)
{
    if ((timestamp == NULL) ||
        (year < 1970U) ||
        (month < 1U) || (month > 12U) ||
        (day < 1U) || (day > 31U) ||
        (hour > 23U) ||
        (minute > 59U) ||
        (second > 60U))
    {
        return false;
    }

    int32_t y = (int32_t)year;
    const int32_t m = (int32_t)month;
    const int32_t d = (int32_t)day;

    y -= (m <= 2);

    const int32_t era = (y >= 0 ? y : y - 399) / 400;
    const uint32_t yoe = (uint32_t)(y - era * 400);
    const uint32_t mp = (uint32_t)(m + (m > 2 ? -3 : 9));
    const uint32_t doy =
        (153U * mp + 2U) / 5U + (uint32_t)d - 1U;
    const uint32_t doe =
        yoe * 365U + yoe / 4U - yoe / 100U + doy;

    const int64_t days =
        (int64_t)era * 146097LL + (int64_t)doe - 719468LL;

    if (days < 0)
    {
        return false;
    }

    const uint64_t seconds =
        (uint64_t)days * 86400ULL +
        (uint64_t)hour * 3600ULL +
        (uint64_t)minute * 60ULL +
        (uint64_t)second;

    if (seconds > UINT32_MAX)
    {
        return false;
    }

    *timestamp = (uint32_t)seconds;
    return true;
}

/* -------------------------------------------------------------------------- */

static bool gps_parse_coordinate(
    const char *value,
    const char *hemisphere,
    double *coordinate_deg)
{
    if ((value == NULL) ||
        (hemisphere == NULL) ||
        (coordinate_deg == NULL) ||
        (value[0] == '\0') ||
        (hemisphere[0] == '\0'))
    {
        return false;
    }

    const double raw = strtod(value, NULL);
    const double degrees = floor(raw / 100.0);
    const double minutes = raw - degrees * 100.0;

    double result = degrees + minutes / 60.0;

    if ((hemisphere[0] == 'S') ||
        (hemisphere[0] == 'W'))
    {
        result = -result;
    }

    *coordinate_deg = result;
    return true;
}

/* -------------------------------------------------------------------------- */

static bool gps_parse_gga(char *line)
{
    char *fields[20];
    const size_t count =
        gps_split_fields(line, fields, 20U);

    if (count < 15U)
    {
        return false;
    }

    gps_data_t update = GPS_get_data();

    uint8_t utc_hour = 0U;
    uint8_t utc_minute = 0U;
    uint8_t utc_second = 0U;
    uint16_t utc_millisecond = 0U;

    (void)gps_parse_utc_time(
        fields[1],
        &utc_hour,
        &utc_minute,
        &utc_second,
        &utc_millisecond);

    gps_parse_coordinate(
        fields[2],
        fields[3],
        &update.latitude_deg);

    gps_parse_coordinate(
        fields[4],
        fields[5],
        &update.longitude_deg);

    update.fix_quality =
        (uint8_t)strtoul(fields[6], NULL, 10);

    update.satellites =
        (uint8_t)strtoul(fields[7], NULL, 10);

    update.hdop =
        (float)strtod(fields[8], NULL);

    update.altitude_m =
        (float)strtod(fields[9], NULL);

    update.fix_valid =
        update.fix_quality > 0U;

    /*
     * valid indica que se ha recibido y parseado una sentencia GNSS válida.
     * fix_valid queda reservado exclusivamente a la validez de la solución
     * de navegación. Así fecha/hora pueden publicarse aunque todavía no haya FIX.
     */
    update.valid = true;
    update.connected = true;
    ++update.received_sentences;

    portENTER_CRITICAL(&s_data_mux);
    s_data = update;
    portEXIT_CRITICAL(&s_data_mux);

    /*
     * Diagnóstico temporal:
     * se muestra la hora UTC cada vez que se recibe una sentencia GGA válida,
     * independientemente de que exista FIX de navegación.
     */
    /*
    ESP_LOGI(
        TAG,
        "GGA UTC %02u:%02u:%02u.%03u | FIX=%s",
        utc_hour,
        utc_minute,
        utc_second,
        utc_millisecond,
        update.fix_valid ? "SI" : "NO");

    */
    return true;
}

/* -------------------------------------------------------------------------- */
// #define MUESTRA_GPS_RATE 1

static bool gps_parse_rmc(char *line)
{

    //------- MEDIDA DE TIEMPO ENTRE DATO RECIBIDO ---------------------------
#ifdef MUESTRA_GPS_RATE
    static int64_t last_rmc_us = 0;

    const int64_t now_us = esp_timer_get_time();

    if (last_rmc_us != 0)
    {
        const float dt_ms = (float)(now_us - last_rmc_us) / 1000.0f;

        const float freq_hz = 1000.0f / dt_ms;

        ESP_LOGI(TAG, "RMC dt=%.1f ms -> %.2f Hz", (double)dt_ms, (double)freq_hz);
    }

    last_rmc_us = now_us;
#endif
    //----------------------------------

    char *fields[20];
    const size_t count =
        gps_split_fields(line, fields, 20U);

    if (count < 13U)
    {
        return false;
    }

    gps_data_t update = GPS_get_data();

    uint8_t utc_hour = 0U;
    uint8_t utc_minute = 0U;
    uint8_t utc_second = 0U;
    uint16_t utc_millisecond = 0U;

    (void)gps_parse_utc_time(
        fields[1],
        &utc_hour,
        &utc_minute,
        &utc_second,
        &utc_millisecond);

    const bool rmc_valid =
        fields[2][0] == 'A';

    gps_parse_coordinate(
        fields[3],
        fields[4],
        &update.latitude_deg);

    gps_parse_coordinate(
        fields[5],
        fields[6],
        &update.longitude_deg);

    update.ground_speed_knots =
        (float)strtod(fields[7], NULL);

    update.ground_speed_kmh =
        update.ground_speed_knots * 1.852f;

    update.ground_speed_mps =
        update.ground_speed_knots * 0.514444f;

    update.ground_track_deg =
        (float)strtod(fields[8], NULL);

    if (strlen(fields[9]) >= 6U)
    {
        char dd[3] = {fields[9][0], fields[9][1], '\0'};
        char mm[3] = {fields[9][2], fields[9][3], '\0'};
        char yy[3] = {fields[9][4], fields[9][5], '\0'};

        const uint8_t utc_day =
            (uint8_t)strtoul(dd, NULL, 10);

        const uint8_t utc_month =
            (uint8_t)strtoul(mm, NULL, 10);

        const uint16_t utc_year =
            (uint16_t)(2000U +
                       (uint16_t)strtoul(yy, NULL, 10));

        uint32_t utc_timestamp = 0U;

        if (gps_utc_to_timestamp(
                utc_year,
                utc_month,
                utc_day,
                utc_hour,
                utc_minute,
                utc_second,
                &utc_timestamp))
        {
            update.utc_timestamp = utc_timestamp;
        }
    }

    update.fix_valid =
        update.fix_valid || rmc_valid;

    /*
     * valid indica que se ha recibido y parseado una sentencia GNSS válida.
     * fix_valid queda reservado exclusivamente a la validez de la solución
     * de navegación. Así fecha/hora pueden publicarse aunque todavía no haya FIX.
     */
    update.valid = true;
    update.connected = true;
    ++update.received_sentences;

    portENTER_CRITICAL(&s_data_mux);
    s_data = update;
    portEXIT_CRITICAL(&s_data_mux);

    /*
     * Diagnóstico temporal:
     * RMC aporta además la fecha, por lo que se muestra fecha + hora UTC.
     * Se imprime aunque el estado RMC sea V (sin FIX válido).
     */
    /*
    ESP_LOGI(
        TAG,
        "RMC UTC timestamp=%" PRIu32 " | FIX=%s",
        update.utc_timestamp,
        update.fix_valid ? "SI" : "NO");
    */
    return true;
}

/* -------------------------------------------------------------------------- */

static bool gps_process_nmea_line(char *line)
{
    if (!gps_nmea_checksum_valid(line))
    {
        portENTER_CRITICAL(&s_data_mux);
        ++s_data.checksum_errors;
        portEXIT_CRITICAL(&s_data_mux);
        return false;
    }

    if ((strncmp(line + 3, "GGA", 3U) == 0))
    {
        return gps_parse_gga(line);
    }

    if ((strncmp(line + 3, "RMC", 3U) == 0))
    {
        return gps_parse_rmc(line);
    }

    /*
     * Si el receptor no aceptó CFG-MSG pueden seguir llegando otras
     * sentencias. Se consideran comunicación válida, pero no se procesan.
     */
    portENTER_CRITICAL(&s_data_mux);
    s_data.connected = true;
    ++s_data.received_sentences;
    portEXIT_CRITICAL(&s_data_mux);

    return true;
}

/* -------------------------------------------------------------------------- */
/* Tarea                                                                      */
/* -------------------------------------------------------------------------- */

static void gps_task(void *argument)
{
    (void)argument;

    uint8_t data[GPS_READ_BUFFER_SIZE];
    char line[GPS_LINE_BUFFER_SIZE];
    size_t position = 0U;

    ESP_LOGI(TAG, "Tarea GPS iniciada");

    for (;;)
    {
        const int received = uart_read_bytes(
            GPS_UART_PORT,
            data,
            sizeof(data),
            pdMS_TO_TICKS(GPS_NMEA_READ_TIMEOUT_MS));

        if (received <= 0)
        {
            continue;
        }

        /*
         * Opcional: diagnóstico del número de bytes recibidos en cada lectura
         * de la UART.
         */
        ESP_LOGD(TAG, "UART RX: %d bytes", received);

        for (int i = 0; i < received; ++i)
        {
            const uint8_t c = data[i];

            /*
             * Final de sentencia NMEA.
             */
            if ((c == '\r') || (c == '\n'))
            {
                if (position > 0U)
                {
                    line[position] = '\0';

                    if (gps_line_looks_like_nmea(line))
                    {
                        /*
                         * Mostrar exactamente la sentencia recibida antes
                         * de procesarla.
                         */
                        // ESP_LOGI(TAG, "NMEA RX: %s", line);

                        if (!gps_process_nmea_line(line))
                        {
                            ESP_LOGW(TAG, "Error procesando NMEA: %s", line);

                            portENTER_CRITICAL(&s_data_mux);
                            ++s_data.parse_errors;
                            portEXIT_CRITICAL(&s_data_mux);
                        }
                    }
                    else
                    {
                        /*
                         * Opcional: muestra líneas recibidas que no cumplen
                         * el formato NMEA esperado.
                         */
                        ESP_LOGW(TAG, "Línea no NMEA: %s", line);
                    }

                    position = 0U;
                }

                continue;
            }

            /*
             * El carácter '$' marca el comienzo de una nueva sentencia.
             * Si había datos incompletos en el buffer se descartan.
             */
            if (c == '$')
            {
                position = 0U;
            }

            if (gps_is_printable_nmea(c) &&
                position < sizeof(line) - 1U)
            {
                line[position++] = (char)c;
            }
            else if (position >= sizeof(line) - 1U)
            {
                ESP_LOGW(
                    TAG,
                    "Overflow de línea NMEA; se descarta la sentencia");

                position = 0U;

                portENTER_CRITICAL(&s_data_mux);
                ++s_data.parse_errors;
                portEXIT_CRITICAL(&s_data_mux);
            }
        }
    }
}

/* -------------------------------------------------------------------------- */
/* API pública                                                                */
/* -------------------------------------------------------------------------- */

esp_err_t GPS_start(void)
{
    ESP_LOGI(TAG, "Inicializando receptor GPS");

    esp_err_t err = gps_uart_init(GPS_INITIAL_BAUD_RATE);

    if (err != ESP_OK)
        return err;

    char first_line[GPS_LINE_BUFFER_SIZE];

    if (!gps_wait_for_nmea(GPS_DETECT_TIMEOUT_MS, first_line, sizeof(first_line)))
    {
        ESP_LOGE(TAG, "No se detectó NMEA a %" PRIu32 " baud", GPS_INITIAL_BAUD_RATE);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "GPS detectado a %" PRIu32 " baud: %s", GPS_INITIAL_BAUD_RATE, first_line);

    gps_publish_diagnostic(GPS_INITIAL_BAUD_RATE, true);

    /*
     * La configuración UBX es opcional. Si falla, el receptor continúa
     * operativo con sus parámetros de fábrica.
     */
    gps_configure_receiver();

    BaseType_t result = xTaskCreate(gps_task, "gps_task", GPS_TASK_STACK_SIZE, NULL, GPS_TASK_PRIORITY, NULL);

    if (result != pdPASS)
        return ESP_ERR_NO_MEM;

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

gps_data_t GPS_get_data(void)
{
    gps_data_t snapshot;

    portENTER_CRITICAL(&s_data_mux);
    snapshot = s_data;
    portEXIT_CRITICAL(&s_data_mux);

    return snapshot;
}

/* -------------------------------------------------------------------------- */

bool GPS_is_connected(void)
{
    return GPS_get_data().connected;
}
