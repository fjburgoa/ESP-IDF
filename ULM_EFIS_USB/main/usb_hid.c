#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"

#include "BNO055.h"
#include "BNO086.h"
#include "MPU6050.h"
#include "config.h"
#include "settings.h"
#include "usb_hid.h"

#define REPORT_ID_SENSOR_1 1U
#define REPORT_ID_BNO055 2U
#define REPORT_ID_SETTINGS 3U
#define REPORT_ID_ORIENTATION 4U
#define MPU_TELEMETRY_PAYLOAD_SIZE 48U
#define BNO_TELEMETRY_PAYLOAD_SIZE 60U

#if defined(PlacaLarga)
#define SENSOR_1_TELEMETRY_PAYLOAD_SIZE BNO_TELEMETRY_PAYLOAD_SIZE
#elif defined(PlacaCorta)
#define SENSOR_1_TELEMETRY_PAYLOAD_SIZE MPU_TELEMETRY_PAYLOAD_SIZE
#endif
#define BNO_VECTOR_LSB_PER_MS2 100.0f
#define BNO_VECTOR_INVALID INT16_MIN
#define STATUS_MOUNT_MASK 0xFFU
#define STATUS_BNO_MODE_SHIFT 8U
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)

typedef struct __attribute__((packed))
{
    uint32_t sequence;
    uint32_t timestamp_ms;
    float pitch_deg;
    float roll_deg;
    float accel_x_ms2;
    float accel_y_ms2;
    float accel_z_ms2;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float pitch_offset_deg;
    uint32_t status;
} efis_hid_report_t;

typedef struct __attribute__((packed))
{
    efis_hid_report_t telemetry;
    int16_t linear_accel_x;
    int16_t linear_accel_y;
    int16_t linear_accel_z;
    int16_t gravity_x;
    int16_t gravity_y;
    int16_t gravity_z;
} efis_bno_hid_report_t;

_Static_assert(sizeof(efis_hid_report_t) == MPU_TELEMETRY_PAYLOAD_SIZE,
               "Tamano inesperado del informe MPU HID");
_Static_assert(sizeof(efis_bno_hid_report_t) == BNO_TELEMETRY_PAYLOAD_SIZE,
               "Tamano inesperado del informe BNO HID");

static const char *TAG = "USB_HID";
static TaskHandle_t s_usb_task;

static uint32_t pack_status(bno055_operation_mode_t bno_mode)
{
    return ((uint32_t)s_mount_mode & STATUS_MOUNT_MASK) |
           ((uint32_t)bno_mode << STATUS_BNO_MODE_SHIFT);
}

static int16_t encode_ms2(float value)
{
    if (value > 327.67f)
        value = 327.67f;
    else if (value < -327.67f)
        value = -327.67f;
    return (int16_t)(value * BNO_VECTOR_LSB_PER_MS2);
}

/* Report 1/2: telemetría; Report 3: offset; Report 4: orientación. */
static const uint8_t s_hid_report_descriptor[] = {
    0x06, 0x00, 0xFF, /* Usage Page (Vendor 0xFF00) */
    0x09, 0x01,       /* Usage 1 */
    0xA1, 0x01,       /* Collection (Application) */
    0x85, REPORT_ID_SENSOR_1,
    0x09, 0x01, 0x15, 0x00, 0x26, 0xFF, 0x00,
    0x75, 0x08, 0x95, SENSOR_1_TELEMETRY_PAYLOAD_SIZE, 0x81, 0x02,
    0x85, REPORT_ID_BNO055,
    0x09, 0x02, 0x15, 0x00, 0x26, 0xFF, 0x00,
    0x75, 0x08, 0x95, BNO_TELEMETRY_PAYLOAD_SIZE, 0x81, 0x02,
    0x85, REPORT_ID_SETTINGS,
    0x09, 0x03, 0x15, 0x00, 0x26, 0xFF, 0x00,
    0x75, 0x08, 0x95, 0x04, 0x91, 0x02,
    0x85, REPORT_ID_ORIENTATION,
    0x09, 0x04, 0x15, 0x00, 0x26, 0x01, 0x00,
    0x75, 0x08, 0x95, 0x01, 0x91, 0x02,
    0xC0};

static const tusb_desc_device_t s_device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A,
    .idProduct = 0x4004,
    .bcdDevice = 0x0101,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01,
};

static const char *s_string_descriptor[] = {
    (const char[]){0x09, 0x04},
    "EFIS ESP32-S3",
    "EFIS USB IMU",
    "EFIS001",
    "EFIS telemetry"};

static const uint8_t s_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(s_hid_report_descriptor),
                       0x81, 64, 10),
};

/*
 * esp_tinyusb publica los callbacks generales desde descriptors_control.c.
 * La aplicación registra los descriptores mediante tinyusb_config_t y solo
 * implementa el callback específico del informe HID.
 */

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return s_hid_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance;
    (void)report_type;
    if ((report_id == REPORT_ID_SETTINGS) && (bufsize >= sizeof(int32_t)))
    {
        int32_t offset;
        memcpy(&offset, buffer, sizeof(offset));
        esp_err_t err = settings_set_pitch_offset_deg(offset);
        if (err != ESP_OK)
            ESP_LOGW(TAG, "No se guardo el offset: %s", esp_err_to_name(err));
    }
    else if ((report_id == REPORT_ID_ORIENTATION) && (bufsize >= 1U) && (buffer[0] <= (uint8_t)MOUNT_HORIZONTAL))
    {
        /* El host envía directamente el modo solicitado: 0=V, 1=H. */
        s_mount_mode = (mount_mode_t)buffer[0];
        ESP_LOGI(TAG, "Orientacion seleccionada: %s", s_mount_mode == MOUNT_VERTICAL ? "V" : "H");
    }
}

static void fill_mpu_report(efis_hid_report_t *r, uint32_t sequence)
{
    const mpu6050_data_t d = MPU6050_get_data();
    const bno055_data_t bno = BNO055_get_data();
    *r = (efis_hid_report_t){
        .sequence = sequence,
        .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000),
        .pitch_deg = d.pitch_deg,
        .roll_deg = d.roll_deg,
        .accel_x_ms2 = d.accel_x_ms2,
        .accel_y_ms2 = d.accel_y_ms2,
        .accel_z_ms2 = d.accel_z_ms2,
        .gyro_x_dps = d.gyro_x_dps,
        .gyro_y_dps = d.gyro_y_dps,
        .gyro_z_dps = d.gyro_z_dps,
        .pitch_offset_deg = (float)settings_get_pitch_offset_deg(),
        .status = pack_status(bno.operation_mode),
    };
}

static void fill_bno086_report(
    efis_bno_hid_report_t *r,
    uint32_t sequence)
{
    const bno086_data_t d = BNO086_get_data();
    const bno055_data_t bno = BNO055_get_data();

    *r = (efis_bno_hid_report_t){
        .telemetry = {
            .sequence = sequence,
            .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000),
            .pitch_deg = d.pitch_deg,
            .roll_deg = d.roll_deg,
            .accel_x_ms2 = d.acceleration_ms2.x,
            .accel_y_ms2 = d.acceleration_ms2.y,
            .accel_z_ms2 = d.acceleration_ms2.z,
            .gyro_x_dps = d.gyro_dps.x,
            .gyro_y_dps = d.gyro_dps.y,
            .gyro_z_dps = d.gyro_dps.z,
            .pitch_offset_deg = (float)settings_get_pitch_offset_deg(),

            /*
             * Se conserva el formato de status existente.
             * El campo de modo sigue reflejando el modo actual del BNO055.
             * El BNO086 no usa AMG/IMUPLUS/NDOF.
             */
            .status = pack_status(bno.operation_mode),
        },

        .linear_accel_x =
            d.linear_accel_valid
                ? encode_ms2(d.linear_accel_ms2.x)
                : BNO_VECTOR_INVALID,

        .linear_accel_y =
            d.linear_accel_valid
                ? encode_ms2(d.linear_accel_ms2.y)
                : BNO_VECTOR_INVALID,

        .linear_accel_z =
            d.linear_accel_valid
                ? encode_ms2(d.linear_accel_ms2.z)
                : BNO_VECTOR_INVALID,

        .gravity_x =
            d.gravity_valid
                ? encode_ms2(d.gravity_ms2.x)
                : BNO_VECTOR_INVALID,

        .gravity_y =
            d.gravity_valid
                ? encode_ms2(d.gravity_ms2.y)
                : BNO_VECTOR_INVALID,

        .gravity_z =
            d.gravity_valid
                ? encode_ms2(d.gravity_ms2.z)
                : BNO_VECTOR_INVALID,
    };
}

static void fill_bno_report(efis_bno_hid_report_t *r, uint32_t sequence)
{
    const bno055_data_t d = BNO055_get_data();
    *r = (efis_bno_hid_report_t){
        .telemetry = {
            .sequence = sequence,
            .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000),
            .pitch_deg = d.pitch_deg,
            .roll_deg = d.roll_deg,
            .accel_x_ms2 = d.acceleration_ms2.x,
            .accel_y_ms2 = d.acceleration_ms2.y,
            .accel_z_ms2 = d.acceleration_ms2.z,
            .gyro_x_dps = d.gyro_dps.x,
            .gyro_y_dps = d.gyro_dps.y,
            .gyro_z_dps = d.gyro_dps.z,
            .pitch_offset_deg = (float)settings_get_pitch_offset_deg(),
            .status = pack_status(d.operation_mode),
        },
        .linear_accel_x = d.linear_accel_valid ? encode_ms2(d.linear_accel.x) : BNO_VECTOR_INVALID,
        .linear_accel_y = d.linear_accel_valid ? encode_ms2(d.linear_accel.y) : BNO_VECTOR_INVALID,
        .linear_accel_z = d.linear_accel_valid ? encode_ms2(d.linear_accel.z) : BNO_VECTOR_INVALID,
        .gravity_x = d.gravity_valid ? encode_ms2(d.gravity.x) : BNO_VECTOR_INVALID,
        .gravity_y = d.gravity_valid ? encode_ms2(d.gravity.y) : BNO_VECTOR_INVALID,
        .gravity_z = d.gravity_valid ? encode_ms2(d.gravity.z) : BNO_VECTOR_INVALID,
    };
}

static void usb_telemetry_task(void *arg)
{
    (void)arg;

    uint32_t sensor_1_sequence = 0U;
    uint32_t bno055_sequence = 0U;

    bool send_sensor_1 = true;

    TickType_t wake = xTaskGetTickCount();

    while (1)
    {
        if (tud_mounted() && tud_hid_ready())
        {
            if (send_sensor_1)
            {
#if defined(PlacaLarga)

                /*
                 * Placa larga:
                 * ID1 = BNO086
                 */
                efis_bno_hid_report_t report;

                fill_bno086_report(
                    &report,
                    ++sensor_1_sequence);

                tud_hid_report(
                    REPORT_ID_SENSOR_1,
                    &report,
                    sizeof(report));

#elif defined(PlacaCorta)

                /*
                 * Placa corta:
                 * ID1 = MPU6050
                 */
                efis_hid_report_t report;

                fill_mpu_report(
                    &report,
                    ++sensor_1_sequence);

                tud_hid_report(
                    REPORT_ID_SENSOR_1,
                    &report,
                    sizeof(report));

#endif
            }
            else
            {
                /*
                 * En ambas placas:
                 * ID2 = BNO055
                 */
                efis_bno_hid_report_t report;

                fill_bno_report(
                    &report,
                    ++bno055_sequence);

                tud_hid_report(
                    REPORT_ID_BNO055,
                    &report,
                    sizeof(report));
            }

            send_sensor_1 = !send_sensor_1;
        }

        vTaskDelayUntil(
            &wake,
            pdMS_TO_TICKS(USB_TELEMETRY_PERIOD_MS));
    }
}

esp_err_t usb_hid_start(void)
{
    /* esp_tinyusb 2.x exige configurar explícitamente su tarea interna. */
    const tinyusb_config_t config = {
        .task = {
            .size = 4096,
            .priority = 5,
            /* Afinidad explícita: esp_tinyusb no acepta tskNO_AFFINITY. */
            .xCoreID = 0,
        },
        .descriptor = {
            .device = &s_device_descriptor,
            .qualifier = NULL,
            .string = s_string_descriptor,
            .string_count = sizeof(s_string_descriptor) / sizeof(s_string_descriptor[0]),
            .full_speed_config = s_configuration_descriptor,
            .high_speed_config = NULL,
        },
    };
    esp_err_t err = tinyusb_driver_install(&config);
    if (err != ESP_OK)
        return err;

    if (xTaskCreate(usb_telemetry_task, "usb_telemetry", 4096, NULL, 6,
                    &s_usb_task) != pdPASS)
        return ESP_ERR_NO_MEM;
#if defined(PlacaLarga)
    ESP_LOGI(
        TAG,
        "USB HID iniciado: BNO086=ID1, BNO055=ID2, offset=ID3");
#else
    ESP_LOGI(
        TAG,
        "USB HID iniciado: MPU6050=ID1, BNO055=ID2, offset=ID3");
#endif
    return ESP_OK;
}
