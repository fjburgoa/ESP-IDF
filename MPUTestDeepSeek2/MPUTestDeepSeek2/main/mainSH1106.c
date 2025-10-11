/*
Este código, imprime "Hola Mundo" por el OLED
*/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "SH1106"

#define I2C_MASTER_SCL_IO           GPIO_NUM_9
#define I2C_MASTER_SDA_IO           GPIO_NUM_8
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000
#define OLED_ADDR                   0x3C

#define SH1106_WIDTH  128
#define SH1106_HEIGHT 64
#define SH1106_PAGES  (SH1106_HEIGHT / 8)
#define SH1106_COL_OFFSET 2

// --- Fuente 5x7 simple ---
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // space
    {0x7E,0x11,0x11,0x11,0x7E}, // A
    {0x7F,0x49,0x49,0x49,0x36}, // B
    {0x3E,0x41,0x41,0x41,0x22}, // C
    {0x7F,0x41,0x41,0x22,0x1C}, // D
    {0x7F,0x49,0x49,0x49,0x41}, // E
    {0x7F,0x09,0x09,0x09,0x01}, // F
    {0x3E,0x41,0x49,0x49,0x7A}, // G
    {0x7F,0x08,0x08,0x08,0x7F}, // H
    {0x00,0x41,0x7F,0x41,0x00}, // I
    {0x20,0x40,0x41,0x3F,0x01}, // J
    {0x7F,0x08,0x14,0x22,0x41}, // K
    {0x7F,0x40,0x40,0x40,0x40}, // L
    {0x7F,0x02,0x04,0x02,0x7F}, // M
    {0x7F,0x04,0x08,0x10,0x7F}, // N
    {0x3E,0x41,0x41,0x41,0x3E}, // O
    {0x7F,0x09,0x09,0x09,0x06}, // P
    {0x3E,0x41,0x51,0x21,0x5E}, // Q
    {0x7F,0x09,0x19,0x29,0x46}, // R
    {0x46,0x49,0x49,0x49,0x31}, // S
    {0x01,0x01,0x7F,0x01,0x01}, // T
    {0x3F,0x40,0x40,0x40,0x3F}, // U
    {0x1F,0x20,0x40,0x20,0x1F}, // V
    {0x7F,0x20,0x18,0x20,0x7F}, // W
    {0x63,0x14,0x08,0x14,0x63}, // X
    {0x03,0x04,0x78,0x04,0x03}, // Y
    {0x61,0x51,0x49,0x45,0x43}, // Z
    {0x3E,0x51,0x49,0x45,0x3E}, // 0
    {0x00,0x42,0x7F,0x40,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46}, // 2
    {0x21,0x41,0x45,0x4B,0x31}, // 3
    {0x18,0x14,0x12,0x7F,0x10}, // 4
    {0x27,0x45,0x45,0x45,0x39}, // 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 6
    {0x01,0x71,0x09,0x05,0x03}, // 7
    {0x36,0x49,0x49,0x49,0x36}, // 8
    {0x06,0x49,0x49,0x29,0x1E}, // 9
};

static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static void send_cmd(uint8_t cmd) {
    uint8_t data[2] = {0x00, cmd};
    i2c_master_write_to_device(I2C_MASTER_NUM, OLED_ADDR, data, 2, pdMS_TO_TICKS(100));
}

static void send_data(const uint8_t *data, size_t len) {
    uint8_t *buf = malloc(len + 1);
    buf[0] = 0x40;
    memcpy(buf + 1, data, len);
    i2c_master_write_to_device(I2C_MASTER_NUM, OLED_ADDR, buf, len + 1, pdMS_TO_TICKS(100));
    free(buf);
}

static void sh1106_init(void) {
    send_cmd(0xAE); // display off
    send_cmd(0xD5); send_cmd(0x80);
    send_cmd(0xA8); send_cmd(0x3F);
    send_cmd(0xD3); send_cmd(0x00);
    send_cmd(0x40);
    send_cmd(0xAD); send_cmd(0x8B);
    send_cmd(0xDA); send_cmd(0x12);
    send_cmd(0x81); send_cmd(0x7F);
    send_cmd(0xA4);
    send_cmd(0xA6);

    // 🔄 ROTACIÓN 180° (pantalla boca abajo)
    send_cmd(0xA0); // SEG normal (en lugar de A1)
    send_cmd(0xC0); // COM normal (en lugar de C8)

    send_cmd(0xAF); // display ON
}

static void sh1106_clear(void) {
    uint8_t zero[128] = {0};
    for (int page = 0; page < SH1106_PAGES; page++) {
        send_cmd(0xB0 + page);
        send_cmd(0x02);
        send_cmd(0x10);
        send_data(zero, SH1106_WIDTH);
    }
}

static void draw_char(uint8_t x, uint8_t page, char c) {
    if (c == ' ') {
        uint8_t space[6] = {0};
        send_cmd(0xB0 + page);
        send_cmd(0x00 + ((x + SH1106_COL_OFFSET) & 0x0F));
        send_cmd(0x10 + ((x + SH1106_COL_OFFSET) >> 4));
        send_data(space, 6);
        return;
    }

    int idx;
    if (c >= 'A' && c <= 'Z') idx = c - 'A' + 1;
    else if (c >= '0' && c <= '9') idx = c - '0' + 27;
    else idx = 0;

    send_cmd(0xB0 + page);
    send_cmd(0x00 + ((x + SH1106_COL_OFFSET) & 0x0F));
    send_cmd(0x10 + ((x + SH1106_COL_OFFSET) >> 4));
    send_data(font5x7[idx], 5);
    uint8_t space = 0x00;
    send_data(&space, 1);
}

static void draw_text(uint8_t x, uint8_t page, const char *text) {
    while (*text) {
        draw_char(x, page, *text++);
        x += 6;
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    sh1106_init();
    sh1106_clear();
    vTaskDelay(pdMS_TO_TICKS(200));
    draw_text(30, 3, "HOLA MUNDO");
    ESP_LOGI(TAG, "Texto mostrado rotado");
}
