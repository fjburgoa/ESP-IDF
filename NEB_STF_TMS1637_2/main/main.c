#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "esp_timer.h"
#include "esp_rom_sys.h"

// ------------------ Pines TM1637 ------------------
#define TM1637_CLK GPIO_NUM_6
#define TM1637_DIO GPIO_NUM_5

// ------------------ Tabla 7 segmentos ------------------
static const uint8_t digits_7seg[10] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};

// -------------------------------------------------------
static inline void tm1637_delay(void)
{
 //   esp_rom_delay_us(3);
}

// -------------------------------------------------------
static void tm1637_start(void)
{
    gpio_set_level(TM1637_DIO, 1);
    gpio_set_level(TM1637_CLK, 1);
    tm1637_delay();
    gpio_set_level(TM1637_DIO, 0);
}

// -------------------------------------------------------
static void tm1637_stop(void)
{
    gpio_set_level(TM1637_CLK, 0);
    tm1637_delay();
    gpio_set_level(TM1637_DIO, 0);
    tm1637_delay();
    gpio_set_level(TM1637_CLK, 1);
    tm1637_delay();
    gpio_set_level(TM1637_DIO, 1);
}

// -------------------------------------------------------
static void tm1637_write_byte(uint8_t data)
{
    for (int i = 0; i < 8; i++) {
        gpio_set_level(TM1637_CLK, 0);
        gpio_set_level(TM1637_DIO, data & 0x01);
        tm1637_delay();
        gpio_set_level(TM1637_CLK, 1);
        tm1637_delay();
        data >>= 1;
    }

    // ACK (se ignora)
    gpio_set_level(TM1637_CLK, 0);
    gpio_set_direction(TM1637_DIO, GPIO_MODE_INPUT);
    tm1637_delay();
    gpio_set_level(TM1637_CLK, 1);
    tm1637_delay();
    gpio_set_level(TM1637_CLK, 0);
    gpio_set_direction(TM1637_DIO, GPIO_MODE_OUTPUT);
}

// -------------------------------------------------------
static void tm1637_set_brightness(uint8_t level)
{
    if (level > 7) level = 7;
    tm1637_start();
    tm1637_write_byte(0x88 | level); // display ON + brillo
    tm1637_stop();
}

// -------------------------------------------------------
static void tm1637_display_digits(const uint8_t seg[4])
{
    tm1637_start();
    tm1637_write_byte(0x40); // auto-increment
    tm1637_stop();

    tm1637_start();
    tm1637_write_byte(0xC0); // dirección inicial
    for (int i = 0; i < 4; i++)
        tm1637_write_byte(seg[i]);
    tm1637_stop();
}

// -------------------------------------------------------
void app_main(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << TM1637_CLK) | (1ULL << TM1637_DIO),
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);

    tm1637_set_brightness(4);

    const char number[] = "1234567890";
    int len = strlen(number);
    int offset = 0;

    uint8_t display[4];

    int x = 9999;

    while (1) 
    {
//        char txt[5] = {0};
        
        display[0] = (x >= 1000) ? digits_7seg[(x / 1000) % 10] : 0x00;
        display[1] = (x >= 100)  ? digits_7seg[(x / 100)  % 10] : 0x00;
        display[2] = (x >= 10)   ? digits_7seg[(x / 10)   % 10] : 0x00;
        display[3] =               digits_7seg[x % 10];


        //sprintf(txt, "%d",x);

        //for (int i = 0; i < 4; i++) 
        //{
        //    char c = number[(offset + i) % len];
        //    display[i] = (c >= '0' && c <= '9') ? digits_7seg[c - '0'] : 0x00;
        //      char c = txt[i];
        //      display[i] = (c >= '0' && c <= '9') ? digits_7seg[c - '0'] : 0x00;

        //}

        x = x-1;
        tm1637_display_digits(display);

        offset = (offset + 1) % len;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
