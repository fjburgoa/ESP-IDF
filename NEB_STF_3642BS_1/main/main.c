#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#define TM1637_CLK GPIO_NUM_11
#define TM1637_DIO GPIO_NUM_12

#define TM1637_DELAY_US 5

static const uint8_t seg_map[] = {
    0x3f, // 0
    0x06, // 1
    0x5b, // 2
    0x4f, // 3
    0x66, // 4
    0x6d, // 5
    0x7d, // 6
    0x07, // 7
    0x7f, // 8
    0x6f  // 9
};

void esp_rom_delay_us(uint32_t us);

/* ---------- Bajo nivel TM1637 ---------- */

static inline void delay_us(uint32_t us)
{
    esp_rom_delay_us(us);
}

static void dio_output(void)
{
    gpio_set_direction(TM1637_DIO, GPIO_MODE_OUTPUT);
}

static void dio_input(void)
{
    gpio_set_direction(TM1637_DIO, GPIO_MODE_INPUT);
}

static void tm1637_start(void)
{
    dio_output();
    gpio_set_level(TM1637_DIO, 1);
    gpio_set_level(TM1637_CLK, 1);
    delay_us(TM1637_DELAY_US);
    gpio_set_level(TM1637_DIO, 0);
}

static void tm1637_stop(void)
{
    dio_output();
    gpio_set_level(TM1637_CLK, 0);
    delay_us(TM1637_DELAY_US);
    gpio_set_level(TM1637_DIO, 0);
    delay_us(TM1637_DELAY_US);
    gpio_set_level(TM1637_CLK, 1);
    delay_us(TM1637_DELAY_US);
    gpio_set_level(TM1637_DIO, 1);
}

static void tm1637_write_byte(uint8_t data)
{
    dio_output();
    for (int i = 0; i < 8; i++) {
        gpio_set_level(TM1637_CLK, 0);
        gpio_set_level(TM1637_DIO, data & 0x01);
        delay_us(TM1637_DELAY_US);
        gpio_set_level(TM1637_CLK, 1);
        delay_us(TM1637_DELAY_US);
        data >>= 1;
    }

    /* ACK */
    gpio_set_level(TM1637_CLK, 0);
    dio_input();
    delay_us(TM1637_DELAY_US);
    gpio_set_level(TM1637_CLK, 1);
    delay_us(TM1637_DELAY_US);
    dio_output();
}

/* ---------- Alto nivel ---------- */

static void tm1637_set_brightness(uint8_t brightness)
{
    tm1637_start();
    tm1637_write_byte(0x88 | (brightness & 0x07)); // ON + brillo
    tm1637_stop();
}

static void tm1637_display_time(uint8_t hh, uint8_t mm)
{
    uint8_t digits[4];

    digits[0] = seg_map[hh / 10];
    digits[1] = seg_map[hh % 10] | 0x80; // punto central (:) encendido
    digits[2] = seg_map[mm / 10];
    digits[3] = seg_map[mm % 10];

    tm1637_start();
    tm1637_write_byte(0x40); // auto-increment
    tm1637_stop();

    tm1637_start();
    tm1637_write_byte(0xC0); // dirección inicial
    for (int i = 0; i < 4; i++) {
        tm1637_write_byte(digits[i]);
    }
    tm1637_stop();
}

/* ---------- app_main ---------- */

void app_main(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << TM1637_CLK) | (1ULL << TM1637_DIO),
    };
    gpio_config(&io_conf);

    tm1637_set_brightness(7); // brillo medio

    uint8_t hours = 12;
    uint8_t minutes = 0;

    while (1) {
        tm1637_display_time(hours, minutes);

        vTaskDelay(pdMS_TO_TICKS(1000));

        minutes++;
        if (minutes >= 60) {
            minutes = 0;
            hours = (hours + 1) % 24;
        }
    }
}
