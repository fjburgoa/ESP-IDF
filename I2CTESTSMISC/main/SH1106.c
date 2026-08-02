/*

Driver para acceder al SH1106

*/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "SH1106.h"
#include "esp_log.h"

#define OLED_ADDR 0x3C

#define SH1106_WIDTH 128
#define SH1106_HEIGHT 64
#define SH1106_PAGES (SH1106_HEIGHT / 8)
#define SH1106_COL_OFFSET 2 // columnas “fantasma” en SH1106

// --- Fuente 5x7: mayúsculas, minúsculas, números y símbolos básicos ---
static const uint8_t font5x7[][5] = {
    // Caracteres básicos
    {0x00, 0x00, 0x00, 0x00, 0x00}, // [0] space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // [1] !

    // Mayúsculas A-Z: índices 2-27
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // [2]  A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // [3]  B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // [4]  C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // [5]  D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // [6]  E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // [7]  F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // [8]  G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // [9]  H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // [10] I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // [11] J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // [12] K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // [13] L
    {0x7F, 0x02, 0x04, 0x02, 0x7F}, // [14] M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // [15] N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // [16] O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // [17] P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // [18] Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // [19] R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // [20] S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // [21] T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // [22] U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // [23] V
    {0x7F, 0x20, 0x18, 0x20, 0x7F}, // [24] W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // [25] X
    {0x03, 0x04, 0x78, 0x04, 0x03}, // [26] Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // [27] Z

    // Minúsculas a-z: índices 28-53
    {0x20, 0x54, 0x54, 0x54, 0x78}, // [28] a
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // [29] b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // [30] c
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // [31] d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // [32] e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // [33] f
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // [34] g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // [35] h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // [36] i
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // [37] j
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // [38] k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // [39] l
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // [40] m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // [41] n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // [42] o
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // [43] p
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // [44] q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // [45] r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // [46] s
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // [47] t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // [48] u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // [49] v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // [50] w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // [51] x
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // [52] y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // [53] z

    // Números 0-9: índices 54-63
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // [54] 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // [55] 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // [56] 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // [57] 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // [58] 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // [59] 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // [60] 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // [61] 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // [62] 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // [63] 9

    // Símbolos
    {0x00, 0x36, 0x36, 0x00, 0x00}, // [64] :
    {0x00, 0x08, 0x08, 0x08, 0x00}, // [65] -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // [66] .
    {0x06, 0x09, 0x09, 0x06, 0x00}, // [67] ° grado
};

#define SH1106_COLUMN_OFFSET 2 // algunos módulos tienen desplazamiento de 2 columnas

void sh1106_set_cursor(uint8_t x, uint8_t page)
{
    x += SH1106_COLUMN_OFFSET;

    sh1106_send_cmd(0xB0 + page);              // Selecciona la página (0xB0–0xB7)
    sh1106_send_cmd(0x00 + (x & 0x0F));        // Bits bajos de la dirección de columna
    sh1106_send_cmd(0x10 + ((x >> 4) & 0x0F)); // Bits altos de la dirección de columna
}

void sh1106_send_data_raw(uint8_t data)
{
    uint8_t buf[2];
    buf[0] = 0x40; // Control byte: Co = 0, D/C# = 1 → siguiente byte(s) son datos
    buf[1] = data;

    i2c_master_write_to_device(I2C_MASTER_NUM, OLED_ADDR, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

void fill_test_circle(uint8_t *bmp, uint8_t w, uint8_t h)
{
    int cx = w / 2, cy = h / 2, r = 31;
    for (int y = 0; y < h; y++)
    {
        for (int x = 0; x < w; x++)
        {
            int dx = x - cx;
            int dy = y - cy;
            if (dx * dx + dy * dy >= r * r - r && dx * dx + dy * dy <= r * r + r)
                bmp[y * w + x] = 1;
        }
    }
}

void sh1106_draw_bitmap(uint8_t x, uint8_t page, const uint8_t *bitmap, uint8_t w, uint8_t h)
{
    // Convierte bitmaps horizontales (1 byte por píxel) al formato vertical que necesita el SH1106
    for (uint8_t page_idx = 0; page_idx < h / 8; page_idx++)
    {
        sh1106_set_cursor(x, page + page_idx);

        for (uint8_t col = 0; col < w; col++)
        {
            uint8_t byte = 0;
            for (uint8_t bit = 0; bit < 8; bit++)
            {
                uint8_t pixel = bitmap[(page_idx * 8 + bit) * w + col];
                if (pixel)
                    byte |= (1 << bit);
            }
            sh1106_send_data_raw(byte);
        }
    }
}

//------------------------------------------------------------------------------------------------
void sh1106_send_cmd(uint8_t cmd)
{
    uint8_t data[2] = {0x00, cmd};
    i2c_master_write_to_device(I2C_MASTER_NUM, OLED_ADDR, data, 2, pdMS_TO_TICKS(100));
}
//------------------------------------------------------------------------------------------------
void sh1106_send_data(const uint8_t *data, size_t len)
{
    uint8_t *buf = malloc(len + 1);
    buf[0] = 0x40;
    memcpy(buf + 1, data, len);
    i2c_master_write_to_device(I2C_MASTER_NUM, OLED_ADDR, buf, len + 1, pdMS_TO_TICKS(100));
    free(buf);
}
//------------------------------------------------------------------------------------------------
void sh1106_init(void)
{
    sh1106_send_cmd(0xAE); // display off
    sh1106_send_cmd(0xD5);
    sh1106_send_cmd(0x80); // clock divide
    sh1106_send_cmd(0xA8);
    sh1106_send_cmd(0x3F); // multiplex
    sh1106_send_cmd(0xD3);
    sh1106_send_cmd(0x00); // offset
    sh1106_send_cmd(0x40); // start line
    sh1106_send_cmd(0xAD);
    sh1106_send_cmd(0x8B); // charge pump

    // sh1106_send_cmd(0xA1); // segment remap              NORMAL
    // sh1106_send_cmd(0xC8); // COM scan direction         NORMAL

    sh1106_send_cmd(0xA0); // UP SIDE DOWN
    sh1106_send_cmd(0xC0); // UP SIDE DOWN

    sh1106_send_cmd(0xDA);
    sh1106_send_cmd(0x12); // com pins

    sh1106_send_cmd(0x81);
    sh1106_send_cmd(0xFF); // contrast

    sh1106_send_cmd(0xA4); // resume RAM content
    sh1106_send_cmd(0xA6); // normal display
    sh1106_send_cmd(0xAF); // display ON
}
//------------------------------------------------------------------------------------------------
void sh1106_clear(void)
{
    uint8_t zero[128] = {0};
    for (int page = 0; page < SH1106_PAGES; page++)
    {
        sh1106_send_cmd(0xB0 + page);
        sh1106_send_cmd(0x02); // lower column start
        sh1106_send_cmd(0x10); // higher column start
        sh1106_send_data(zero, SH1106_WIDTH);
    }
}
//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------
void sh1106_draw_char(uint8_t x, uint8_t page, char c)
{
    int index;

    if (c == ' ')
    {
        index = 0;
    }
    else if (c == '!')
    {
        index = 1;
    }
    else if (c >= 'A' && c <= 'Z')
    {
        index = (c - 'A') + 2;
    }
    else if (c >= 'a' && c <= 'z')
    {
        index = (c - 'a') + 28;
    }
    else if (c >= '0' && c <= '9')
    {
        index = (c - '0') + 54;
    }
    else if (c == ':')
    {
        index = 64;
    }
    else if (c == '-')
    {
        index = 65;
    }
    else if (c == '.')
    {
        index = 66;
    }
    else if (c == '^')
    {
        index = 67; // Símbolo de grado
    }
    else
    {
        index = 0;
    }

    sh1106_send_cmd(0xB0 + page);

    uint8_t column = x + SH1106_COL_OFFSET;

    sh1106_send_cmd(0x00 | (column & 0x0F));
    sh1106_send_cmd(0x10 | ((column >> 4) & 0x0F));

    sh1106_send_data(font5x7[index], 5);

    uint8_t spacing = 0x00;
    sh1106_send_data(&spacing, 1);
}
//------------------------------------------------------------------------------------------------
void sh1106_draw_text(uint8_t x, uint8_t page, const char *text)
{
    while (*text)
    {
        sh1106_draw_char(x, page, *text);
        x += 6;
        text++;
    }
}
