#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "SH1106.h"   // tu header con sh1106_init(), sh1106_clear(), sh1106_set_cursor(), etc.

#define TAG "BALL"

uint8_t fb[PAGES][W];

// --- utilidades framebuffer ---
static inline void fb_clear(void)
{
    memset(fb, 0, sizeof(fb));
}

static inline void fb_set_pixel(int x, int y, int on)
{
    if (x < 0 || x >= W || y < 0 || y >= H) return;
    int page = y >> 3;
    int bit  = y & 7;
    if (on) fb[page][x] |=  (1 << bit);
    else    fb[page][x] &= ~(1 << bit);
}

// Círculo relleno (pelota)
static void fb_draw_filled_circle(int cx, int cy, int r)
{
    int r2 = r * r;
    for (int y = cy - r; y <= cy + r; y++) {
        for (int x = cx - r; x <= cx + r; x++) {
            int dx = x - cx;
            int dy = y - cy;
            if (dx*dx + dy*dy <= r2) {
                fb_set_pixel(x, y, 1);
            }
        }
    }
}



void app_main(void)
{
    i2c_master_init();
    sh1106_init();
    sh1106_clear();

    char texto[32] = "BOUNCING BALL";
    sh1106_draw_text(0, 3, texto); 
    vTaskDelay(pdMS_TO_TICKS(6000));


    // Estado pelota
    int x = 20, y = 20;
    int vx = 2, vy = 2;
    const int r = 8;          // radio pelota
    const int dt_ms = 20;     // ~28 FPS

    while (1) {
        fb_clear();
        fb_draw_filled_circle(x, y, r);
        sh1106_update_from_fb();

        x += vx;
        y += vy;

        // rebote (considerando radio)
        if (x <= r) { x = r; vx = -vx; }
        if (x >= (W - 1 - r)) { x = W - 1 - r; vx = -vx; }

        if (y <= r) { y = r; vy = -vy; }
        if (y >= (H - 1 - r)) { y = H - 1 - r; vy = -vy; }

        vTaskDelay(pdMS_TO_TICKS(dt_ms));
    }
}
