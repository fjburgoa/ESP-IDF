#ifndef _SH1106_H
#define _SH1106_H

// Ajusta a tu pantalla
#define W 128
#define H 64
#define PAGES (H/8)

// Framebuffer nativo: 8 páginas * 128 columnas = 1024 bytes
extern uint8_t fb[PAGES][W];

void i2c_master_init(void);

//static esp_err_t i2c_master_init(void) ;
void sh1106_send_cmd(uint8_t cmd);
void sh1106_send_data(const uint8_t *data, size_t len) ;
void sh1106_init(void) ;
void sh1106_clear(void) ;
void sh1106_draw_char(uint8_t x, uint8_t page, char c) ;
void sh1106_draw_text(uint8_t x, uint8_t page, const char *text);
void sh1106_draw_bitmap(uint8_t x, uint8_t page, const uint8_t *bitmap, uint8_t w, uint8_t h);
void sh1106_set_cursor(uint8_t x, uint8_t page);

void sh1106_update_from_fb(void);

void fill_test_circle(uint8_t *bmp, uint8_t w, uint8_t h);

            
#endif