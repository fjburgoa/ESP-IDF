#ifndef _SH1106_H
#define _SH1106_H

//static esp_err_t i2c_master_init(void) ;
void sh1106_send_cmd(uint8_t cmd);
void sh1106_send_data(const uint8_t *data, size_t len) ;
void sh1106_init(void) ;
void sh1106_clear(void) ;
void sh1106_draw_char(uint8_t x, uint8_t page, char c) ;
void sh1106_draw_text(uint8_t x, uint8_t page, const char *text);
void sh1106_draw_bitmap(uint8_t x, uint8_t page, const uint8_t *bitmap, uint8_t w, uint8_t h);

void fill_test_circle(uint8_t *bmp, uint8_t w, uint8_t h);

            
#endif