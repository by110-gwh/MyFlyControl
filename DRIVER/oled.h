#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

void oled_init(void);
void oled_w_dat(uint8_t dat);
void oled_w_cmd(uint8_t cmd);
void oled_set_pos(uint8_t x, uint8_t y);
void oled_cls(void);
void oled_6x8_str(uint8_t x, uint8_t y, uint8_t ch[]);
void oled_6x8_char(uint8_t x, uint8_t y, uint8_t ucData);
void oled_6x8_number(uint8_t x, uint8_t y, float number);
void oled_8x16_str(uint8_t x, uint8_t y, uint8_t ch[]);
void oled_8x16_char(uint8_t x, uint8_t y, uint8_t ch);
void oled_8x16_number(uint8_t x, uint8_t y, float number);
void oled_clear_line(uint8_t x, uint8_t y);



#endif
