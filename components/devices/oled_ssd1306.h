#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H

#include "struct_typedef.h"

#define OLED_SSD1306_WIDTH   128u
#define OLED_SSD1306_HEIGHT  64u

uint8_t oled_ssd1306_init(void);
void oled_ssd1306_clear(void);
void oled_ssd1306_update(void);
void oled_ssd1306_write_char(uint8_t x, uint8_t y, char ch);
void oled_ssd1306_write_string(uint8_t x, uint8_t y, const char *str);

#endif
