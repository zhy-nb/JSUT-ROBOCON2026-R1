/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       oled_ssd1306.h
  * @brief      SSD1306 I2C OLED driver interface.
  * @brief      SSD1306 I2C OLED 驱动接口。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */
#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H

#include "struct_typedef.h"

#define OLED_SSD1306_WIDTH   128u
#define OLED_SSD1306_HEIGHT  64u

/**
  * @brief  Initialize OLED hardware and internal framebuffer.
  * @brief  初始化 OLED 硬件和内部显存。
  */
uint8_t oled_ssd1306_init(void);

/**
  * @brief  Clear internal framebuffer.
  * @brief  清空内部显存。
  */
void oled_ssd1306_clear(void);

/**
  * @brief  Push framebuffer content to OLED.
  * @brief  将显存内容刷新到 OLED。
  */
void oled_ssd1306_update(void);

/**
  * @brief  Write one character into framebuffer.
  * @brief  向显存写入单个字符。
  */
void oled_ssd1306_write_char(uint8_t x, uint8_t y, char ch);

/**
  * @brief  Write one string into framebuffer.
  * @brief  向显存写入字符串。
  */
void oled_ssd1306_write_string(uint8_t x, uint8_t y, const char *str);

#endif
