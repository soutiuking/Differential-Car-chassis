/**
  ******************************************************************************
  * @file    ssd1306.h
  * @brief   0.96 寸 128×64 SSD1306 OLED 驱动（I2C1，地址 0x3C）。
  ******************************************************************************
  */
#ifndef SSD1306_H
#define SSD1306_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

bool ssd1306_init(void);
void ssd1306_clear(void);
void ssd1306_clear_page(uint8_t page);
void ssd1306_draw_text(uint8_t x, uint8_t page, const char *text);
void ssd1306_draw_i32(uint8_t x, uint8_t page, int32_t value, uint8_t width);
bool ssd1306_flush_page(uint8_t page);

#ifdef __cplusplus
}
#endif

#endif /* SSD1306_H */
