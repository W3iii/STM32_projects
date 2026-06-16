/*
 * ssd1306.h
 *
 *  Created on: May 13, 2026
 *      Author: xushu
 */

#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>

/* 初始化 */
void ssd1306_init(void);

/* framebuffer 操作（不會自動更新螢幕） */
void ssd1306_clear(void);
void ssd1306_fill(void);
void ssd1306_draw_pixel (uint8_t x, uint8_t y, uint8_t color);
void ssd1306_draw_char  (uint8_t x, uint8_t y, char c);
void ssd1306_draw_string(uint8_t x, uint8_t y, const char *s);
void ssd1306_draw_int   (uint8_t x, uint8_t y, int32_t n);

/* 把 framebuffer 推到 OLED（必須手動呼叫才會顯示） */
void ssd1306_update_screen(void);

/* 非同步：啟動後立刻 return，要自己呼叫 wait */
void ssd1306_update_screen_async(void);
void ssd1306_wait_update_done   (void);
int  ssd1306_is_updating        (void);

#endif // SSD1306_H
