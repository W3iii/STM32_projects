/*
 * app_button.h
 *
 *  User button（STM32F407-Discovery B1 = PA0, active-high）。
 *  time-based 去抖。只負責回報「按下」事件，不決定按下要做什麼——
 *  start/stop 切換由 main 處理（driver 只報狀態）。
 *
 *  Created on: Jun 19, 2026
 *      Author: xushu
 */

#ifndef APP_BUTTON_H_
#define APP_BUTTON_H_

#include <stdint.h>

/* 初始化 user button：開 GPIOA clock、PA0 設為 input。
 * 需先有 systick（去抖靠 systick_get_ms() 計時）。 */
void app_button_init(void);

/* 主迴圈每圈呼叫：取樣 + 去抖。
 * 回傳 1 = 這次偵測到「確認的按下邊緣」（release→press）；否則 0。
 * now_ms 傳 systick_get_ms()。 */
int app_button_poll(uint32_t now_ms);

#endif /* APP_BUTTON_H_ */
