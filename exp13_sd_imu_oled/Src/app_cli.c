/*
 * app_cli.c
 *
 *  UART console：start / stop / status。
 *  ─────────────────────────────────────────────
 *  用 usart1_read_byte() 從 RX ring buffer 一個一個取，自己組行，
 *  '\r' 或 '\n' 當行結尾 —— 這樣 Tera Term 送 CR / LF / CRLF 都吃得下。
 *
 *  Created on: Jun 18, 2026
 *      Author: xushu
 */

#include "app_cli.h"
#include "app_datalogger.h"
#include "driver_usart1.h"
#include "ff.h"
#include <string.h>

static char    s_line[32];
static uint8_t s_idx;

/* 印一個 uint32 的十進位（usart 沒有 printf，自己轉）。 */
static void cli_print_u32(uint32_t v)
{
    char b[11];
    int  i = 10;
    b[i] = '\0';
    if (v == 0) {
        usart1_send_string("0");
        return;
    }
    while (v && i > 0) {
        b[--i] = (char)('0' + (v % 10));
        v /= 10;
    }
    usart1_send_string(&b[i]);
}

static void cli_status(void)
{
    if (app_datalogger_state() == DL_LOGGING) {
        usart1_send_string("state=LOGGING file=");
        usart1_send_string(app_datalogger_filename());
        usart1_send_string(" samples=");
        cli_print_u32(app_datalogger_count());
        usart1_send_string("\r\n");
    } else {
        usart1_send_string("state=IDLE\r\n");
    }
}

/* 把目前/上一個 CSV 從卡讀回來印到 UART（沒讀卡機也能驗證）。
 * 需要先 stop（記錄中檔案被開著寫，不重複開）。 */
static void cli_dump(void)
{
    static FIL f;            /* static：FIL 很大，別放 stack */
    char line[64];

    if (app_datalogger_state() == DL_LOGGING) {
        usart1_send_string("stop first\r\n");
        return;
    }
    const char *fn = app_datalogger_filename();
    if (fn[0] == '\0') {
        usart1_send_string("no file yet\r\n");
        return;
    }
    if (f_open(&f, fn, FA_READ) != FR_OK) {
        usart1_send_string("open failed\r\n");
        return;
    }
    usart1_send_string("--- ");
    usart1_send_string(fn);
    usart1_send_string(" ---\r\n");
    while (f_gets(line, sizeof line, &f) != 0) {
        for (char *p = line; *p; p++) {     /* 行尾 \n 補 \r 才不會階梯 */
            if (*p == '\n') {
                usart1_send_byte('\r');
            }
            usart1_send_byte((uint8_t)*p);
        }
    }
    f_close(&f);
    usart1_send_string("--- end ---\r\n");
}

static void cli_dispatch(const char *cmd)
{
    if (strcmp(cmd, "start") == 0) {
        if (app_datalogger_start() == 0) {
            usart1_send_string("started: ");
            usart1_send_string(app_datalogger_filename());
            usart1_send_string("\r\n");
        } else {
            usart1_send_string("start failed\r\n");
        }
    } else if (strcmp(cmd, "stop") == 0) {
        app_datalogger_stop();
        usart1_send_string("stopped\r\n");
    } else if (strcmp(cmd, "status") == 0) {
        cli_status();
    } else if (strcmp(cmd, "dump") == 0) {
        cli_dump();
    } else {
        usart1_send_string("? cmds: start | stop | status | dump\r\n");
    }
    usart1_send_string("> ");
}

void app_cli_init(void)
{
    s_idx = 0;
    usart1_send_string("CLI ready: start | stop | status | dump\r\n> ");
}

void app_cli_poll(void)
{
    uint8_t c;
    while (usart1_read_byte(&c) == USART1_OK) {
        if (c == '\r' || c == '\n') {
            if (s_idx > 0) {
                s_line[s_idx] = '\0';
                cli_dispatch(s_line);
                s_idx = 0;
            }
        } else if (s_idx < sizeof(s_line) - 1) {
            s_line[s_idx++] = (char)c;
        }
    }
}
