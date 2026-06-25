/*
 * app_cli.c
 *
 *  UART console：start / stop / status。
 *  ─────────────────────────────────────────────
 *  用 usart1_try_read_byte() 從 RX ring buffer 一個一個取，自己組行，
 *  '\r' 或 '\n' 當行結尾 —— 這樣 Tera Term 送 CR / LF / CRLF 都吃得下。
 *
 *  Created on: Jun 18, 2026
 *      Author: xushu
 */

#include "app_cli.h"
#include "app_datalogger.h"
#include "app_tasks.h"          /* app_tasks_overruns（status 用）*/
#include "driver_usart1.h"
#include <string.h>
/* 注意：本檔不再 #include "ff.h"、也不再呼叫 bus_lock —— 所有 FatFs/SD 存取
 * 都委派給 app_datalogger 的 dl_list/dl_dump/dl_clear，CLI 只負責解析與印字。 */

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
    dl_status_t st;
    app_datalogger_get_status(&st);     /* 一次取走快照（mState 保護）*/

    if (st.state == DL_LOGGING) {
        usart1_send_string("state=LOGGING file=");
        usart1_send_string(st.filename);
        usart1_send_string(" samples=");
        cli_print_u32(st.count);
        usart1_send_string("\r\n");
    } else {
        usart1_send_string("state=IDLE\r\n");
    }
    usart1_send_string("overruns=");
    cli_print_u32(app_tasks_overruns());    /* Sensor 因佇列滿丟棄數，理想 0 */
    usart1_send_string("\r\n");
}

/* dl_dump 的 per-line 回呼：把一行印到 UART，行尾 \n 補 \r 才不會階梯。
 * 注意：此函式由 dl_dump() 在「持有 bus_lock 期間」回呼 → 只能碰 UART，
 * 不可碰 SD/OLED（同一支 bus_lock）。 */
static void cli_emit_line(const char *line)
{
    for (const char *p = line; *p; p++) {
        if (*p == '\n') {
            usart1_send_byte('\r');
        }
        usart1_send_byte((uint8_t)*p);
    }
}

/* 把一個 CSV 從卡讀回來印到 UART（沒讀卡機也能驗證）。
 *   dump        → 本次 session 剛錄/上一個檔（app_datalogger_filename()）
 *   dump <file> → 指定檔名（配 ls 用，可印上一輪開機留下的 log）
 * 無參數版需要先 stop（記錄中檔案被開著寫，不重複開）。
 * 實際 FatFs 讀取在 app_datalogger 的 dl_dump() 內、包在 bus_lock 裡。 */
static void cli_dump(const char *arg)
{
    const char *fn;

    if (arg[0] != '\0') {
        fn = arg;                       /* dump <file>：直接開指定檔 */
    } else {
        if (app_datalogger_state() == DL_LOGGING) {
            usart1_send_string("stop first (or: dump <file>)\r\n");
            return;
        }
        fn = app_datalogger_filename();
        if (fn[0] == '\0') {            /* 這次開機還沒 start 過 */
            usart1_send_string("no session file (try: ls, then dump <file>)\r\n");
            return;
        }
    }

    usart1_send_string("--- ");
    usart1_send_string(fn);
    usart1_send_string(" ---\r\n");
    if (dl_dump(fn, cli_emit_line) != 0) {
        usart1_send_string("open failed\r\n");
        return;
    }
    usart1_send_string("--- end ---\r\n");
}

/* dl_list 的 per-file 回呼：印檔名 + 大小。同樣在 bus_lock 期間被回呼，只碰 UART。 */
static void cli_emit_file(const char *name, uint32_t size)
{
    usart1_send_string("  ");
    usart1_send_string(name);
    usart1_send_string("  ");
    cli_print_u32(size);
    usart1_send_string(" B\r\n");
}

/* 列出根目錄的檔案 + 大小，再印剩餘空間。像真的設備 console。
 * 目錄掃描 + getfree 都在 dl_list() 內、包在 bus_lock 裡。 */
static void cli_ls(void)
{
    uint32_t free_kb = 0;
    int n = dl_list(cli_emit_file, &free_kb);
    if (n < 0) {
        usart1_send_string("opendir failed\r\n");
        return;
    }
    if (n == 0) {
        usart1_send_string("  (no files)\r\n");
    }
    usart1_send_string("free: ");
    cli_print_u32(free_kb);
    usart1_send_string(" KB\r\n");
}

/* 刪掉所有 log_NNN.csv（重複 demo 前清場用）。實際 unlink 迴圈在 dl_clear()。 */
static void cli_clear(void)
{
    if (app_datalogger_state() == DL_LOGGING) {
        usart1_send_string("stop first\r\n");
        return;
    }
    uint32_t deleted = 0;
    dl_clear(&deleted);
    usart1_send_string("deleted ");
    cli_print_u32(deleted);
    usart1_send_string(" files\r\n");
}

/* 把行切成「指令 + 參數」：第一個空白前是指令，之後（去頭空白）是參數。
 * 直接在 buffer 上切（塞 '\0'），參數空字串代表沒給。 */
static void cli_dispatch(char *line)
{
    char *cmd = line;
    char *arg = line;
    while (*arg && *arg != ' ') {
        arg++;                      /* 走到指令結尾 */
    }
    while (*arg == ' ') {
        *arg++ = '\0';              /* 砍掉分隔空白，arg 指向參數開頭 */
    }

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
        cli_dump(arg);
    } else if (strcmp(cmd, "ls") == 0) {
        cli_ls();
    } else if (strcmp(cmd, "clear") == 0) {
        cli_clear();
    } else {
        usart1_send_string("? cmds: start | stop | status | dump [file] | ls | clear\r\n");
    }
    usart1_send_string("> ");
}

void app_cli_init(void)
{
    s_idx = 0;
    usart1_send_string("CLI ready: start | stop | status | dump [file] | ls | clear\r\n> ");
}

void app_cli_poll(void)
{
    uint8_t c;
    while (usart1_try_read_byte(&c) == USART1_OK) {
        if (c == '\r' || c == '\n') {
            /* 只在有內容時才處理：CRLF 終端機的第二個 \n（此時 s_idx==0）
             * 會被這裡略過，不會跑出兩次 prompt。 */
            if (s_idx > 0) {
                usart1_send_string("\r\n");     // echo 換行
                s_line[s_idx] = '\0';
                cli_dispatch(s_line);           // 內部會印 "> "
                s_idx = 0;
            }
        } else if (c == '\b' || c == 0x7F) {    // backspace / DEL
            if (s_idx > 0) {
                s_idx--;
                usart1_send_string("\b \b");     // 終端機上把字擦掉
            }
        } else if (c >= 0x20 && c < 0x7F && s_idx < sizeof(s_line) - 1) {
            s_line[s_idx++] = (char)c;
            usart1_send_byte(c);                 // echo 可見字元
        }
    }
}