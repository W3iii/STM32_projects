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
#include "device_tempsensor.h"  /* status：晶片溫度 raw / 換算除錯讀數 */
#include "driver_usart1.h"
#include "FreeRTOS.h"           /* tasks 指令：run-time stats */
#include "task.h"              /* uxTaskGetSystemState */
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

/* 送字串後補空白到欄寬 width（左對齊）。表格 / key-value 對齊共用。 */
static void cli_field(const char *s, int width)
{
    int n = 0;
    for (const char *p = s; *p; p++) {
        usart1_send_byte((uint8_t)*p);
        n++;
    }
    while (n < width) {
        usart1_send_byte(' ');
        n++;
    }
}

/* 送 uint（左對齊）後補空白到欄寬 width。 */
static void cli_uint_field(uint32_t v, int width)
{
    char b[11];
    int  i = 10;
    b[i] = '\0';
    if (v == 0) {
        b[--i] = '0';
    } else {
        while (v && i > 0) {
            b[--i] = (char)('0' + (v % 10));
            v /= 10;
        }
    }
    cli_field(&b[i], width);
}

/* 印 float 到固定小數位（usart 沒 printf，自己轉），例如 41.2 / 3.01。 */
static void cli_print_fixed(float v, int decimals)
{
    int32_t scale = 1;
    for (int i = 0; i < decimals; i++) {
        scale *= 10;
    }
    int   neg = (v < 0.0f);
    float mag = neg ? -v : v;
    int32_t scaled = (int32_t)(mag * (float)scale + 0.5f);
    if (neg && scaled != 0) {
        usart1_send_byte('-');
    }
    cli_print_u32((uint32_t)(scaled / scale));
    if (decimals > 0) {
        usart1_send_byte('.');
        int32_t frac = scaled % scale;
        for (int32_t p = scale / 10; p >= 1; p /= 10) {
            usart1_send_byte((uint8_t)('0' + (frac / p) % 10));
        }
    }
}

/* status 的一列標籤：縮排兩格 + 標籤左對齊到 10 欄。 */
static void cli_label(const char *name)
{
    usart1_send_string("  ");
    cli_field(name, 10);
}

static void cli_status(void)
{
    dl_status_t st;
    app_datalogger_get_status(&st);     /* 一次取走快照（mState 保護）*/

    cli_label("state");
    usart1_send_string(st.state == DL_LOGGING ? "LOGGING\r\n" : "IDLE\r\n");
    if (st.state == DL_LOGGING) {
        cli_label("file");
        usart1_send_string(st.filename);
        usart1_send_string("\r\n");
        cli_label("samples");
        cli_print_u32(st.count);
        usart1_send_string("\r\n");
    }
    cli_label("overruns");
    cli_print_u32(app_tasks_overruns());    /* Sensor 因佇列滿丟棄數，理想 0 */
    usart1_send_string("\r\n");

    /* 晶片溫度（VREFINT 補償後）＋ 反推的實際 VDDA。die 溫，非室溫。 */
    cli_label("chip");
    cli_print_fixed(tempsensor_read_c(), 1);
    usart1_send_string(" C  (VDDA ");
    cli_print_fixed(tempsensor_vdda(), 2);
    usart1_send_string(" V)\r\n");
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

/* ───────────────── tasks：FreeRTOS 執行期監控 ─────────────────
 * 一次快照全部 task（含 IDLE / Timer service）的狀態、優先權、堆疊剩餘水位、
 * CPU 佔用％。資料源是 uxTaskGetSystemState()（需 configUSE_TRACE_FACILITY=1），
 * CPU% 靠 run-time stats 計時器（TIM2，見 app_runtime_stats.c）。
 * 純唯讀 FreeRTOS 內部狀態，不碰 SD/OLED、不需 bus_lock、不新增 task/mutex。 */

#define CLI_MAX_TASKS   12      /* 5 app task + IDLE，留餘裕（cli_field / cli_uint_field 定義在檔案上方）*/

static const char *cli_state_str(eTaskState st)
{
    switch (st) {
    case eRunning:   return "Run";
    case eReady:     return "Rdy";
    case eBlocked:   return "Blk";
    case eSuspended: return "Susp";
    case eDeleted:   return "Del";
    default:         return "?";
    }
}

static void cli_cmd_tasks(void)
{
    static TaskStatus_t st[CLI_MAX_TASKS];   /* static：TaskStatus_t 陣列別放 stack */
    uint32_t total_run = 0;

    UBaseType_t n = uxTaskGetSystemState(st, CLI_MAX_TASKS, &total_run);
    if (n == 0) {
        usart1_send_string("uxTaskGetSystemState failed (raise CLI_MAX_TASKS)\r\n");
        return;
    }

    uint32_t div = total_run / 100U;         /* 1% 對應的 counter 量（避免溢位）*/

    usart1_send_string("Task         State Pri Stack CPU%\r\n");
    for (UBaseType_t i = 0; i < n; i++) {
        cli_field(st[i].pcTaskName, 13);
        cli_field(cli_state_str(st[i].eCurrentState), 6);
        cli_uint_field((uint32_t)st[i].uxCurrentPriority, 4);
        cli_uint_field((uint32_t)st[i].usStackHighWaterMark, 6);   /* 剩餘堆疊（words），越小越接近爆 */
        uint32_t pct = (div != 0U) ? (st[i].ulRunTimeCounter / div) : 0U;
        cli_uint_field(pct, 3);
        usart1_send_string("%\r\n");
    }
}

/* 對齊的指令清單（開機、help、打錯指令都印這個）。 */
static void cli_print_help(void)
{
    usart1_send_string(
        "commands:\r\n"
        "  start          begin logging (new log_NNN.csv)\r\n"
        "  stop           stop logging\r\n"
        "  status         logger state + chip temp\r\n"
        "  dump [file]    print a csv (default: last session)\r\n"
        "  ls             list files + free space\r\n"
        "  clear          delete all logs\r\n"
        "  tasks          FreeRTOS task monitor\r\n"
        "  help           this list\r\n");
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
    } else if (strcmp(cmd, "tasks") == 0) {
        cli_cmd_tasks();
    } else if (strcmp(cmd, "help") == 0) {
        cli_print_help();
    } else {
        usart1_send_string("unknown command '");
        usart1_send_string(cmd);
        usart1_send_string("' (type 'help')\r\n");
    }
    usart1_send_string("> ");
}

void app_cli_init(void)
{
    s_idx = 0;
    cli_print_help();
    usart1_send_string("> ");
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