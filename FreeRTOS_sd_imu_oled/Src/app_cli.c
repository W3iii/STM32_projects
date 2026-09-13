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
#include "bus_port.h"           /* i2cscan 指令：i2c1_lock / i2c1_unlock */
#include "device_tempsensor.h"  /* status：晶片溫度 raw / 換算除錯讀數 */
#include "device_mpu6050.h"     /* i2cstuck 指令：MPU6050_ADDR / who_am_i */
#include "driver_i2c1.h"        /* i2cscan 指令：i2c1_bus_scan / i2c1_bus_recovery */
#include "driver_usart1.h"
#include "FreeRTOS.h"           /* tasks 指令：run-time stats */
#include "task.h"              /* uxTaskGetSystemState：State/Pri/Stack 三欄 */
#include "app_cpu_stats.h"     /* CPU% / Peak% / Avg%：自建統計，見該檔檔頭 */
#include <string.h>
/* 注意：本檔不再 #include "ff.h"、也不再呼叫 spi1_lock —— 所有 FatFs/SD 存取
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

/* 印 "0xNN"（i2cscan 用）。 */
static void cli_print_hex8(uint8_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    usart1_send_string("0x");
    usart1_send_byte((uint8_t)hex[(v >> 4) & 0xF]);
    usart1_send_byte((uint8_t)hex[v & 0xF]);
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

    /* SD 沒掛上時系統會進降級模式：CLI/OLED/i2cscan 都還在，只是 start 會失敗。
     * 這一行要放最前面 —— 掛載失敗時它才是你第一個該看到的東西。 */
    if (!app_datalogger_mounted()) {
        cli_label("SD");
        usart1_send_string("NOT MOUNTED (logging disabled, FRESULT ");
        cli_print_u32((uint32_t)app_datalogger_mount_result());
        usart1_send_string(")\r\n");
    } else if (app_datalogger_mount_retries() > 0) {
        cli_label("SD");
        usart1_send_string("mounted after ");
        cli_print_u32((uint32_t)app_datalogger_mount_retries());
        usart1_send_string(" retry (power margin?)\r\n");
    }

    cli_label("state");
    usart1_send_string(st.state == DL_LOGGING ? "LOGGING\r\n"
                     : st.state == DL_ERROR   ? "ERROR (write failed)\r\n"
                                              : "IDLE\r\n");
    if (st.state == DL_LOGGING || st.state == DL_ERROR) {
        cli_label("file");
        usart1_send_string(st.filename);
        usart1_send_string("\r\n");
        cli_label("samples");
        cli_print_u32(st.count);
        usart1_send_string("\r\n");
    }

    /* 寫入錯誤。非 0 就代表有樣本沒被寫進卡片 —— 以前這件事完全沒有出口，
     * state 停在 LOGGING、計數不動，看起來像「還在錄」。
     * fres 是 FatFs 的 FRESULT（1=FR_DISK_ERR、2=FR_INT_ERR、9=FR_INVALID_OBJECT…），
     * FR_DISK_ERR 多半是供電/接觸不良，或卡片忙碌超過 SD_WRITE_TIMEOUT_MS。 */
    cli_label("write err");
    cli_print_u32(st.err_count);
    if (st.fres != 0) {
        usart1_send_string(" (FRESULT ");
        cli_print_u32(st.fres);
        usart1_send_string(")");
    }
    usart1_send_string("\r\n");

    cli_label("overruns");
    cli_print_u32(app_tasks_overruns());    /* Sensor 因佇列滿丟棄數，理想 0 */
    usart1_send_string("\r\n");

    /* Sensor 連續 I2C 讀取失敗而觸發 bus recovery 的次數，理想 0。
     * 非 0 代表匯流排曾經卡住 —— 通常是供電、接觸或拉電阻的問題。 */
    cli_label("i2c recov");
    cli_print_u32(app_tasks_i2c_recovers());
    usart1_send_string("\r\n");

    /* 非同步 I2C 的失敗次數，依類型分開 —— 三者指向完全不同的方向，
     * 混成一個數字就沒辦法對症下藥。理想全 0；偶爾跳 nack 在麵包板上
     * 算正常（Display 對 INA219 有一次重試，所以畫面不一定會閃 "--"）。 */
    {
        uint32_t nack = 0, bus = 0, tmo = 0;
        i2c1_async_err_counts(&nack, &bus, &tmo);
        cli_label("i2c err");
        usart1_send_string("nack ");
        cli_print_u32(nack);
        usart1_send_string("  bus ");
        cli_print_u32(bus);
        usart1_send_string("  timeout ");
        cli_print_u32(tmo);
        usart1_send_string("\r\n");
    }

    /* 晶片溫度（VREFINT 補償後）＋ 反推的實際 VDDA。die 溫，非室溫。 */
    cli_label("chip");
    cli_print_fixed(tempsensor_read_c(), 1);
    usart1_send_string(" C  (VDDA ");
    cli_print_fixed(tempsensor_vdda(), 2);
    usart1_send_string(" V)\r\n");
}

/* dl_dump 的 per-line 回呼：把一行印到 UART，行尾 \n 補 \r 才不會階梯。
 * 注意：此函式由 dl_dump() 在「持有 spi1_lock 期間」回呼 → 只能碰 UART，
 * 不可碰 SD/OLED（同一支 spi1_lock）。 */
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
 * 實際 FatFs 讀取在 app_datalogger 的 dl_dump() 內、包在 spi1_lock 裡。 */
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

/* dl_list 的 per-file 回呼：印檔名 + 大小。同樣在 spi1_lock 期間被回呼，只碰 UART。 */
static void cli_emit_file(const char *name, uint32_t size)
{
    usart1_send_string("  ");
    usart1_send_string(name);
    usart1_send_string("  ");
    cli_print_u32(size);
    usart1_send_string(" B\r\n");
}

/* 列出根目錄的檔案 + 大小，再印剩餘空間。像真的設備 console。
 * 目錄掃描 + getfree 都在 dl_list() 內、包在 spi1_lock 裡。 */
static void cli_ls(void)
{
    uint32_t free_kb = 0;
    uint8_t  fres    = 0;
    int n = dl_list(cli_emit_file, &free_kb, &fres);
    if (n < 0) {
        /* 讀失敗跟空目錄是兩回事：1=FR_DISK_ERR（供電/接觸）、3=FR_NOT_READY。
         * 再打一次 ls 常常就好了 —— 那本身就是供電邊界的徵兆。 */
        usart1_send_string("dir read failed (FRESULT ");
        cli_print_u32(fres);
        usart1_send_string(") - retry, or check power\r\n");
        return;
    }
    if (n == 0) {
        usart1_send_string("  (no files)\r\n");
    }
    usart1_send_string("free: ");
    if (free_kb == DL_FREE_KB_UNKNOWN) {
        usart1_send_string("unknown (getfree failed)\r\n");
    } else {
        cli_print_u32(free_kb);
        usart1_send_string(" KB\r\n");
    }
}

/* 刪掉所有 log_NNN.csv（重複 demo 前清場用）。實際 unlink 迴圈在 dl_clear()。 */
static void cli_clear(void)
{
    if (app_datalogger_state() == DL_LOGGING) {
        usart1_send_string("stop first\r\n");
        return;
    }
    uint32_t deleted = 0;
    uint8_t  fres    = 0;
    int rc = dl_clear(&deleted, &fres);
    usart1_send_string("deleted ");
    cli_print_u32(deleted);
    usart1_send_string(" files");
    if (rc != 0) {
        /* 沒刪完。數量本身不代表卡上原本就只有這些。 */
        usart1_send_string(" then ABORTED (FRESULT ");
        cli_print_u32(fres);
        usart1_send_string(") - files remain");
    }
    usart1_send_string("\r\n");
}

/* ───────────────── tasks：FreeRTOS 執行期監控 ─────────────────
 *   tasks            一次快照：狀態 / 優先權 / 堆疊水位 / CPU 佔用
 *   tasks hist [t]   最近 6.4 秒的 CPU% 時間軸（t = task 名稱，省略則全部）
 *   tasks reset      清掉峰值與時間軸，重新開始觀察
 *
 * 資料來源分兩邊，各取所長：
 *   State / Pri / Stack   uxTaskGetSystemState()（需 configUSE_TRACE_FACILITY=1）
 *   CPU% / Peak% / Avg%   app_cpu_stats.c 的自建統計
 *
 * 為什麼百分比不用 kernel 內建的 ulRunTimeCounter：它把 ISR 時間算到剛好被
 * 打斷的那支 task 頭上，而且漏掉當下執行中 task 的未結算切片（於是 Σ 各
 * task% 湊不到 100%）。自建那份把時間拆成 各 task / [ISR] / [SCHED]，
 * 三者相加就是牆鐘時間。完整說明見 app_cpu_stats.h 檔頭。
 *
 * 三個百分比欄位的分工：
 *   CPU%   最近 1 秒（10 筆 100ms 取樣）的平均 —— 「現在」的負載
 *   Peak%  單一 100ms 取樣的歷史最大值 —— 抓瞬間尖峰。logger 每秒 f_sync
 *          一次，SD 卡 FAT 更新遇到 block erase 可以停上百毫秒，秒級平均
 *          會把它整個抹平；Sensor 的 overrun 多半就是這樣來的（見 status）。
 *   Avg%   開機至今的累積平均 —— 長期基準線
 *
 * 取樣由 Display task 以 10Hz 驅動（app_tasks.c），不需要打指令也一直在記，
 * 所以錄製中的尖峰事後用 hist / Peak% 就查得到，不必掐準時機打 tasks。 */

#define CLI_MAX_TASKS   12      /* 5 app task + IDLE，留餘裕 */

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

/* 印「千分比」成 "12.3" 並補空白到欄寬（一位小數：Sensor 這種短命 task
 * 的佔用常在個位數，整數欄位會全部擠成 0 或 1，看不出變化）。 */
static void cli_pct10_field(uint32_t p10, int width)
{
    char b[8];
    int  i = 0;
    uint32_t w = p10 / 10U;
    if (w >= 100U) { b[i++] = (char)('0' + (w / 100U) % 10U); }
    if (w >= 10U)  { b[i++] = (char)('0' + (w / 10U)  % 10U); }
    b[i++] = (char)('0' + w % 10U);
    b[i++] = '.';
    b[i++] = (char)('0' + p10 % 10U);
    b[i]   = '\0';
    cli_field(b, width);
}

/* 在 uxTaskGetSystemState 的結果裡找同一支 task（用 handle 比對，不用名字：
 * 同名不保證是同一支）。找不到回 -1 —— [ISR] / [SCHED] 這兩個虛擬列就是。 */
static int cli_find_status(const TaskStatus_t *st, UBaseType_t n, void *handle)
{
    if (handle != NULL) {
        for (UBaseType_t i = 0; i < n; i++) {
            if ((void *)st[i].xHandle == handle) {
                return (int)i;
            }
        }
    }
    return -1;
}

/* 把千分比壓成一個字元的濃淡，用來畫時間軸。 */
static char cli_spark(uint16_t pm)
{
    if (pm <  25U) return ' ';      /* < 2.5% */
    if (pm < 100U) return '.';      /* < 10%  */
    if (pm < 250U) return ':';      /* < 25%  */
    if (pm < 500U) return '-';      /* < 50%  */
    if (pm < 750U) return '=';      /* < 75%  */
    return '#';                     /* >= 75% */
}

/* ---- tasks（無參數）：目前狀態總覽 ---- */
static void cli_tasks_table(void)
{
    /* static：這兩個陣列合計 656 bytes（432 + 224），別放 task stack 上。 */
    static TaskStatus_t     st[CLI_MAX_TASKS];
    static cpustats_slot_t  sl[CPUSTATS_SLOTS];

    UBaseType_t n_st = uxTaskGetSystemState(st, CLI_MAX_TASKS, NULL);
    uint32_t    n_sl = cpustats_get(sl, CPUSTATS_SLOTS);

    usart1_send_string("Task         State Pri Stack CPU%  Peak% Avg%\r\n");
    for (uint32_t i = 0; i < n_sl; i++) {
        int k = cli_find_status(st, n_st, sl[i].handle);

        cli_field(sl[i].name, 13);
        if (k >= 0) {
            cli_field(cli_state_str(st[k].eCurrentState), 6);
            cli_uint_field((uint32_t)st[k].uxCurrentPriority, 4);
            /* 剩餘堆疊（words），越小越接近爆掉 */
            cli_uint_field((uint32_t)st[k].usStackHighWaterMark, 6);
        } else {
            /* [ISR] / [SCHED] 沒有 task 實體，這三欄留白；已刪除的 task 也走這裡 */
            cli_field("-", 6);
            cli_field("-", 4);
            cli_field("-", 6);
        }
        cli_pct10_field(sl[i].cur_pm,  6);
        cli_pct10_field(sl[i].peak_pm, 6);
        cli_pct10_field(sl[i].avg_pm,  5);
        usart1_send_string("\r\n");
    }
    usart1_send_string("CPU%=last 1s  Peak%=worst 100ms sample  Avg%=since boot\r\n");
}

/* ---- tasks hist：時間軸 ---- */
static void cli_tasks_hist(const char *want)
{
    static cpustats_slot_t sl[CPUSTATS_SLOTS];
    static uint16_t        h[CPUSTATS_HIST_LEN];

    uint32_t n_sl  = cpustats_get(sl, CPUSTATS_SLOTS);
    int      found = 0;

    usart1_send_string("legend: ' '<2.5%  '.'<10%  ':'<25%  '-'<50%  '='<75%  '#'>=75%\r\n");
    cli_field("", 10);                  /* 空出名字欄，讓標尺對齊時間軸 */
    usart1_send_string("older");
    for (uint32_t i = 0; i < (CPUSTATS_HIST_LEN - 5U - 3U); i++) {
        usart1_send_byte(' ');
    }
    usart1_send_string("now\r\n");

    for (uint32_t i = 0; i < n_sl; i++) {
        uint32_t n;

        if ((want != NULL) && (want[0] != '\0') && (strcmp(want, sl[i].name) != 0)) {
            continue;
        }
        found = 1;

        cli_field(sl[i].name, 10);
        n = cpustats_hist(sl[i].slot, h, CPUSTATS_HIST_LEN);
        for (uint32_t k = 0; k < n; k++) {
            usart1_send_byte((uint8_t)cli_spark(h[k]));
        }
        usart1_send_string("\r\n");
    }

    if (!found) {
        usart1_send_string("no such task (run 'tasks' for the list)\r\n");
    } else {
        /* 一格 = 一次取樣，換算成秒讓「尖峰在幾秒前」看得出來 */
        usart1_send_string("one column = ");
        cli_print_u32(CPUSTATS_SAMPLE_MS);
        usart1_send_string(" ms, full width = ");
        cli_print_fixed((float)(CPUSTATS_HIST_LEN * CPUSTATS_SAMPLE_MS) / 1000.0f, 1);
        usart1_send_string(" s\r\n");
    }
}

static void cli_cmd_tasks(const char *arg)
{
    if (strncmp(arg, "hist", 4) == 0) {
        const char *t = arg + 4;
        while (*t == ' ') {
            t++;                    /* 跳過 "hist" 後面的空白，剩下的是 task 名 */
        }
        cli_tasks_hist(t);
    } else if (strcmp(arg, "reset") == 0) {
        cpustats_reset();
        usart1_send_string("peaks + timeline cleared\r\n");
    } else {
        cli_tasks_table();
    }
}

/* ───────────────── wfi：IDLE 省電開關 ─────────────────
 *   wfi        看目前狀態
 *   wfi on     IDLE 沒事做時 WFI 睡覺（預設）
 *   wfi off    IDLE 全速空轉，當對照組
 *
 * 配 power 指令就是一組現成的省電量測：wfi off → power → wfi on → power，
 * 兩次電流差就是「核心空轉」燒掉的部分。開關與說明在 app_tasks.c。 */
static void cli_wfi(const char *arg)
{
    if (strcmp(arg, "on") == 0) {
        g_wfi_enabled = 1;
    } else if (strcmp(arg, "off") == 0) {
        g_wfi_enabled = 0;
    } else if (arg[0] != '\0') {
        usart1_send_string("usage: wfi [on|off]\r\n");
        return;
    }
    usart1_send_string("wfi: ");
    usart1_send_string(g_wfi_enabled ? "on (idle sleeps)\r\n" : "off (idle spins)\r\n");
}

/* ───────────────── power：INA219 讀數（最近 10 筆平均）─────────────────
 * 不自己碰 I2C —— 量測由 Display task 以 10Hz 做，這裡只取它存下的資料。
 * 好處：CLI 不必上 I2C 鎖、不會和 Sensor 搶 bus，指令回應也是瞬間的。
 *
 * 印的是「最近 10 筆的平均」而不是最新一筆：單筆讀數偶爾會跳出離群值
 * （I2C 雜訊、量測窗剛好撞上 SD 寫入 / OLED 更新的湧浪電流），拿它當
 * 功耗數字會誤判。10 筆 @10Hz = 最近 1 秒，尖峰被攤成 1/10，量 wfi on/off
 * 這種對照時兩次讀數才穩定可比。
 * 代價：反應慢一拍 —— 負載剛變化時要等約 1 秒平均窗才換完。
 * 想看瞬時值，OLED 第 3 行印的仍是最新一筆。 */
static void cli_power(void)
{
    ina219_data_t p;
    uint32_t      n;
    if (!app_tasks_get_power_avg(&p, &n)) {
        usart1_send_string("INA219 not available (check wiring, try: i2cscan)\r\n");
        return;
    }

    cli_label("bus");
    cli_print_fixed(p.bus_v, 3);
    usart1_send_string(" V\r\n");

    cli_label("shunt");
    cli_print_fixed(p.shunt_mv, 2);
    usart1_send_string(" mV\r\n");

    cli_label("current");
    cli_print_fixed(p.current_ma, 1);
    usart1_send_string(" mA\r\n");

    cli_label("power");
    cli_print_fixed(p.power_mw, 1);
    usart1_send_string(" mW\r\n");

    /* 把樣本數印出來，讀數才有可信度可言：開機後第一秒內會少於 10。 */
    cli_label("avg of");
    cli_print_u32(n);
    usart1_send_string(" samples @10Hz\r\n");
}

/* ───────────────── i2cscan：I2C1 匯流排掃描 ─────────────────
 * 掃 0x08~0x77，對每個位址送 START + addr+W，回 ACK 就代表線上有這顆。
 * 用途：新裝置（INA219）並上 PB8/PB9 後，先確認「位址真的出現、原有的
 * MPU6050 也還在」，再去寫 driver —— 位址衝突 / 沒接好在這裡就看得出來。
 *
 * 整段掃描包在 i2c1_lock 內：和 Sensor task 的 mpu6050_read_all() 互斥，
 * 不會有兩個 task 在同一條 bus 上互相插 START 把狀態機弄亂。
 *
 * 為什麼仍要求先 stop（這是延遲考量，不是正確性）：
 *   掃描 112 個位址期間一直抱著 I2C1 鎖，而且 driver 是 busy-wait 不讓出 CPU。
 *   正常情況約 15ms → Sensor 掉 1~2 筆樣本；但只要有一條線被拉死，每次探測
 *   都會吃滿 I2C_TIMEOUT_MS(100ms)，最慢 11s，100Hz 取樣等於整段停擺。
 *   i2cscan 本來就是「接線階段」用的，要求先 stop 最省事。
 *   （若之後想在記錄中也能掃，把這個 guard 拿掉即可，鎖已經保證安全。）*/

#define CLI_I2C_MAX_FOUND   16

/* 已知位址的猜測標籤，純粹方便看（不代表真的驗過 WHO_AM_I）。 */
static const char *cli_i2c_guess(uint8_t addr)
{
    if (addr == 0x68 || addr == 0x69) return "MPU6050 (AD0 low/high)";
    if (addr >= 0x40 && addr <= 0x4F) return "INA219 (A1/A0 straps)";
    if (addr == 0x3C || addr == 0x3D) return "SSD1306 (I2C ver.)";
    return "?";
}

static void cli_cmd_i2cscan(void)
{
    if (app_datalogger_state() == DL_LOGGING) {
        usart1_send_string("stop first (scan would stall 100Hz sampling)\r\n");
        return;
    }

    usart1_send_string("scanning I2C1 (PB8=SCL PB9=SDA) 0x08..0x77 ...\r\n");

    uint8_t found[CLI_I2C_MAX_FOUND];
    i2c1_lock();
    int n = i2c1_bus_scan(found, CLI_I2C_MAX_FOUND);
    i2c1_unlock();

    for (int i = 0; i < n; i++) {
        usart1_send_string("  ");
        cli_print_hex8(found[i]);
        usart1_send_string("  ");
        usart1_send_string(cli_i2c_guess(found[i]));
        usart1_send_string("\r\n");
    }

    if (n == 0) {
        /* 一顆都沒有 → 多半是沒供電 / SDA-SCL 接反 / 缺 pull-up；
         * 順手試一次 recovery（slave 把 SDA 卡在 low 的話能救回來）。 */
        usart1_send_string("  (none) check VCC/GND, SDA-SCL swap, pull-ups\r\n");
        usart1_send_string("  bus recovery: ");
        i2c1_lock();                    /* recovery 會 disable I2C1、手動搖 SCL，同樣要獨佔 */
        i2c_status_t rc = i2c1_bus_recovery();
        i2c1_unlock();
        usart1_send_string(rc == I2C_OK            ? "ok\r\n"
                         : rc == I2C_ERR_SCL_STUCK ? "SCL held low (dead slave / short / no pull-up)\r\n"
                                                   : "SDA still low\r\n");
        return;
    }

    usart1_send_string("found ");
    cli_print_u32((uint32_t)n);
    usart1_send_string(" device(s)");
    if (n == CLI_I2C_MAX_FOUND) {
        usart1_send_string(" (list full, raise CLI_I2C_MAX_FOUND)");
    }
    usart1_send_string("\r\n");
}

#if I2C1_FAULT_INJECT
/*
 * i2cstuck —— 故障注入，驗證 bus recovery 真的能救回卡住的匯流排。
 *
 *   i2cstuck        注入後立刻手動 recovery，三行輸出把整個流程驗完
 *   i2cstuck raw    只注入就放手，讓 Sensor task 的自動路徑去救
 *
 * raw 模式是端到端測試：離開時 I2C1 是 disabled，之後每筆 mpu6050_read_all()
 * 都會吃滿逾時，連續 SENSOR_I2C_FAIL_LIMIT 次後 app_tasks.c 觸發 recovery。
 * 驗收方式是 status 的 i2c recov 從 0 變 1、取樣自動續跑（CSV 中間少幾筆）。
 * 要先 start 讓 Sensor task 在跑，否則沒人會去救，匯流排會一直卡著。
 */
static void cli_cmd_i2cstuck(const char *arg)
{
    int raw = (strcmp(arg, "raw") == 0);

    i2c1_lock();
    int stuck = i2c1_inject_sda_stuck(MPU6050_ADDR);
    usart1_send_string(stuck ? "  inject:    SDA held low by slave\r\n"
                             : "  inject:    failed (no ACK - check addr/power)\r\n");

    if (raw) {
        i2c1_unlock();      /* 匯流排就這樣壞著交出去 */
        usart1_send_string("  left broken: Sensor task should recover in ~250ms\r\n");
        usart1_send_string("  check 'status' i2c recov (needs logging active)\r\n");
        return;
    }

    i2c_status_t rc = i2c1_bus_recovery();
    usart1_send_string("  recovery:  ");
    usart1_send_string(rc == I2C_OK            ? "ok\r\n"
                     : rc == I2C_ERR_SCL_STUCK ? "SCL held low (dead slave / short / no pull-up)\r\n"
                                               : "SDA still low\r\n");

    uint8_t id = mpu6050_who_am_i();     /* recovery 之後匯流排真的能用嗎 */
    i2c1_unlock();

    usart1_send_string("  who_am_i:  ");
    cli_print_hex8(id);          /* 自己會印 "0x" 前綴 */
    usart1_send_string(id == 0x68 ? "  (bus healthy)\r\n" : "  (expected 0x68)\r\n");
}
#endif

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
        "  tasks          FreeRTOS task monitor (CPU% / peak / stack)\r\n"
        "  tasks hist [t] CPU% timeline, last 6.4s (t = task name)\r\n"
        "  tasks reset    clear peaks + timeline\r\n"
        "  power          INA219 V / I / P (avg of last 10 samples)\r\n"
        "  wfi [on|off]   idle WFI sleep (power saving)\r\n"
        "  i2cscan        scan I2C1 bus (0x08..0x77)\r\n"
#if I2C1_FAULT_INJECT
        "  i2cstuck [raw] jam SDA to test bus recovery (debug build)\r\n"
#endif
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
        int start_rc = app_datalogger_start();
        if (start_rc == 0) {
            usart1_send_string("started: ");
            usart1_send_string(app_datalogger_filename());
            usart1_send_string("\r\n");
        } else if (start_rc == -1) {
            /* 掛載失敗（含當場重掛也沒過）。這條路徑不經過 f_open，所以
             * s_fres 是 0 —— 要印的是 f_mount 的 FRESULT，不是寫入的。 */
            usart1_send_string("start failed: SD not mounted (FRESULT ");
            cli_print_u32((uint32_t)app_datalogger_mount_result());
            usart1_send_string(") - check power/card\r\n");
        } else {
            /* 開檔失敗。1=FR_DISK_ERR、3=FR_NOT_READY 指向供電或接觸，
             * 7=FR_DENIED 是空間。 */
            dl_status_t st;
            app_datalogger_get_status(&st);
            usart1_send_string("start failed");
            if (st.fres != 0) {
                usart1_send_string(" (FRESULT ");
                cli_print_u32(st.fres);
                usart1_send_string(")");
            }
            usart1_send_string("\r\n");
        }
    } else if (strcmp(cmd, "stop") == 0) {
        if (app_datalogger_stop() == 0) {
            usart1_send_string("stopped\r\n");
        } else {
            /* 收尾寫入失敗 = 這個 session 的資料沒落地。必須說出來。 */
            dl_status_t st;
            app_datalogger_get_status(&st);
            usart1_send_string("stopped, but FLUSH FAILED (FRESULT ");
            cli_print_u32(st.fres);
            usart1_send_string(") - data may be lost\r\n");
        }
    } else if (strcmp(cmd, "status") == 0) {
        cli_status();
    } else if (strcmp(cmd, "dump") == 0) {
        cli_dump(arg);
    } else if (strcmp(cmd, "ls") == 0) {
        cli_ls();
    } else if (strcmp(cmd, "clear") == 0) {
        cli_clear();
    } else if (strcmp(cmd, "tasks") == 0) {
        cli_cmd_tasks(arg);
    } else if (strcmp(cmd, "power") == 0) {
        cli_power();
    } else if (strcmp(cmd, "wfi") == 0) {
        cli_wfi(arg);
    } else if (strcmp(cmd, "i2cscan") == 0) {
        cli_cmd_i2cscan();
#if I2C1_FAULT_INJECT
    } else if (strcmp(cmd, "i2cstuck") == 0) {
        cli_cmd_i2cstuck(arg);
#endif
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