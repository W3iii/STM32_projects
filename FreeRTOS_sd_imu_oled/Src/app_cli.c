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
#include "driver_i2c1.h"        /* i2cscan 指令：i2c1_bus_scan / i2c1_bus_recovery */
#include "driver_usart1.h"
#include "FreeRTOS.h"           /* tasks 指令：run-time stats */
#include "task.h"              /* uxTaskGetSystemState */
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
 * 計時源是 run-time stats 計時器（TIM2 @100kHz，見 app_runtime_stats.c）。
 * 純唯讀 FreeRTOS 內部狀態，不碰 SD/OLED、不需 spi1_lock、不新增 task/mutex。
 *
 * CPU% 有兩欄，差別很重要：
 *   Avg%   開機到現在的累積佔用 = ulRunTimeCounter / total。開機後大部分時間
 *          在 IDLE，所以錄製中的負載會被前面那段閒置稀釋掉，看不出真實負荷。
 *   CPU%   距離「上一次執行 tasks」這段區間的佔用，也就是兩次快照的差值：
 *              (累積@t1 - 累積@t0) / (total@t1 - total@t0)
 *          要量「錄製中 Sensor 吃多少 CPU」就看這欄：start 後打一次 tasks
 *          當基準，跑個幾秒再打一次，第二次的 CPU% 就是那段區間的實際值。
 *
 * 每次呼叫都會把基準換成這次的快照，所以連打兩次 tasks 得到的是「這兩次之間」。
 *
 * 計數器繞回（TIM2 32-bit @100kHz 約 11.9 小時）不影響：uint32 減法本身
 * 就是模 2^32 運算，只要兩次呼叫間隔不超過一整圈，差值仍然正確。 */

#define CLI_MAX_TASKS   12      /* 5 app task + IDLE，留餘裕（cli_field / cli_uint_field 定義在檔案上方）*/

/* 上一次快照（算差值用）。用 xHandle 當識別，不用名字或索引：
 * task 在列表裡的順序會變，同名也不保證是同一支。 */
static TaskHandle_t s_prev_handle[CLI_MAX_TASKS];
static uint32_t     s_prev_rt[CLI_MAX_TASKS];
static UBaseType_t  s_prev_n;
static uint32_t     s_prev_total;
static TickType_t   s_prev_tick;
static uint8_t      s_have_base;        /* 0 = 還沒有基準，CPU% 印 "-" */

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

/* 印「百分比 ×10」成 "12.3" 並補空白到欄寬（一位小數：Sensor 這種短命 task
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

/* 在上一次快照裡找同一支 task；找到回索引，沒有回 -1（這次才新建的 task）。 */
static int cli_prev_index(TaskHandle_t h)
{
    for (UBaseType_t i = 0; i < s_prev_n; i++) {
        if (s_prev_handle[i] == h) {
            return (int)i;
        }
    }
    return -1;
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

    TickType_t now = xTaskGetTickCount();

    /* 兩個分母。除以 1000 = 「0.1% 對應多少 counter」，先除再除避免
     * delta*1000 溢位（區間夠長時 delta 會很大）。 */
    uint32_t delta_total = total_run - s_prev_total;      /* uint32 減法自帶繞回處理 */
    uint32_t div_int     = delta_total / 1000U;           /* 區間用 */
    uint32_t div_avg     = total_run   / 1000U;           /* 累積用 */

    /* 表頭前先講清楚這次的 CPU% 涵蓋多長的區間，不然數字沒有意義。 */
    if (!s_have_base) {
        usart1_send_string("baseline captured - run 'tasks' again for interval CPU%\r\n");
    } else {
        usart1_send_string("interval: ");
        cli_print_fixed((float)(now - s_prev_tick) / 1000.0f, 1);
        usart1_send_string(" s\r\n");
    }

    usart1_send_string("Task         State Pri Stack CPU%  Avg%\r\n");
    for (UBaseType_t i = 0; i < n; i++) {
        cli_field(st[i].pcTaskName, 13);
        cli_field(cli_state_str(st[i].eCurrentState), 6);
        cli_uint_field((uint32_t)st[i].uxCurrentPriority, 4);
        cli_uint_field((uint32_t)st[i].usStackHighWaterMark, 6);   /* 剩餘堆疊（words），越小越接近爆 */

        /* 區間佔用：這支 task 在上一次快照裡的累積值找得到才算得出來。 */
        int p = s_have_base ? cli_prev_index(st[i].xHandle) : -1;
        if (p < 0 || div_int == 0U) {
            cli_field("-", 6);              /* 沒基準 / 新 task / 區間太短 */
        } else {
            uint32_t d = st[i].ulRunTimeCounter - s_prev_rt[p];
            cli_pct10_field(d / div_int, 6);
        }

        /* 累積佔用（原本的行為）。 */
        cli_pct10_field((div_avg != 0U) ? (st[i].ulRunTimeCounter / div_avg) : 0U, 5);
        usart1_send_string("\r\n");
    }

    /* 這次的快照變成下次的基準。 */
    for (UBaseType_t i = 0; i < n; i++) {
        s_prev_handle[i] = st[i].xHandle;
        s_prev_rt[i]     = st[i].ulRunTimeCounter;
    }
    s_prev_n     = n;
    s_prev_total = total_run;
    s_prev_tick  = now;
    s_have_base  = 1;
}

/* ───────────────── power：INA219 讀數 ─────────────────
 * 不自己碰 I2C —— 量測由 Display task 以 10Hz 做，這裡只取它存下的快照
 * （app_tasks_get_power 內用 critical section 複製，四個值同一時刻）。
 * 好處：CLI 不必上 I2C 鎖、不會和 Sensor 搶 bus，指令回應也是瞬間的。
 * 代價：讀數最舊可能落後 100ms —— 對看電源狀態完全夠用。 */
static void cli_power(void)
{
    ina219_data_t p;
    if (!app_tasks_get_power(&p)) {
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
        usart1_send_string(rc == I2C_OK ? "ok\r\n" : "SDA still low\r\n");
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
        "  power          INA219 voltage / current / power\r\n"
        "  i2cscan        scan I2C1 bus (0x08..0x77)\r\n"
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
    } else if (strcmp(cmd, "power") == 0) {
        cli_power();
    } else if (strcmp(cmd, "i2cscan") == 0) {
        cli_cmd_i2cscan();
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