/*
 * app_datalogger.c
 *
 *  IMU 資料記錄器 — Application 層核心（FreeRTOS 版）。
 *  ─────────────────────────────────────────────
 *  與 bare-metal 版的差別：
 *    - 取樣（讀 MPU）移到 Sensor task；本模組只負責「寫入 + 狀態」。
 *    - 所有 FatFs/SD 操作包在 spi1_lock()/spi1_unlock()（SPI1 匯流排互斥，
 *      和 OLED 共用同一支鎖，見 bus_port.h / bus_port_freertos.c）。
 *    - 狀態（state/count/filename）由自有的 s_state_mtx 保護，給 Display /
 *      CLI status 用 get_status() 一次取走。
 *
 *  鎖序：需要時 spi1_lock 在外、s_state_mtx 在內（只有 writer 會巢狀），
 *  讀者只取 s_state_mtx，故不會死鎖。
 *
 *  緩衝策略不變：FatFs 的 FIL 內建 512B sector 緩衝，f_printf 先累積，
 *  滿一個 sector 才真的寫 SD；另外每秒 f_sync 把 FAT/目錄落地（SPEC §6.3）。
 *
 *  Created on: Jun 18, 2026
 *      Author: xushu
 */

#include "app_datalogger.h"
#include "bus_port.h"       /* spi1_lock（FatFs/SD）*/
#include "ff.h"
#include "diskio.h"     /* disk_initialize：只重置卡、不換掉 volume（見 dl_salvage_locked）*/
#include "device_sdcard.h"  /* sd_check_alive：IDLE 時偵測拔卡（FatFs 沒有對應的 API）*/
#include "device_mpu6050.h"   /* mpu6050_axes_t（write_sample 的參數）*/
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"       /* TaskHandle_t / xTaskNotifyGive */

static FATFS      s_fs;
static FIL        s_fil;
static dl_state_t s_state = DL_IDLE;
static uint32_t   s_count;
static char       s_filename[16];   /* "log_NNN.csv" + '\0' */
static uint8_t    s_mounted;

static SemaphoreHandle_t s_state_mtx;   /* 保護 s_state / s_count / s_filename */
static uint8_t    s_fres;               /* 最後一次寫入失敗的 FatFs FRESULT */
static uint32_t   s_err_count;          /* 累計寫入失敗次數 */
static uint8_t    s_mount_retries;      /* 開機掛載重試了幾次（0 = 一次就過）*/
static uint8_t    s_fil_orphan;         /* DL_ERROR 留下的舊檔還沒收拾（s_fil 仍掛在舊 volume）*/
static uint32_t   s_recover_count;      /* DL_ERROR 自動/手動恢復成功次數（累計）*/
static uint32_t   s_remount_tries;      /* IDLE 沒掛載時 remount 實際試過幾次（累計）*/

static TaskHandle_t s_producer;         /* sensor task handle；start 成功時喚醒它 */

/* 診斷：volume 目前為什麼沒掛上（s_mounted = 0 時才有意義）。
 * 來源不只 f_mount：DL_ERROR 收拾舊檔失敗時是 f_close 的結果，卡重置起不來
 * 或 IDLE 偵測到拔卡時直接記 FR_NOT_READY。讀的人：開機 FS FAIL 畫面、
 * OLED 的 NO SD、status 的 NOT MOUNTED、start 失敗訊息。
 *   1=FR_DISK_ERR、3=FR_NOT_READY → SPI/卡片硬體；13=FR_NO_FILESYSTEM → 不是 FAT32 */
static volatile FRESULT s_mount_res = FR_OK;

static inline void state_lock(void)   { xSemaphoreTake(s_state_mtx, portMAX_DELAY); }
static inline void state_unlock(void) { xSemaphoreGive(s_state_mtx); }

/* 找第一個還沒被用的 log_NNN.csv（NNN = 000..999）。 */
/* 找第一個沒被占用的 log_NNN.csv。
 *
 * 回傳 FR_OK = s_filename 可用；其他值 = 卡讀不動，呼叫端該放棄。
 * 「讀不動」必須跟「這個名字是空的」分開：只有 FR_NO_FILE 才代表可用，
 * FR_DISK_ERR / FR_NOT_READY 這些以前會被當成「已被占用」繼續往下找，
 * 於是在壞掉的卡上要磨完 1000 圈、每圈各吃一次 SD 逾時（實測 start 要等
 * 十幾秒才回 "start failed"），最後還沿用 log_999.csv 去開檔。 */
static FRESULT dl_make_filename(void)
{
    FILINFO fno;
    for (int i = 0; i < 1000; i++) {
        s_filename[0] = 'l';
        s_filename[1] = 'o';
        s_filename[2] = 'g';
        s_filename[3] = '_';
        s_filename[4] = (char)('0' + (i / 100));
        s_filename[5] = (char)('0' + (i / 10) % 10);
        s_filename[6] = (char)('0' + (i % 10));
        s_filename[7] = '.';
        s_filename[8] = 'c';
        s_filename[9] = 's';
        s_filename[10] = 'v';
        s_filename[11] = '\0';
        FRESULT fr = f_stat(s_filename, &fno);
        if (fr == FR_NO_FILE) {
            return FR_OK;           /* 這個沒人用，就用它 */
        }
        if (fr != FR_OK) {
            return fr;              /* 卡讀不動，再找下去只是浪費時間 */
        }
    }
    /* 000..999 都被用了：沿用迴圈最後設定的 log_999.csv（會被覆蓋）*/
    return FR_OK;
}

/* 開新檔 + 寫表頭。呼叫端持有 spi1_lock。 */
static FRESULT dl_open_new_locked(void)
{
    FRESULT fr = dl_make_filename();    /* 寫 s_filename（在 bus 內、稍後 state 內公布）*/
    if (fr == FR_OK) {
        fr = f_open(&s_fil, s_filename, FA_WRITE | FA_CREATE_ALWAYS);
    }
    if (fr == FR_OK) {
        f_puts("t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c\n", &s_fil);
    }
    return fr;
}

/* 收拾 DL_ERROR 留下的舊檔。呼叫端持有 spi1_lock。
 *
 * 寫入失敗時卡多半只是卡住了（供電邊界下 busy 一直不放），從 CMD0 重新
 * 初始化就會好 —— 實測 stop 再 start 就恢復。但重新初始化不能走 f_mount：
 * volume 一換掉，s_fil 就失效，fp->buf 裡沒寫出去的那個 sector、檔案長度、
 * FAT 鏈全部跟著丟，舊檔只剩上一次 f_sync 的內容。
 *
 * 所以直接呼叫 disk_initialize() 只重置卡，volume 原封不動，再 f_close。
 * f_sync 不看 fp->err（只有 f_read/f_write 看），卡好了就會把 dirty sector、
 * FAT、目錄項依序寫回去，舊檔救到失敗前最後一筆（最後一行可能只寫了半行）。
 *
 * give_up：卡還起不來時要不要放棄舊檔。自動恢復傳 0（留著下一秒再救），
 * 使用者 stop 傳 1（stop 不能卡在一張死卡上）。
 * 回傳 FR_OK = 收好了或本來就沒有舊檔；卡沒起來且 give_up = 0 時回
 * FR_NOT_READY，s_fil_orphan 維持 1。 */
static FRESULT dl_salvage_locked(int give_up)
{
    if (!s_fil_orphan) {
        return FR_OK;
    }
    FRESULT fr;
    if (disk_initialize(0) & STA_NOINIT) {
        if (!give_up) {
            return FR_NOT_READY;
        }
        fr = FR_NOT_READY;
    } else {
        fr = f_close(&s_fil);
    }
    s_fil_orphan = 0;

    if (fr == FR_OK) {
        s_mounted = 1;              /* 卡好了、volume 沒動過，直接可以開新檔 */
    } else {
        /* 救不回來：卸載 volume，讓 s_fil 跟 FatFs 的 FAT 快取一起作廢，
         * 下次重新掛載，不帶著可能不一致的快取繼續寫。 */
        f_mount(NULL, "", 0);
        s_mounted   = 0;
        s_mount_res = fr;           /* status 的 NOT MOUNTED 才印得出原因 */
    }
    return fr;
}

int app_datalogger_init(void)
{
    s_state_mtx = xSemaphoreCreateMutex();
    if (s_state_mtx == NULL) {
        return -3;
    }
    /* MPU6050 不在這裡 init 了：以前 MPU 沒過就直接 return、連卡都不掛，
     * 而 MPU 之後插上去也沒人回頭掛卡，只能重開機。MPU 的在線狀態改由
     * app_tasks 管（開機探一次，之後 Display 每秒重探），跟 SD 互不牽連。 */

    /* 掛載重試：f_mount(opt=1) 會呼叫 disk_initialize() → sd_init()，而
     * diskio 沒有快取狀態，每次都真的重跑一次初始化，所以重試是有意義的。
     *
     * 為什麼需要：實測會「偶爾」掛不起來，FRESULT=3 (FR_NOT_READY) 代表
     * CMD0/ACMD41 沒走完 —— SD 卡在初始化期間對供電最敏感，而板子目前是
     * ST-LINK USB 供電，3.3V LDO 的餘裕本來就很小。這是邊界狀態，不是
     * 確定性失敗，所以重試很有效。
     *
     * 每次之間隔 200ms 讓電源與卡片內部穩定；三次都失敗才真的放棄。
     * 跑在 bootstrap task 脈絡，vTaskDelay 合法。 */
    for (int attempt = 0; attempt < 3; attempt++) {
        if (attempt > 0) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        s_mount_res = f_mount(&s_fs, "", 1);
        if (s_mount_res == FR_OK) {
            break;
        }
        s_mount_retries++;
    }
    if (s_mount_res != FR_OK) {
        return -2;                  /* 掛載失敗：FRESULT 存在 s_mount_res */
    }
    s_mounted = 1;
    s_state = DL_IDLE;
    return 0;
}

int app_datalogger_mount_result(void)
{
    return (int)s_mount_res;        /* 診斷用：volume 為什麼沒掛上，見 s_mount_res */
}

int app_datalogger_start(void)
{
    /* 從 DL_ERROR 直接 start：跟自動恢復走同一條路（先救舊檔再開新檔）。
     * 不能先跑下面的 f_mount —— 那會換掉 volume，舊檔沒寫回去的部分就沒了。 */
    if (app_datalogger_state() == DL_ERROR) {
        return app_datalogger_recover();
    }

    if (!s_mounted) {
        /* 開機那三次掛載失敗，不代表卡從此不能用 —— 供電邊界下卡往往只是
         * 慢了一步（FRESULT 3 = FR_NOT_READY）。但 s_mounted 以前是開機一鎚
         * 定音，之後就算卡完全正常也解不開，只能重開機。
         *
         * 實測踩到：剛完整格式化過的卡，開機第一次 f_mount 沒過，之後 ls
         * 讀得好好的、free 也正確（FatFs 在下一次 API 呼叫時自己重掛成功了），
         * 唯獨 start 一直回 -1 —— 而且那條路徑不設 s_fres，CLI 只能印一句
         * 沒有任何資訊的 "start failed"。
         *
         * 所以這裡當場重掛一次。f_mount(opt=1) 會真的重跑 disk_initialize()
         * → sd_init()，是實質重試而不是只改旗標。 */
        spi1_lock();
        s_mount_res = f_mount(&s_fs, "", 1);
        spi1_unlock();
        if (s_mount_res != FR_OK) {
            return -1;              /* 原因在 app_datalogger_mount_result() */
        }
        s_mounted = 1;
        state_lock();
        s_state = DL_IDLE;
        state_unlock();
    }
    if (app_datalogger_state() == DL_LOGGING) {
        return 0;                   /* 已在記錄 */
    }

    int     rc = 0;
    FRESULT fr;
    spi1_lock();
    fr = dl_open_new_locked();
    if (fr != FR_OK) {
        rc = -2;
    }
    spi1_unlock();

    if (rc != 0) {
        /* 失敗原因留在 s_fres，讓 status 印得出 FRESULT。沒有這個的話
         * CLI 只能說 "start failed"，分不出 FR_DISK_ERR(1) 是供電/接觸，
         * 還是 FR_NOT_READY(3) 是卡根本沒認到。 */
        state_lock();
        s_fres = (uint8_t)fr;
        state_unlock();
        return rc;
    }

    state_lock();
    s_count = 0;
    s_fres  = 0;            /* 新 session：清掉上一次的錯誤碼（err_count 累計保留）*/
    s_state = DL_LOGGING;
    state_unlock();

    /* 事件驅動：喚醒 sensor 進 100Hz 取樣迴圈。只在 IDLE→LOGGING 成功時發。
     * 不變量：IDLE ⟺ sensor 卡在 ulTaskNotifyTake()，所以這個通知一定接得到、
     * 不會掉，也不會在 sensor 跑迴圈時被 latch 成假喚醒。stop 不發通知。 */
    if (s_producer != NULL) {
        xTaskNotifyGive(s_producer);
    }
    return 0;
}

int app_datalogger_remount(void)
{
    int rc = -1;

    /* 只在 IDLE 重掛。DL_ERROR 時 s_mounted 也是 0，但 s_fil 還掛在舊 volume
     * 上等 recover 收拾 —— 這裡一 f_mount 就把它作廢，舊檔沒寫回去的部分全丟。
     *
     * 跟 start 的重掛可能同時發生：兩邊都在 spi1_lock 內 f_mount，而且都是
     * 「掛好才開檔」，所以最壞只是同一個 volume 多掛一次，不會換掉開著的檔。 */
    spi1_lock();
    if (s_mounted) {
        rc = 0;
    } else if (app_datalogger_state() == DL_IDLE) {
        s_remount_tries++;
        s_mount_res = f_mount(&s_fs, "", 1);
        if (s_mount_res == FR_OK) {
            s_mounted = 1;
            rc = 0;
        }
    }
    spi1_unlock();
    return rc;
}

int app_datalogger_check_card(void)
{
    int rc = 0;

    /* 只在 IDLE 且已掛載時問。LOGGING 時寫入失敗就會進 DL_ERROR，不必另外問；
     * DL_ERROR 歸 recover 管。
     *
     * 失敗要連兩次才算拔卡：麵包板上偶發一次雜訊就卸載，雖然下一秒會重掛回來，
     * 但白白重跑一次 init。卡真的不在的話，第二次一樣失敗，只多花幾十 µs。 */
    spi1_lock();
    if (s_mounted && app_datalogger_state() == DL_IDLE
        && sd_check_alive() != SD_OK && sd_check_alive() != SD_OK) {
        /* 跟 dl_salvage_locked 救不回來時同一套：卸載 volume，丟掉 FatFs 對
         * 這張卡的快取。插回來的可能是另一張卡，不能帶著舊的 FAT 快取繼續用。 */
        f_mount(NULL, "", 0);
        s_mounted   = 0;
        s_mount_res = FR_NOT_READY;
        rc = -1;
    }
    spi1_unlock();
    return rc;
}

int app_datalogger_recover(void)
{
    int     rc = 0;
    FRESULT fr = FR_OK;

    /* 整段持有 spi1_lock，並在鎖內才檢查狀態：自動恢復（Logger task）和
     * CLI/按鈕的 start 可能同時進來，後到的會看到已經是 LOGGING 而直接返回。 */
    spi1_lock();
    if (app_datalogger_state() != DL_ERROR) {
        spi1_unlock();
        return 0;                   /* 已經被別人恢復，或使用者 stop 了 */
    }

    dl_salvage_locked(0);
    if (s_fil_orphan) {
        s_mount_res = FR_NOT_READY; /* 卡還起不來，舊檔留著下一秒再救 */
        rc = -1;
    } else {
        if (!s_mounted) {           /* 舊檔救不回來時 volume 已卸載，重新掛載 */
            s_mount_res = f_mount(&s_fs, "", 1);
            s_mounted   = (s_mount_res == FR_OK);
        }
        if (!s_mounted) {
            rc = -1;
        } else {
            fr = dl_open_new_locked();
            if (fr != FR_OK) {
                rc = -2;
            }
        }
    }

    /* 狀態在 spi1_lock 內決定（鎖序：spi1 在外、state 在內）。這段期間
     * 使用者可能 stop 了（stop 先翻 IDLE 才等 spi1_lock）：那就不能翻回
     * LOGGING，剛開的新檔也要當場關掉 —— 放開鎖再關的話，使用者接著
     * start 開的檔會被誤關。留下一個只有表頭的檔，無害。 */
    int resumed = 0;
    state_lock();
    if (rc == 0 && s_state == DL_ERROR) {
        s_count = 0;
        s_fres  = 0;
        s_state = DL_LOGGING;
        s_recover_count++;
        resumed = 1;
    } else if (rc == -2) {
        s_fres = (uint8_t)fr;
    }
    state_unlock();
    if (rc == 0 && !resumed) {
        f_close(&s_fil);
    }
    spi1_unlock();

    if (resumed && s_producer != NULL) {
        /* DL_ERROR 時 sensor 已經退出取樣迴圈、回頂端睡了（跟 IDLE 一樣），
         * 所以這個通知一定接得到。 */
        xTaskNotifyGive(s_producer);
    }
    return rc;
}

int app_datalogger_stop(void)
{
    dl_state_t st = app_datalogger_state();

    /* DL_ERROR 也要能 stop：舊檔還沒收拾，要先重置卡再 f_close，把已寫入
     * 的部分收尾到 FAT，否則整個 session 的資料都拿不回來（見 dl_salvage_locked）。 */
    if (st != DL_LOGGING && st != DL_ERROR) {
        return 0;
    }

    /* 先翻狀態 → Sensor 立刻停止取樣、Logger 不再寫；再關檔。 */
    state_lock();
    s_state = DL_IDLE;
    state_unlock();

    /* f_sync/f_close 的結果以前被丟掉，於是「收尾失敗」印出來也是 "stopped"。
     * 實測踩到：VDDA 掉到 2.74V 時 f_close 寫不回 FAT，檔案停在 0 B，CLI 卻
     * 回報成功 —— 使用者以為錄到了，實際上整個 session 都沒落地。
     * f_close 失敗時 FIL 的狀態已經不可信，一律把 state 留在 IDLE 不重試。 */
    FRESULT fr;
    spi1_lock();
    /* 看 s_fil_orphan 而不是 st：st 是進鎖前讀的，這中間自動恢復可能已經
     * 開好新檔（那就該正常關新檔）；s_fil_orphan 在 spi1_lock 內才準。 */
    if (s_fil_orphan) {
        /* 不能直接 f_sync：卡多半還卡著，每個 sector 都要走滿 disk_write 的
         * 重試，stop 會停十幾秒。先重置卡；起不來就放棄舊檔。 */
        fr = dl_salvage_locked(1);
    } else {
        fr = f_sync(&s_fil);
        FRESULT frc = f_close(&s_fil);
        if (fr == FR_OK) {
            fr = frc;
        }
        if (st == DL_ERROR && fr == FR_INVALID_OBJECT) {
            fr = FR_OK;             /* 恢復時開新檔失敗：本來就沒有檔開著 */
        }
    }
    spi1_unlock();

    if (fr != FR_OK) {
        state_lock();
        s_fres = (uint8_t)fr;
        s_err_count++;
        state_unlock();
        return -1;
    }
    return 0;
}

/* 把 float 格式成固定小數位字串，例如 fmt_fixed(buf,-1.0012f,3) → "-1.001"。
 * FatFs 的 f_printf 沒開 %f（ffconf FF_PRINT_FLOAT=0），所以自己轉、再用 %s
 * 塞進 CSV，避免拉進整個 float printf。四捨五入；buf 需 >= 3+整數位+decimals。 */
static void fmt_fixed(char *buf, float v, int decimals)
{
    int32_t scale = 1;
    for (int i = 0; i < decimals; i++) {
        scale *= 10;
    }
    int   neg = (v < 0.0f);
    float mag = neg ? -v : v;
    int32_t scaled = (int32_t)(mag * (float)scale + 0.5f);   /* 四捨五入 */
    int32_t whole  = scaled / scale;
    int32_t frac   = scaled % scale;

    int n = 0;
    if (neg && scaled != 0) {                 /* 避免印出 "-0.000" */
        buf[n++] = '-';
    }
    /* 整數位：先反轉再倒回。 */
    char rev[10];
    int  r = 0;
    do { rev[r++] = (char)('0' + whole % 10); whole /= 10; } while (whole && r < (int)sizeof(rev));
    while (r > 0) {
        buf[n++] = rev[--r];
    }
    if (decimals > 0) {
        buf[n++] = '.';
        for (int32_t p = scale / 10; p >= 1; p /= 10) {   /* 補足前導零 */
            buf[n++] = (char)('0' + (frac / p) % 10);
        }
    }
    buf[n] = '\0';
}

int app_datalogger_write_sample(uint32_t t_ms,
                                const mpu6050_axes_t *acc,
                                const mpu6050_axes_t *gyro,
                                float temp_c)
{
    int rc = 0;

    spi1_lock();
    if (s_state == DL_LOGGING) {
        /* raw int16 → 物理單位再寫入：加速度 g（3 位）、角速度 °/s（2 位）、溫度 °C（1 位）。 */
        char ax[12], ay[12], az[12], gx[12], gy[12], gz[12], tc[12];
        fmt_fixed(ax, acc->x  / MPU6050_ACC_LSB_PER_G,    3);
        fmt_fixed(ay, acc->y  / MPU6050_ACC_LSB_PER_G,    3);
        fmt_fixed(az, acc->z  / MPU6050_ACC_LSB_PER_G,    3);
        fmt_fixed(gx, gyro->x / MPU6050_GYRO_LSB_PER_DPS, 2);
        fmt_fixed(gy, gyro->y / MPU6050_GYRO_LSB_PER_DPS, 2);
        fmt_fixed(gz, gyro->z / MPU6050_GYRO_LSB_PER_DPS, 2);
        fmt_fixed(tc, temp_c, 1);
        rc = f_printf(&s_fil, "%u,%s,%s,%s,%s,%s,%s,%s\n",
                      (unsigned)t_ms,
                      ax, ay, az, gx, gy, gz, tc);
        if (rc >= 0) {
            state_lock();
            s_count++;
            state_unlock();
            rc = 0;
        } else {
            /* FatFs 已經把錯誤鎖進 s_fil.err，之後每次 f_write 都會在
             * 開頭直接返回。所以這次 session 到此為止，翻成 DL_ERROR：
             * Sensor 會停止取樣、Logger 不再嘗試寫，使用者看得到。
             * 檔案先不關 —— 標成 s_fil_orphan，留給 recover（Logger task
             * 每秒自動試）或 stop 先重置卡再 f_close，已寫進去的資料保住。
             *
             * s_mounted 也一併清掉：檔案層錯誤常常是卡片層退回 idle state
             * 造成的（供電瞬斷、熱插拔），不能假設卡還在線。recover 會從
             * CMD0 重跑 sd_init()，而不是只清檔案狀態、對已經掉線的卡片
             * 視而不見。 */
            state_lock();
            s_fres = (uint8_t)s_fil.err;
            s_err_count++;
            s_state = DL_ERROR;
            state_unlock();
            s_mounted    = 0;
            s_fil_orphan = 1;       /* 舊檔留給 recover / stop 收拾 */
        }
    }
    spi1_unlock();
    return rc;
}

void app_datalogger_sync(void)
{
    spi1_lock();
    if (s_state == DL_LOGGING) {
        /* f_sync 失敗代表 FAT/目錄項沒寫回去，跟寫樣本失敗一樣嚴重，
         * 同樣要翻狀態，不能再默默丟掉回傳值。同樣清 s_mounted，理由見
         * write_sample 失敗那段。 */
        FRESULT fr = f_sync(&s_fil);
        if (fr != FR_OK) {
            state_lock();
            s_fres = (uint8_t)fr;
            s_err_count++;
            s_state = DL_ERROR;
            state_unlock();
            s_mounted    = 0;
            s_fil_orphan = 1;       /* 舊檔留給 recover / stop 收拾 */
        }
    }
    spi1_unlock();
}

void app_datalogger_get_status(dl_status_t *out)
{
    state_lock();
    out->state = s_state;
    out->count = s_count;
    out->fres      = s_fres;
    out->err_count = s_err_count;
    out->recoveries = s_recover_count;
    for (int i = 0; i < (int)sizeof(s_filename); i++) {
        out->filename[i] = s_filename[i];
    }
    state_unlock();
}

dl_state_t app_datalogger_state(void)
{
    dl_state_t s;
    state_lock();
    s = s_state;
    state_unlock();
    return s;
}

/* 開機掛載重試次數。非 0 代表 SD 卡第一次沒起來 —— 通常是供電邊界，
 * 值得在硬體上處理（獨立 3.3V、加大電容），別等到寫入時才炸。 */
int app_datalogger_mount_retries(void)
{
    return (int)s_mount_retries;
}

/* SD 卡目前有沒有掛上。掛不起來時系統會進降級模式：CLI/OLED 照常，
 * 只是 start 會失敗。 */
int app_datalogger_mounted(void)
{
    return (int)s_mounted;
}

uint32_t app_datalogger_remount_tries(void)
{
    return s_remount_tries;
}

uint32_t app_datalogger_count(void)
{
    uint32_t c;
    state_lock();
    c = s_count;
    state_unlock();
    return c;
}

const char *app_datalogger_filename(void)
{
    return s_filename;
}

/* ───────────────── sensor 喚醒注入 ───────────────── */
void app_datalogger_set_producer(TaskHandle_t producer)
{
    s_producer = producer;
}

/* ───────────────── 檔案層操作（給 CLI 用）─────────────────
 * 全部 FatFs/SD 存取 + spi1_lock 收在這裡，emit callback 在 spi1_lock 內被回呼，
 * callback 內只能碰 UART、不可碰 SD/OLED（同一支 spi1_lock）。 */

int dl_list(void (*emit)(const char *name, uint32_t size),
            uint32_t *free_kb_out, uint8_t *fres_out)
{
    static DIR dir;             /* static：DIR 別放 stack */
    FILINFO    fno;
    int        n = 0;

    if (fres_out) {
        *fres_out = 0;
    }

    spi1_lock();                 /* 整段目錄掃描 + getfree 與寫卡/OLED 互斥 */
    FRESULT fr = f_opendir(&dir, "/");
    if (fr != FR_OK) {
        spi1_unlock();
        if (fres_out) { *fres_out = (uint8_t)fr; }
        return -1;
    }

    /* f_readdir 的回傳值以前被併進 while 條件裡一起吃掉，於是「讀取失敗」
     * 跟「列舉結束」長得一模一樣 —— 迴圈正常離開、n 留在 0，CLI 就印出
     * "(no files)"。實測供電邊界下踩到過：卡上明明有 24 個檔，ls 卻說空的，
     * 過一下再 ls 又全部回來。錯誤必須跟空目錄分開回報。 */
    for (;;) {
        fr = f_readdir(&dir, &fno);
        if (fr != FR_OK) {
            f_closedir(&dir);
            spi1_unlock();
            if (fres_out) { *fres_out = (uint8_t)fr; }
            return -1;
        }
        if (fno.fname[0] == '\0') {
            break;              /* 真的列舉完了 */
        }
        if (fno.fattrib & AM_DIR) {
            continue;           /* 只列檔案 */
        }
        if (emit) {
            emit(fno.fname, (uint32_t)fno.fsize);
        }
        n++;
    }
    f_closedir(&dir);

    if (free_kb_out) {
        /* 剩餘空間：free clusters × 每 cluster 幾個 sector × 512 / 1024 = KB */
        DWORD  fre_clust;
        FATFS *fsp;
        if (f_getfree("", &fre_clust, &fsp) == FR_OK) {
            *free_kb_out = (uint32_t)((uint64_t)fre_clust * fsp->csize / 2);
        } else {
            *free_kb_out = DL_FREE_KB_UNKNOWN;   /* 0 會被讀成「卡滿了」*/
        }
    }
    spi1_unlock();
    return n;
}

int dl_dump(const char *name, void (*emit_line)(const char *line))
{
    static FIL f;               /* static：FIL 很大，別放 stack */
    char       line[64];
    int        rc = 0;

    /* 整段 FatFs 讀取包在 spi1_lock：和 Logger 寫卡、Display 畫 OLED 互斥
     * （dump 期間 OLED 會短暫凍結，可接受）。 */
    spi1_lock();
    if (f_open(&f, name, FA_READ) != FR_OK) {
        rc = -1;
    } else {
        while (f_gets(line, sizeof line, &f) != 0) {
            if (emit_line) {
                emit_line(line);
            }
        }
        f_close(&f);
    }
    spi1_unlock();
    return rc;
}

/* 名字是不是 log_*.csv（不分大小寫）。FF_USE_LFN=0 → f_readdir 回大寫 8.3 名。 */
static int is_log_csv(const char *nm)
{
    static const char pfx[4] = { 'l', 'o', 'g', '_' };
    for (int i = 0; i < 4; i++) {
        char c = nm[i];
        if (c >= 'A' && c <= 'Z') {
            c = (char)(c - 'A' + 'a');
        }
        if (c != pfx[i]) {
            return 0;
        }
    }
    int len = 0;
    while (nm[len]) {
        len++;
    }
    if (len < 5 || nm[len - 4] != '.') {
        return 0;
    }
    char e1 = nm[len - 3], e2 = nm[len - 2], e3 = nm[len - 1];
    if (e1 >= 'A' && e1 <= 'Z') e1 = (char)(e1 + 32);
    if (e2 >= 'A' && e2 <= 'Z') e2 = (char)(e2 + 32);
    if (e3 >= 'A' && e3 <= 'Z') e3 = (char)(e3 + 32);
    return (e1 == 'c' && e2 == 's' && e3 == 'v');
}

int dl_clear(uint32_t *deleted_out, uint8_t *fres_out)
{
    static DIR dir;             /* static：DIR 別放 stack */
    FILINFO    fno;
    char       name[16];
    int        n = 0;

    /* 只掃「真的存在」的檔：每輪重開目錄找第一個 log_*.csv 就刪、再重掃，
     * 直到掃不到為止。空目錄一次掃完即回 —— 不再盲目對 000..999 跑 1000 次
     * f_unlink（那會抱著 spi1_lock 掃目錄上千遍，看起來像當機）。
     * 刪不掉就停手，避免同一檔一直被重掃造成無限迴圈。 */
    FRESULT fr = FR_OK;
    if (fres_out) {
        *fres_out = 0;
    }

    spi1_lock();
    for (;;) {
        fr = f_opendir(&dir, "/");
        if (fr != FR_OK) {
            break;
        }
        int found = 0;
        for (;;) {
            fr = f_readdir(&dir, &fno);
            if (fr != FR_OK || fno.fname[0] == '\0') {
                break;          /* 讀失敗或列舉完；fr 帶著原因往外走 */
            }
            if (fno.fattrib & AM_DIR) {
                continue;
            }
            if (is_log_csv(fno.fname)) {
                int k = 0;
                while (fno.fname[k] && k < (int)sizeof(name) - 1) {
                    name[k] = fno.fname[k];
                    k++;
                }
                name[k] = '\0';
                found = 1;
                break;
            }
        }
        f_closedir(&dir);
        if (fr != FR_OK) {
            break;              /* f_readdir 失敗 */
        }
        if (!found) {
            break;              /* 掃完了，沒有 log_*.csv 了 —— 正常結束 */
        }
        fr = f_unlink(name);
        if (fr != FR_OK) {
            break;
        }
        n++;
    }
    spi1_unlock();

    if (deleted_out) {
        *deleted_out = (uint32_t)n;
    }
    /* 以前不管發生什麼都回 0，於是「卡上只有 3 個檔」和「刪到第 4 個時卡
     * 讀不動了」印出來一模一樣（都是 "deleted 3 files"）。實測在供電邊界
     * 的卡上踩到：23 個檔只刪掉 3 個，CLI 卻回報成功。 */
    if (fr != FR_OK) {
        if (fres_out) { *fres_out = (uint8_t)fr; }
        return -1;
    }
    return 0;
}