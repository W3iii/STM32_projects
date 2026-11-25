/*
 * app_i2c1_supervisor.c
 *
 *  I2C1 匯流排管理員。設計見 app_i2c1_supervisor.h 檔頭。
 *
 *  Created on: Oct 27, 2026
 *      Author: xushu
 */

#include "app_i2c1_supervisor.h"
#include "bus_port.h"        /* i2c1_lock / i2c1_unlock */
#include "driver_i2c1.h"     /* i2c_status_t / i2c1_bus_recovery */
#include "systick.h"

/* 晶片連續幾次 NACK 才判離線。MPU 100Hz 下 = 50ms，INA 10Hz 下 = 0.5s
 * （INA 每次讀取在 Display 端已經重試過一次）。偶發一次 NACK 不該讓 OLED
 * 閃 NO MPU。 */
#define I2C1_SUP_NACK_LIMIT         5

/* 匯流排連續幾次錯誤（TIMEOUT/BUS/SCL，不分哪顆晶片）才 bus recovery。
 * 一次成功的交易就證明匯流排是好的，計數歸零。 */
#define I2C1_SUP_BUS_ERR_LIMIT      5

/* 兩次 bus recovery 至少隔多久。recovery 救不回來（SCL 被拉住、slave 真的
 * 卡死）時錯誤會一直來，不限速就變成每幾筆交易搖一次匯流排。 */
#define I2C1_SUP_RECOVERY_MIN_MS    1000

typedef struct {
    const char       *name;
    int             (*init)(void);
    uint32_t          probe_ms;
    volatile uint8_t  online;
    uint8_t           nack_streak;
    uint32_t          last_contact_ms;  /* 最後一次成功交易或探測的時間 */
    volatile uint32_t went_offline;
} dev_state_t;

static dev_state_t s_dev[I2C1_SUP_MAX_DEVS];
static int         s_dev_count;

static uint8_t           s_bus_streak;
static uint32_t          s_bus_last_recovery_ms;
static volatile uint32_t s_bus_recoveries;
static volatile uint8_t  s_bus_last_rc = I2C_OK;

static int valid(i2c1_dev_t dev)
{
    return dev >= 0 && dev < s_dev_count;
}

static int is_bus_error(int st)
{
    return st == I2C_ERR_TIMEOUT || st == I2C_ERR_BUS || st == I2C_ERR_SCL_STUCK;
}

static void set_online(dev_state_t *d, int online)
{
    if (d->online && !online) {
        d->went_offline++;
    }
    d->online      = (uint8_t)(online ? 1 : 0);
    d->nack_streak = 0;
}

/* 匯流排層：記一次錯誤，夠多次就 recovery。持有 i2c1_lock。 */
static void bus_error(void)
{
    if (++s_bus_streak < I2C1_SUP_BUS_ERR_LIMIT) {
        return;
    }
    uint32_t now = systick_get_ms();
    if (s_bus_recoveries != 0 && (now - s_bus_last_recovery_ms) < I2C1_SUP_RECOVERY_MIN_MS) {
        return;                     /* 限速：計數留著，時間到了下一個錯誤就會觸發 */
    }
    s_bus_streak           = 0;
    s_bus_last_recovery_ms = now;
    s_bus_recoveries++;
    s_bus_last_rc = (uint8_t)i2c1_bus_recovery();

    /* 每顆晶片都重新 init，不只觸發的那顆：
     * 匯流排卡住常是熱插拔途中抽走 slave、或接點彈跳讓它停在半途 —— 插回來的
     * 晶片是 power-on 預設值。recovery 只讓它放開 SDA，不會幫它重新設定。
     * init 順便當在線偵測：recovery 沒救回來（SDA/SCL 還卡著）時 init 全部失敗
     * → 全部離線，讀取端停止讀取，之後只剩每顆的重探在碰匯流排。
     * 這裡不把 init 的錯誤再回報給 bus_error()，免得在 recovery 裡遞迴。 */
    for (int i = 0; i < s_dev_count; i++) {
        dev_state_t *d = &s_dev[i];
        set_online(d, d->init() == I2C_OK);
        d->last_contact_ms = now;
    }
}

i2c1_dev_t i2c1_sup_register(const char *name, int (*init)(void), uint32_t probe_ms)
{
    if (s_dev_count >= I2C1_SUP_MAX_DEVS) {
        return -1;
    }
    dev_state_t *d = &s_dev[s_dev_count];
    d->name     = name;
    d->init     = init;
    d->probe_ms = probe_ms;
    d->online   = 0;
    return s_dev_count++;
}

void i2c1_sup_report(i2c1_dev_t dev, int st)
{
    if (!valid(dev)) {
        return;
    }
    dev_state_t *d = &s_dev[dev];

    if (st == I2C_OK) {
        d->nack_streak     = 0;
        d->last_contact_ms = systick_get_ms();
        s_bus_streak       = 0;     /* 有交易成功 = 匯流排是好的 */
        return;
    }
    if (is_bus_error(st)) {
        bus_error();                /* 匯流排的問題，不算在這顆晶片頭上 */
        return;
    }
    /* NACK（或 PARAM 這類晶片層的錯）：只影響這顆，不動匯流排 */
    if (++d->nack_streak >= I2C1_SUP_NACK_LIMIT) {
        set_online(d, 0);
    }
}

int i2c1_sup_probe(i2c1_dev_t dev)
{
    if (!valid(dev)) {
        return 0;
    }
    dev_state_t *d = &s_dev[dev];

    /* 失敗重試一次再下結論：偶發一次 NACK 就把在線的晶片判成離線，
     * OLED 會閃一下 NO MPU。 */
    int rc = d->init();
    if (rc != I2C_OK) {
        rc = d->init();
    }
    d->last_contact_ms = systick_get_ms();

    if (rc == I2C_OK) {
        s_bus_streak = 0;
        set_online(d, 1);
    } else {
        if (is_bus_error(rc)) {
            bus_error();            /* 探測撞到的匯流排錯誤一樣算進匯流排層 */
        }
        set_online(d, 0);
    }
    return d->online;
}

void i2c1_sup_tick(void)
{
    uint32_t now = systick_get_ms();
    for (int i = 0; i < s_dev_count; i++) {
        if ((now - s_dev[i].last_contact_ms) >= s_dev[i].probe_ms) {
            i2c1_lock();
            i2c1_sup_probe(i);
            i2c1_unlock();
        }
    }
}

int i2c1_sup_online(i2c1_dev_t dev)
{
    return valid(dev) ? s_dev[dev].online : 0;
}

int i2c1_sup_dev_count(void)
{
    return s_dev_count;
}

void i2c1_sup_bus_stats(i2c1_sup_bus_stats_t *out)
{
    out->recoveries = s_bus_recoveries;
    out->last_rc    = s_bus_last_rc;
}

void i2c1_sup_dev_stats(i2c1_dev_t dev, i2c1_sup_dev_stats_t *out)
{
    if (!valid(dev)) {
        out->name         = "?";
        out->online       = 0;
        out->probe_ms     = 0;
        out->went_offline = 0;
        return;
    }
    const dev_state_t *d = &s_dev[dev];
    out->name         = d->name;
    out->online       = d->online;
    out->probe_ms     = d->probe_ms;
    out->went_offline = d->went_offline;
}
