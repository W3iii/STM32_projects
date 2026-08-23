/*
 * driver_i2c1.c
 *
 * I2C1 driver implementation
 *
 *  Created on: May 18, 2026
 *      Author: xushu
 */

#include "driver_i2c1.h"
#include "stm32f407xx.h"
#include "systick.h"
#include "driver_rcc.h"     /* rcc_pclk1_hz()：I2C1 在 APB1 */
#include "bus_port.h"       /* 非同步交易的等待/喚醒（RTOS 中立）*/
#include "isr_port.h"       /* ISR 計時掛勾；沒有 isr_port_config.h 時展開成空 */

/* ========================================================================= */
/*                       Register bit definitions                            */
/* (stm32f407xx.h 沒提供 I2C bit macro，這裡自己定義，避免裸 (1<<n))         */
/* ========================================================================= */

/* CR1 */
#define I2C_CR1_PE          (1U << 0)    /* Peripheral enable */
#define I2C_CR1_START       (1U << 8)    /* Start generation */
#define I2C_CR1_STOP        (1U << 9)    /* Stop generation */
#define I2C_CR1_ACK         (1U << 10)   /* ACK enable */
#define I2C_CR1_POS         (1U << 11)   /* ACK position (for 2-byte read) */
#define I2C_CR1_SWRST       (1U << 15)   /* Software reset */

/* SR1 */
#define I2C_SR1_SB          (1U << 0)    /* Start bit sent */
#define I2C_SR1_ADDR        (1U << 1)    /* Address sent / matched */
#define I2C_SR1_BTF         (1U << 2)    /* Byte transfer finished */
#define I2C_SR1_RXNE        (1U << 6)    /* RX data register not empty */
#define I2C_SR1_TXE         (1U << 7)    /* TX data register empty */
#define I2C_SR1_AF          (1U << 10)   /* Acknowledge failure (= NACK) */
#define I2C_SR1_ARLO        (1U << 9)    /* Arbitration lost */
#define I2C_SR1_BERR        (1U << 8)    /* Bus error */

/* SR2 */
#define I2C_SR2_BUSY        (1U << 1)    /* Bus busy */

/* CCR：時序設定 */
#define I2C_CCR_FS          (1U << 15)   /* 0 = standard mode, 1 = fast mode */
#define I2C_CCR_DUTY        (1U << 14)   /* fast mode 的 Tlow/Thigh 比例 */

/* CR2：中斷致能（中斷驅動的非同步讀取用，polling 路徑不會開這些）*/
#define I2C_CR2_ITERREN     (1U << 8)    /* 錯誤中斷（BERR/ARLO/AF/OVR）*/
#define I2C_CR2_ITEVTEN     (1U << 9)    /* 事件中斷（SB/ADDR/BTF/STOPF）*/
#define I2C_CR2_ITBUFEN     (1U << 10)   /* 緩衝區中斷（TXE/RXNE），與 ITEVTEN 併用 */

/* ========================================================================= */
/*                          Configuration                                    */
/* ========================================================================= */

/* I2C1 在 APB1 → fCK = rcc_pclk1_hz()（目前 42MHz）。CR2 FREQ / CCR / TRISE
 * 都由它即時推導（見 i2c1_init），改 PLL 設定不用再動這裡。 */

/* 每個 flag 等待的 timeout（ms）
 * 400kHz 下讀 14 byte 約 0.4ms，100ms 綽綽有餘，
 * 又不會在真的斷線時讓使用者等太久。
 */
#define I2C_TIMEOUT_MS      100U


/* ========================================================================= */
/*                          GPIO setup (shared)                              */
/* ========================================================================= */

/*
 * 設定 PB8/PB9 為 I2C1 AF mode（open-drain + pull-up）
 * init 和 bus_recovery 結束後都會呼叫，所以抽成 static helper（決策 2）
 */
static void i2c1_gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* PB8 = I2C1_SCL, PB9 = I2C1_SDA → AF mode
     * （SCL 從 PB6 改到 PB8，把 PB6 讓給 USART1_TX）*/
    GPIOB->MODER &= ~(0x3U << (8 * 2));
    GPIOB->MODER |=  (GPIO_MODER_AF << (8 * 2));
    GPIOB->MODER &= ~(0x3U << (9 * 2));
    GPIOB->MODER |=  (GPIO_MODER_AF << (9 * 2));

    /* PB8/PB9 都在 AFRH（pin 8~15）*/
    GPIOB->AFRH &= ~(0xFU << ((8 - 8) * 4));
    GPIOB->AFRH |=  (GPIO_AF4_I2C1 << ((8 - 8) * 4));
    GPIOB->AFRH &= ~(0xFU << ((9 - 8) * 4));
    GPIOB->AFRH |=  (GPIO_AF4_I2C1 << ((9 - 8) * 4));

    /* I2C 必須 open-drain + pull-up */
    GPIOB->OTYPER |= (1U << 8);
    GPIOB->OTYPER |= (1U << 9);
    GPIOB->PUPDR &= ~(0x3U << (8 * 2));
    GPIOB->PUPDR |=  (0x1U << (8 * 2));
    GPIOB->PUPDR &= ~(0x3U << (9 * 2));
    GPIOB->PUPDR |=  (0x1U << (9 * 2));
}


/* ========================================================================= */
/*                          Wait helpers                                     */
/* ========================================================================= */

/*
 * 等某個 SR1 flag 變 1，有 timeout。
 * 用於 SB / TXE / BTF / RXNE 這些「正常情況一定會來」的 flag。
 *
 * 回傳：
 *   I2C_OK          flag 來了
 *   I2C_ERR_TIMEOUT 超時（bus stuck）
 */
static i2c_status_t wait_flag(uint32_t flag)
{
    uint32_t start = systick_get_ms();
    while (!(I2C1->SR1 & flag)) {
        if ((systick_get_ms() - start) >= I2C_TIMEOUT_MS) {
            return I2C_ERR_TIMEOUT;
        }
    }
    return I2C_OK;
}

/*
 * 專門等 ADDR flag，因為送完位址後有兩種結果：
 *   - slave 存在 → 回 ACK → ADDR flag 置位
 *   - slave 不存在 → 回 NACK → AF flag 置位
 * 所以這裡要同時監看 ADDR 和 AF。
 *
 * 回傳：
 *   I2C_OK          ADDR 來了（slave 有回應）
 *   I2C_ERR_NACK    AF 置位（slave 沒回應），已清 AF + 送 STOP
 *   I2C_ERR_TIMEOUT 兩個都沒來
 */
static i2c_status_t wait_addr(void)
{
    uint32_t start = systick_get_ms();
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;       /* 清 AF flag */
            I2C1->CR1 |= I2C_CR1_STOP;      /* 收尾，釋放 bus */
            return I2C_ERR_NACK;
        }
        if ((systick_get_ms() - start) >= I2C_TIMEOUT_MS) {
            return I2C_ERR_TIMEOUT;
        }
    }
    return I2C_OK;
}


/* ========================================================================= */
/*                          Initialization                                   */
/* ========================================================================= */

void i2c1_init(void)
{
    i2c1_gpio_init();

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* 先 disable，才能改設定 */
    I2C1->CR1 &= ~I2C_CR1_PE;

    /* 軟體 reset 一下，清掉可能殘留的狀態（保險） */
    I2C1->CR1 |=  I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    uint32_t pclk1     = rcc_pclk1_hz();           /* APB1 實際頻率（Hz）*/
    uint32_t pclk1_mhz = pclk1 / 1000000U;         /* = 42 */

    /* CR2 FREQ：告訴硬體 APB1 頻率（MHz），有效範圍 2~50 */
    I2C1->CR2 = pclk1_mhz;

    /* CCR：fast mode 400kHz，DUTY = 0（Tlow/Thigh = 2）
     *   CCR = APB1 / (3 × I2C_speed) = 42M / (3 × 400k) = 35
     *
     * 為什麼從 100kHz 改成 400kHz：Sensor task 每 10ms 讀一次 MPU6050 的
     * 14-byte burst，100kHz 下這筆交易要 ~1.5ms，而 driver 是 busy-wait，
     * 那 1.5ms 全部算成 CPU 時間 —— 實測 sensor 佔 16.2%（`tasks` 指令）。
     * 400kHz 把傳輸時間壓到 ~0.4ms，同一筆交易的 CPU 成本降到約四分之一。
     *
     * MPU6050 與 INA219 都支援 fast mode（規格書皆標示 400kHz）。若換到
     * 拉電阻較弱或線較長的板子出現 NACK / bus stuck，把下面兩行改回
     * standard mode 即可（CCR = pclk1 / (2 × 100000)、TRISE = pclk1_mhz + 1）。 */
    I2C1->CCR = I2C_CCR_FS | (pclk1 / (3U * 400000U));

    /* TRISE：上升時間上限 / TPCLK1 + 1。
     * standard mode 上限 1000ns → pclk1_mhz + 1
     * fast mode     上限  300ns → pclk1_mhz × 300 / 1000 + 1 = 13 */
    I2C1->TRISE = (pclk1_mhz * 300U) / 1000U + 1U;

    /* 啟用 */
    I2C1->CR1 |= I2C_CR1_PE;

    /* NVIC：事件與錯誤兩支 IRQ，給中斷驅動的 i2c1_read_regs_async() 用。
     * 這裡只是「開起來」，CR2 的 ITEVTEN/ITBUFEN/ITERREN 預設是關的，所以
     * polling 路徑完全不受影響 —— 中斷只在 async 交易期間才會被打開。
     *
     * 優先權必須 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(=5)，因為
     * ISR 會透過 i2c1_xfer_signal() 呼叫 ...FromISR()。設 6，與 EXTI0 /
     * USART1 / DMA2_Stream2 一致。 */
    NVIC_SetPriority(IRQ_I2C1_EV, 6);
    NVIC_SetPriority(IRQ_I2C1_ER, 6);
    NVIC_EnableIRQ(IRQ_I2C1_EV);
    NVIC_EnableIRQ(IRQ_I2C1_ER);
}


/* ========================================================================= */
/*                          Write register                                   */
/* ========================================================================= */

i2c_status_t i2c1_write_reg(uint8_t slave_addr, uint8_t reg_addr, uint8_t data)
{
    i2c_status_t st;

    /* 1. START */
    I2C1->CR1 |= I2C_CR1_START;
    st = wait_flag(I2C_SR1_SB);
    if (st != I2C_OK) return st;

    /* 2. 送 slave + W（寫 DR 同時清 SB） */
    I2C1->DR = (slave_addr << 1) | 0;
    st = wait_addr();                       /* 同時檢查 NACK */
    if (st != I2C_OK) return st;

    /* 3. 清 ADDR（讀 SR1 + SR2） */
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    /* 4. 送 reg_addr */
    st = wait_flag(I2C_SR1_TXE);
    if (st != I2C_OK) return st;
    I2C1->DR = reg_addr;

    /* 5. 送 data */
    st = wait_flag(I2C_SR1_TXE);
    if (st != I2C_OK) return st;
    I2C1->DR = data;

    /* 6. 等傳輸完成 */
    st = wait_flag(I2C_SR1_BTF);
    if (st != I2C_OK) return st;

    /* 7. STOP */
    I2C1->CR1 |= I2C_CR1_STOP;

    return I2C_OK;
}


i2c_status_t i2c1_write_reg16(uint8_t slave_addr, uint8_t reg_addr, uint16_t data)
{
    i2c_status_t st;

    /* 1. START */
    I2C1->CR1 |= I2C_CR1_START;
    st = wait_flag(I2C_SR1_SB);
    if (st != I2C_OK) return st;

    /* 2. 送 slave + W */
    I2C1->DR = (slave_addr << 1) | 0;
    st = wait_addr();
    if (st != I2C_OK) return st;

    /* 3. 清 ADDR */
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    /* 4. 送 reg_addr */
    st = wait_flag(I2C_SR1_TXE);
    if (st != I2C_OK) return st;
    I2C1->DR = reg_addr;

    /* 5. 送 data 高位元組 → 低位元組（I2C 慣例：MSB first）*/
    st = wait_flag(I2C_SR1_TXE);
    if (st != I2C_OK) return st;
    I2C1->DR = (uint8_t)(data >> 8);

    st = wait_flag(I2C_SR1_TXE);
    if (st != I2C_OK) return st;
    I2C1->DR = (uint8_t)(data & 0xFF);

    /* 6. 等傳輸完成 */
    st = wait_flag(I2C_SR1_BTF);
    if (st != I2C_OK) return st;

    /* 7. STOP */
    I2C1->CR1 |= I2C_CR1_STOP;

    return I2C_OK;
}


/* ========================================================================= */
/*                          Read registers                                   */
/* ========================================================================= */

i2c_status_t i2c1_read_regs(uint8_t slave, uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2c_status_t st;

    if (buf == 0 || len == 0) {
        return I2C_ERR_PARAM;
    }

    /* ===== 前半段：寫 register address ===== */
    I2C1->CR1 |= I2C_CR1_START;
    st = wait_flag(I2C_SR1_SB);
    if (st != I2C_OK) return st;

    I2C1->DR = (slave << 1) | 0;            /* addr + W */
    st = wait_addr();
    if (st != I2C_OK) return st;
    (void)I2C1->SR1; (void)I2C1->SR2;       /* 清 ADDR */

    st = wait_flag(I2C_SR1_TXE);
    if (st != I2C_OK) return st;
    I2C1->DR = reg;

    st = wait_flag(I2C_SR1_BTF);            /* 等 reg 真的送出去 */
    if (st != I2C_OK) return st;

    /* ===== RESTART → 切讀 ===== */
    I2C1->CR1 |= I2C_CR1_START;
    st = wait_flag(I2C_SR1_SB);
    if (st != I2C_OK) return st;

    I2C1->DR = (slave << 1) | 1;            /* addr + R */

    /* ===== 依長度分三種 case ===== */
    if (len == 1) {
        /* ---- Case 1：讀單 byte ----
         * 清 ADDR 之前要先 ACK=0，這樣收完唯一一個 byte 後回 NACK
         */
        I2C1->CR1 &= ~I2C_CR1_ACK;

        st = wait_addr();
        if (st != I2C_OK) return st;
        (void)I2C1->SR1; (void)I2C1->SR2;

        I2C1->CR1 |= I2C_CR1_STOP;

        st = wait_flag(I2C_SR1_RXNE);
        if (st != I2C_OK) return st;
        buf[0] = (uint8_t)I2C1->DR;

    } else if (len == 2) {
        /* ---- Case 2：讀 2 byte（要設 POS）---- */
        I2C1->CR1 |= I2C_CR1_ACK;
        I2C1->CR1 |= I2C_CR1_POS;

        st = wait_addr();
        if (st != I2C_OK) return st;
        (void)I2C1->SR1; (void)I2C1->SR2;

        I2C1->CR1 &= ~I2C_CR1_ACK;          /* 給第 2 byte 用 */

        st = wait_flag(I2C_SR1_BTF);        /* 兩 byte 都進 shift+data reg */
        if (st != I2C_OK) return st;
        I2C1->CR1 |= I2C_CR1_STOP;

        buf[0] = (uint8_t)I2C1->DR;
        buf[1] = (uint8_t)I2C1->DR;

        I2C1->CR1 &= ~I2C_CR1_POS;          /* 恢復正常 */

    } else {
        /* ---- Case 3：讀 N >= 3 byte ---- */
        I2C1->CR1 |= I2C_CR1_ACK;

        st = wait_addr();
        if (st != I2C_OK) return st;
        (void)I2C1->SR1; (void)I2C1->SR2;

        /* 讀前 N-3 byte，每次 ACK */
        for (int i = 0; i < len - 3; i++) {
            st = wait_flag(I2C_SR1_RXNE);
            if (st != I2C_OK) return st;
            buf[i] = (uint8_t)I2C1->DR;
        }

        /* 倒數第 3 byte：等 BTF，然後 ACK=0 */
        st = wait_flag(I2C_SR1_BTF);
        if (st != I2C_OK) return st;
        I2C1->CR1 &= ~I2C_CR1_ACK;

        buf[len - 3] = (uint8_t)I2C1->DR;   /* 讀倒數第 3 */

        st = wait_flag(I2C_SR1_BTF);
        if (st != I2C_OK) return st;
        I2C1->CR1 |= I2C_CR1_STOP;

        buf[len - 2] = (uint8_t)I2C1->DR;   /* 倒數第 2 */
        buf[len - 1] = (uint8_t)I2C1->DR;   /* 最後一 byte */
    }

    /* 收尾：把 ACK 設回 1，給下次 transaction 用 */
    I2C1->CR1 |= I2C_CR1_ACK;

    return I2C_OK;
}


/* ========================================================================= */
/*                  Interrupt-driven read (async)                            */
/* ========================================================================= */

/*
 * i2c1_read_regs_async() —— 與上面 i2c1_read_regs() 的匯流排時序完全相同，
 * 差別只在「等 flag」的方式：polling 版在 while 迴圈裡空轉，這裡把同樣的
 * 狀態轉移搬進 ISR，發起後 task 睡覺，整筆交易結束才被喚醒一次。
 *
 * 為什麼兩份都留：
 *   polling  給不在熱路徑、或不能依賴 RTOS 的呼叫端 —— i2c1_bus_scan
 *            （CLI 的 i2cscan）、i2c1_bus_recovery、寫暫存器，以及
 *            bare-metal 的 exp13。行為與時序一個字都沒改。
 *   async    給 Sensor task 的 100Hz 熱路徑（mpu6050 的 14-byte burst）。
 *
 * 狀態機（箭頭上是觸發的中斷事件）：
 *
 *   START ─SB─> addr+W ─ADDR─> 清ADDR+寫reg ─BTF─> RESTART
 *         ─SB─> addr+R ─ADDR─> 清ADDR ─RXNE/BTF─> 收 len byte ─> STOP
 *
 * 收尾依長度分三種，與 polling 版一致（RM0090 §27.3.3 Master receiver）：
 *   len == 1   清 ADDR 前先 ACK=0，清完立刻 STOP，等一次 RXNE
 *   len == 2   清 ADDR 前 POS=1/ACK=1，清完 ACK=0，等 BTF 一次讀兩 byte
 *   len >= 3   前面由 RXNE 驅動；剩 3 byte 時關掉 ITBUFEN 交棒給 BTF
 *              （BTF: ACK=0 → 讀一筆 → BTF: STOP → 讀兩筆）
 *
 * ── 兩個 ISR livelock 陷阱，設計上繞掉了 ──
 * 中斷旗標沒被清掉就離開 ISR，NVIC 會立刻再進來，變成空轉重入：
 *
 *   1. TXE：寫完 addr+W 後 DR 一空 TXE 就長期為真。所以這裡「不」用 TXE
 *      當狀態 —— 清完 ADDR 時 DR 本來就是空的，直接寫 reg，省掉一個狀態，
 *      也就不需要在寫入階段開 ITBUFEN。
 *   2. RXNE：len<=3 或剩 3 byte 時關掉 ITBUFEN，只讓 BTF 驅動尾巴，
 *      避免「RXNE 為真但這輪不該讀」的空轉。
 *
 * 唯一剩下的短暫重入是 AST_REG_BTF 送出 RESTART 後、硬體真的產生 START
 * 之前的那 ~2.5µs（BTF 由 START/STOP 硬體清除）。自限、且只有幾次。
 */

typedef enum {
    AST_IDLE = 0,
    AST_SB_W,       /* 等第一次 START 的 SB */
    AST_ADDR_W,     /* 等 addr+W 的 ADDR */
    AST_REG_BTF,    /* reg 已寫入 DR，等 BTF（真的送出去才能 RESTART）*/
    AST_SB_R,       /* 等 RESTART 的 SB */
    AST_ADDR_R,     /* 等 addr+R 的 ADDR */
    AST_RX,         /* 收資料中 */
} i2c_astate_t;

static struct {
    volatile i2c_astate_t state;
    volatile i2c_status_t status;
    volatile uint8_t      done;   /* 0 = 進行中；bare-metal 的 wait 盯這個 */
    volatile uint8_t      idx;
    uint8_t               slave;
    uint8_t               reg;
    uint8_t              *buf;
    uint8_t               len;
} s_a;

/* 收尾：關中斷、把 ACK/POS 復原成下一筆交易的預設值，喚醒等待者。 */
static void i2c_async_finish(i2c_status_t st)
{
    I2C1->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
    I2C1->CR1 &= ~I2C_CR1_POS;
    I2C1->CR1 |=  I2C_CR1_ACK;

    s_a.state  = AST_IDLE;
    s_a.status = st;
    s_a.done   = 1;
    i2c1_xfer_signal();
}

int i2c1_async_busy(void)
{
    return (s_a.done == 0);
}

void I2C1_EV_IRQHandler(void)
{
    ISR_PORT_ENTER();
    {
        uint32_t sr1 = I2C1->SR1;

        if (sr1 & I2C_SR1_SB) {
            /* SB 由「讀 SR1（上面已讀）+ 寫 DR」清除 */
            if (s_a.state == AST_SB_W) {
                I2C1->DR  = (uint8_t)((s_a.slave << 1) | 0U);
                s_a.state = AST_ADDR_W;
            } else {
                I2C1->DR = (uint8_t)((s_a.slave << 1) | 1U);
                /* ACK / POS 必須在清 ADDR 之前就位 */
                if (s_a.len == 1U) {
                    I2C1->CR1 &= ~I2C_CR1_ACK;
                } else if (s_a.len == 2U) {
                    I2C1->CR1 |= I2C_CR1_ACK;
                    I2C1->CR1 |= I2C_CR1_POS;
                } else {
                    I2C1->CR1 |= I2C_CR1_ACK;
                }
                s_a.state = AST_ADDR_R;
            }
        }
        else if (sr1 & I2C_SR1_ADDR) {
            if (s_a.state == AST_ADDR_W) {
                (void)I2C1->SR1; (void)I2C1->SR2;   /* 清 ADDR */
                /* 清完 ADDR 後 DR 是空的，直接寫 reg，不必等 TXE（見上方陷阱 1）*/
                I2C1->DR  = s_a.reg;
                s_a.state = AST_REG_BTF;
            } else {
                (void)I2C1->SR1; (void)I2C1->SR2;   /* 清 ADDR */

                if (s_a.len == 1U) {
                    I2C1->CR1 |= I2C_CR1_STOP;
                    I2C1->CR2 |= I2C_CR2_ITBUFEN;   /* 靠那唯一一次 RXNE */
                } else if (s_a.len == 2U) {
                    I2C1->CR1 &= ~I2C_CR1_ACK;      /* 給第 2 byte 用 */
                } else if (s_a.len > 3U) {
                    I2C1->CR2 |= I2C_CR2_ITBUFEN;   /* 前段由 RXNE 驅動 */
                }
                /* len == 3：ITBUFEN 保持關閉，三筆全由 BTF 驅動 */
                s_a.state = AST_RX;
            }
        }
        else if (sr1 & I2C_SR1_BTF) {
            if (s_a.state == AST_REG_BTF) {
                I2C1->CR1 |= I2C_CR1_START;         /* RESTART，切成讀 */
                s_a.state  = AST_SB_R;
            } else if (s_a.state == AST_RX) {
                uint8_t remain = (uint8_t)(s_a.len - s_a.idx);

                if (remain == 2U) {
                    /* len==2 的 POS 路徑與 len>=3 的尾巴在這裡會合 */
                    I2C1->CR1 |= I2C_CR1_STOP;
                    s_a.buf[s_a.idx++] = (uint8_t)I2C1->DR;
                    s_a.buf[s_a.idx++] = (uint8_t)I2C1->DR;
                    i2c_async_finish(I2C_OK);
                } else if (remain == 3U) {
                    I2C1->CR1 &= ~I2C_CR1_ACK;      /* 最後兩筆要 NACK */
                    s_a.buf[s_a.idx++] = (uint8_t)I2C1->DR;
                } else {
                    /* 不該發生；讀掉 DR 免得 BTF 一直為真卡住 ISR */
                    if (s_a.idx < s_a.len) {
                        s_a.buf[s_a.idx++] = (uint8_t)I2C1->DR;
                    } else {
                        (void)I2C1->DR;
                    }
                }
            }
        }
        else if ((sr1 & I2C_SR1_RXNE) && (s_a.state == AST_RX)) {
            if (s_a.len == 1U) {
                s_a.buf[0] = (uint8_t)I2C1->DR;
                s_a.idx    = 1U;
                i2c_async_finish(I2C_OK);
            } else if ((uint8_t)(s_a.len - s_a.idx) > 3U) {
                s_a.buf[s_a.idx++] = (uint8_t)I2C1->DR;
                if ((uint8_t)(s_a.len - s_a.idx) == 3U) {
                    I2C1->CR2 &= ~I2C_CR2_ITBUFEN;  /* 剩三筆，交棒給 BTF */
                }
            }
            /* 其餘情況不在這裡讀，交給 BTF 分支（見上方陷阱 2）*/
        }
    }
    ISR_PORT_EXIT();
}

void I2C1_ER_IRQHandler(void)
{
    ISR_PORT_ENTER();
    {
        uint32_t     sr1 = I2C1->SR1;
        i2c_status_t st  = I2C_ERR_BUS;

        if (sr1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;       /* slave 沒回應 */
            st = I2C_ERR_NACK;
        }
        if (sr1 & I2C_SR1_BERR) { I2C1->SR1 &= ~I2C_SR1_BERR; }
        if (sr1 & I2C_SR1_ARLO) { I2C1->SR1 &= ~I2C_SR1_ARLO; }

        I2C1->CR1 |= I2C_CR1_STOP;          /* 收尾，釋放 bus */
        i2c_async_finish(st);
    }
    ISR_PORT_EXIT();
}

i2c_status_t i2c1_read_regs_async(uint8_t slave, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (buf == 0 || len == 0) {
        return I2C_ERR_PARAM;
    }

    /* 呼叫端必須已持有 i2c1_lock()，所以同一時間只會有一筆 async 交易，
     * s_a 這組狀態不需要額外保護。 */
    i2c1_xfer_reset();          /* 清掉上一筆逾時後才姍姍來遲的完成通知 */

    s_a.slave  = slave;
    s_a.reg    = reg;
    s_a.buf    = buf;
    s_a.len    = len;
    s_a.idx    = 0;
    s_a.status = I2C_ERR_TIMEOUT;
    s_a.done   = 0;
    s_a.state  = AST_SB_W;

    I2C1->CR1 |= I2C_CR1_ACK;
    /* 寫入階段不開 ITBUFEN（見上方陷阱 1），讀取階段才依長度打開 */
    I2C1->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
    I2C1->CR1 |= I2C_CR1_START;

    if (i2c1_xfer_wait() != 0) {
        /* 狀態機沒把交易推完（bus stuck、中斷沒來）。把周邊收乾淨再回報，
         * 否則下一筆交易會踩在半途的狀態上。 */
        I2C1->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
        I2C1->CR1 |=  I2C_CR1_STOP;
        I2C1->CR1 &= ~I2C_CR1_POS;
        I2C1->CR1 |=  I2C_CR1_ACK;
        s_a.state  = AST_IDLE;
        s_a.done   = 1;
        return I2C_ERR_TIMEOUT;
    }

    return s_a.status;
}


/* ========================================================================= */
/*                          Bus recovery                                     */
/* ========================================================================= */

/*
 * 簡單的 µs 級延遲（bus recovery 的 clock pulse 用）
 * 100kHz I2C → 半週期 5µs，這裡用粗略 busy loop 即可。
 * 迴圈次數隨 HCLK 縮放：16MHz 時 80 次 ≈ 5µs，168MHz（×10.5）→ 約 840 次。
 */
static void short_delay(void)
{
    for (volatile int i = 0; i < 840; i++) { __asm volatile("nop"); }
}

i2c_status_t i2c1_bus_recovery(void)
{
    /* 1. disable I2C peripheral，準備手動操作腳位 */
    I2C1->CR1 &= ~I2C_CR1_PE;

    /* 2. PB8(SCL)、PB9(SDA) 切成 GPIO output（open-drain 保留）*/
    GPIOB->MODER &= ~(0x3U << (8 * 2));
    GPIOB->MODER |=  (GPIO_MODER_OUTPUT << (8 * 2));
    GPIOB->MODER &= ~(0x3U << (9 * 2));
    GPIOB->MODER |=  (GPIO_MODER_OUTPUT << (9 * 2));
    /* OTYPER 在 gpio_init 已設 open-drain，SDA 拉高靠 pull-up */
    GPIOB->ODR |= (1U << 9);    /* SDA 放開（high）*/
    GPIOB->ODR |= (1U << 8);    /* SCL high */

    /* 3. 送最多 9 個 clock pulse，期間檢查 SDA 是否被釋放 */
    i2c_status_t result = I2C_ERR_BUS;
    for (int i = 0; i < 9; i++) {
        /* 如果 SDA 已經回到 high，slave 釋放了 → 成功 */
        if (GPIOB->IDR & (1U << 9)) {
            result = I2C_OK;
            break;
        }
        GPIOB->ODR &= ~(1U << 8);   /* SCL low */
        short_delay();
        GPIOB->ODR |=  (1U << 8);   /* SCL high */
        short_delay();
    }

    /* 4. 產生一次 STOP condition（SDA 由 low → high 當 SCL high）*/
    GPIOB->ODR &= ~(1U << 9);   /* SDA low */
    short_delay();
    GPIOB->ODR |=  (1U << 8);   /* SCL high */
    short_delay();
    GPIOB->ODR |=  (1U << 9);   /* SDA high → STOP */
    short_delay();

    /* 5. 重新 init I2C（gpio_init 會把腳位切回 AF mode）*/
    i2c1_init();

    return result;
}

/* ========================================================================= */
/*                          Bus scanner                                      */
/* ========================================================================= */

/*
 * 探測單一位址：送 START + addr+W，看回 ACK 還是 NACK。
 * 回傳 1 = 有裝置（ACK），0 = 沒有（NACK 或 timeout）。
 *
 * 注意：這裡不能直接用 wait_addr()，因為 wait_addr 在 NACK 時
 *       會自動送 STOP。掃描時我們也要送 STOP，邏輯剛好可重用，
 *       但 ACK 的情況要自己補 STOP。
 */
static int i2c1_bus_probe(uint8_t addr)
{
    /* START */
    I2C1->CR1 |= I2C_CR1_START;
    if (wait_flag(I2C_SR1_SB) != I2C_OK) {
        return 0;
    }

    /* 送 addr + W */
    I2C1->DR = (addr << 1) | 0;

    /* 等 ADDR 或 AF */
    uint32_t start = systick_get_ms();
    int found = 0;
    while (1) {
        if (I2C1->SR1 & I2C_SR1_ADDR) {
            /* 有裝置：清 ADDR，送 STOP */
            (void)I2C1->SR1; (void)I2C1->SR2;
            I2C1->CR1 |= I2C_CR1_STOP;
            found = 1;
            break;
        }
        if (I2C1->SR1 & I2C_SR1_AF) {
            /* 沒裝置：清 AF，送 STOP */
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            found = 0;
            break;
        }
        if ((systick_get_ms() - start) >= I2C_TIMEOUT_MS) {
            I2C1->CR1 |= I2C_CR1_STOP;
            found = 0;
            break;
        }
    }
    return found;
}

int i2c1_bus_scan(uint8_t *found, int max)
{
    int count = 0;

    /* 0x00~0x07 和 0x78~0x7F 是 I2C 保留位址，不掃 */
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (count >= max) {
            break;
        }
        if (i2c1_bus_probe(addr)) {
            found[count++] = addr;
        }
    }

    return count;
}