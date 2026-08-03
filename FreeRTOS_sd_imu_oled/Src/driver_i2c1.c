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

/* ========================================================================= */
/*                          Configuration                                    */
/* ========================================================================= */

/* I2C1 在 APB1 → fCK = rcc_pclk1_hz()（目前 42MHz）。CR2 FREQ / CCR / TRISE
 * 都由它即時推導（見 i2c1_init），改 PLL 設定不用再動這裡。 */

/* 每個 flag 等待的 timeout（ms）
 * 100kHz 下讀 14 byte 約 1.5ms，100ms 綽綽有餘，
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

    /* CCR：standard mode 100kHz
     * CCR = APB1 / (2 × I2C_speed) = 42M / (2 × 100k) = 210 */
    I2C1->CCR = pclk1 / (2U * 100000U);

    /* TRISE：standard mode = APB1(MHz) + 1 */
    I2C1->TRISE = pclk1_mhz + 1U;

    /* 啟用 */
    I2C1->CR1 |= I2C_CR1_PE;
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