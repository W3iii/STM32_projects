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

/* APB1 clock (I2C1 在 APB1) — 目前 16MHz HSI，APB1 prescaler = 1 */
#define APB1_MHZ            16U

/* 每個 flag 等待的 timeout（ms）
 * 100kHz 下讀 14 byte 約 1.5ms，100ms 綽綽有餘，
 * 又不會在真的斷線時讓使用者等太久。
 */
#define I2C_TIMEOUT_MS      100U


/* ========================================================================= */
/*                          GPIO setup (shared)                              */
/* ========================================================================= */

/*
 * 設定 PB6/PB9 為 I2C1 AF mode（open-drain + pull-up）
 * init 和 bus_recovery 結束後都會呼叫，所以抽成 static helper（決策 2）
 */
static void i2c1_gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* PB6 = I2C1_SCL, PB9 = I2C1_SDA → AF mode */
    GPIOB->MODER &= ~(0x3U << (6 * 2));
    GPIOB->MODER |=  (GPIO_MODER_AF << (6 * 2));
    GPIOB->MODER &= ~(0x3U << (9 * 2));
    GPIOB->MODER |=  (GPIO_MODER_AF << (9 * 2));

    GPIOB->AFRL &= ~(0xFU << (6 * 4));
    GPIOB->AFRL |=  (GPIO_AF4_I2C1 << (6 * 4));
    GPIOB->AFRH &= ~(0xFU << ((9 - 8) * 4));
    GPIOB->AFRH |=  (GPIO_AF4_I2C1 << ((9 - 8) * 4));

    /* I2C 必須 open-drain + pull-up */
    GPIOB->OTYPER |= (1U << 6);
    GPIOB->OTYPER |= (1U << 9);
    GPIOB->PUPDR &= ~(0x3U << (6 * 2));
    GPIOB->PUPDR |=  (0x1U << (6 * 2));
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
 * 專門等 ADDR flag，因為送完位址後有兩種結果（決策 3）：
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

    /* CR2 FREQ：告訴硬體 APB1 頻率（MHz） */
    I2C1->CR2 = APB1_MHZ;

    /* CCR：standard mode 100kHz
     * CCR = APB1 / (2 × I2C_speed) = 16M / (2 × 100k) = 80
     */
    I2C1->CCR = (APB1_MHZ * 1000000U) / (2U * 100000U);   /* = 80 */

    /* TRISE：standard mode = APB1_MHz + 1 */
    I2C1->TRISE = APB1_MHZ + 1U;

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
 */
static void short_delay(void)
{
    for (volatile int i = 0; i < 80; i++) { __asm volatile("nop"); }
}

i2c_status_t i2c1_bus_recovery(void)
{
    /* 1. disable I2C peripheral，準備手動操作腳位 */
    I2C1->CR1 &= ~I2C_CR1_PE;

    /* 2. PB6(SCL)、PB9(SDA) 切成 GPIO output（open-drain 保留）*/
    GPIOB->MODER &= ~(0x3U << (6 * 2));
    GPIOB->MODER |=  (GPIO_MODER_OUTPUT << (6 * 2));
    GPIOB->MODER &= ~(0x3U << (9 * 2));
    GPIOB->MODER |=  (GPIO_MODER_OUTPUT << (9 * 2));
    /* OTYPER 在 gpio_init 已設 open-drain，SDA 拉高靠 pull-up */
    GPIOB->ODR |= (1U << 9);    /* SDA 放開（high）*/
    GPIOB->ODR |= (1U << 6);    /* SCL high */

    /* 3. 送最多 9 個 clock pulse，期間檢查 SDA 是否被釋放 */
    i2c_status_t result = I2C_ERR_BUS;
    for (int i = 0; i < 9; i++) {
        /* 如果 SDA 已經回到 high，slave 釋放了 → 成功 */
        if (GPIOB->IDR & (1U << 9)) {
            result = I2C_OK;
            break;
        }
        GPIOB->ODR &= ~(1U << 6);   /* SCL low */
        short_delay();
        GPIOB->ODR |=  (1U << 6);   /* SCL high */
        short_delay();
    }

    /* 4. 產生一次 STOP condition（SDA 由 low → high 當 SCL high）*/
    GPIOB->ODR &= ~(1U << 9);   /* SDA low */
    short_delay();
    GPIOB->ODR |=  (1U << 6);   /* SCL high */
    short_delay();
    GPIOB->ODR |=  (1U << 9);   /* SDA high → STOP */
    short_delay();

    /* 5. 重新 init I2C（gpio_init 會把腳位切回 AF mode）*/
    i2c1_init();

    return result;
}