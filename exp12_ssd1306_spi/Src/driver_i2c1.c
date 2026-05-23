/*
 * i2c_driver.c
 *
 *  Created on: May 18, 2026
 *      Author: xushu
 */

#include "driver_i2c1.h"
#include "stm32f407xx.h"

static void i2c1_gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // PB6 = I2C1_SCL, PB9 = I2C1_SDA
    GPIOB->MODER &= ~(0x3U << (6 * 2));
    GPIOB->MODER |=  (GPIO_MODER_AF << (6 * 2));
    GPIOB->MODER &= ~(0x3U << (9 * 2));
    GPIOB->MODER |=  (GPIO_MODER_AF << (9 * 2));

    GPIOB->AFRL &= ~(0xFU << (6 * 4));
    GPIOB->AFRL |=  (GPIO_AF4_I2C1 << (6 * 4));
    GPIOB->AFRH &= ~(0xFU << ((9 - 8) * 4));
    GPIOB->AFRH |=  (GPIO_AF4_I2C1 << ((9 - 8) * 4));

    // I2C bus needs open-drain pins with pull-up.
    GPIOB->OTYPER |= (1U << 6);
    GPIOB->OTYPER |= (1U << 9);
    GPIOB->PUPDR &= ~(0x3U << (6 * 2));
    GPIOB->PUPDR |=  (0x1U << (6 * 2));
    GPIOB->PUPDR &= ~(0x3U << (9 * 2));
    GPIOB->PUPDR |=  (0x1U << (9 * 2));
}

void i2c1_init(void) {
    i2c1_gpio_init();

    RCC -> APB1ENR |= RCC_APB1ENR_I2C1EN;   // I2C1

	I2C1 -> CR1 &= ~(1 << 0);

	// 4. 軟體 reset 一下（保險）
	I2C1->CR1 |=  (1 << 15);     // SWRST
	I2C1->CR1 &= ~(1 << 15);

	// 5. 告訴硬體 APB1 是 16MHz
	I2C1->CR2 = 16;              // FREQ[5:0] = 16

	// 6. CCR：standard mode 100kHz
	// CCR = APB1 / (2 × I2C_speed) = 16M / 200k = 80
	I2C1->CCR = 80;

	// 7. TRISE：最大上升時間
	// TRISE = APB1_MHz + 1 = 17（standard mode）
	I2C1->TRISE = 17;

	// 8. 啟用 I2C
	I2C1->CR1 |= (1 << 0);       // PE = 1
}

void i2c1_write_reg(uint8_t slave_addr, uint8_t reg_addr, uint8_t data) {
	// 1. START
	I2C1->CR1 |= (1 << 8);                    // START
	while (!(I2C1->SR1 & (1 << 0)));          // 等 SB

	// 2. 送 slave + W
	I2C1->DR = (slave_addr << 1) | 0;         // 寫 DR 同時清 SB
	while (!(I2C1->SR1 & (1 << 1)));          // 等 ADDR

	// 3. 清 ADDR（讀 SR1 + 讀 SR2）
	(void)I2C1->SR1;
	(void)I2C1->SR2;

	// 4. 送 reg_addr
	while (!(I2C1->SR1 & (1 << 7)));          // 等 TXE
	I2C1->DR = reg_addr;

	// 5. 送 data
	while (!(I2C1->SR1 & (1 << 7)));          // 等 TXE
	I2C1->DR = data;

	// 6. 等完整傳完
	while (!(I2C1->SR1 & (1 << 2)));          // 等 BTF

	// 7. STOP
	I2C1->CR1 |= (1 << 9);                    // STOP
}

void i2c1_read_regs(uint8_t slave, uint8_t reg, uint8_t *buf, uint8_t len) {
    if (len == 0) return;

    // ============ 前半段：指定 register ============
    I2C1->CR1 |= (1 << 8);                    // START
    while (!(I2C1->SR1 & (1 << 0)));          // 等 SB

    I2C1->DR = (slave << 1) | 0;              // addr + W
    while (!(I2C1->SR1 & (1 << 1)));          // 等 ADDR
    (void)I2C1->SR1; (void)I2C1->SR2;         // 清 ADDR

    while (!(I2C1->SR1 & (1 << 7)));          // 等 TXE
    I2C1->DR = reg;
    while (!(I2C1->SR1 & (1 << 2)));          // 等 BTF（要 RESTART）

    // ============ RESTART → 切讀 ============
    I2C1->CR1 |= (1 << 8);                    // RESTART
    while (!(I2C1->SR1 & (1 << 0)));          // 等 SB

    I2C1->DR = (slave << 1) | 1;              // addr + R

    // ============ 三種 case 分開處理 ============
    if (len == 1) {
        // ---- Case 1：讀單 byte ----
        // 清 ADDR 之前要先 ACK = 0
        I2C1->CR1 &= ~(1 << 10);              // ACK = 0

        while (!(I2C1->SR1 & (1 << 1)));      // 等 ADDR
        (void)I2C1->SR1; (void)I2C1->SR2;     // 清 ADDR

        I2C1->CR1 |= (1 << 9);                // STOP

        while (!(I2C1->SR1 & (1 << 6)));      // 等 RXNE
        buf[0] = I2C1->DR;

    } else if (len == 2) {
        // ---- Case 2：讀 2 byte（特殊，要設 POS）----
        I2C1->CR1 |= (1 << 10);               // ACK = 1
        I2C1->CR1 |= (1 << 11);               // POS = 1

        while (!(I2C1->SR1 & (1 << 1)));      // 等 ADDR
        (void)I2C1->SR1; (void)I2C1->SR2;     // 清 ADDR

        I2C1->CR1 &= ~(1 << 10);              // ACK = 0（給第 2 byte 用）

        while (!(I2C1->SR1 & (1 << 2)));      // 等 BTF（兩 byte 都收完）
        I2C1->CR1 |= (1 << 9);                // STOP

        buf[0] = I2C1->DR;
        buf[1] = I2C1->DR;

        I2C1->CR1 &= ~(1 << 11);              // 清 POS（恢復正常）

    } else {
        // ---- Case 3：讀 N >= 3 byte ----
        I2C1->CR1 |= (1 << 10);               // ACK = 1

        while (!(I2C1->SR1 & (1 << 1)));      // 等 ADDR
        (void)I2C1->SR1; (void)I2C1->SR2;     // 清 ADDR

        // 讀前 N-3 byte（迴圈每次 ACK）
        for (int i = 0; i < len - 3; i++) {
            while (!(I2C1->SR1 & (1 << 6)));  // 等 RXNE
            buf[i] = I2C1->DR;
        }

        // 讀倒數第 3 byte，要等 BTF（不是 RXNE）
        while (!(I2C1->SR1 & (1 << 2)));      // 等 BTF
        I2C1->CR1 &= ~(1 << 10);              // ACK = 0

        buf[len - 3] = I2C1->DR;              // 讀倒數第 3

        while (!(I2C1->SR1 & (1 << 2)));      // 等 BTF
        I2C1->CR1 |= (1 << 9);                // STOP

        buf[len - 2] = I2C1->DR;              // 讀倒數第 2
        buf[len - 1] = I2C1->DR;              // 讀最後一 byte
    }
}
