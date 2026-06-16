/*
 * device_sdcard.c
 *
 *  SD 卡 (SPI 模式) driver — exp13 第一步：CMD0 初始化
 *  ─────────────────────────────────────────────
 *  初始化流程（本檔只做到 CMD0）：
 *    1. SPI 鮑率降到 ≤ 400kHz（規格要求，這裡用 250kHz）
 *    2. CS 拉高，送 ≥74 個 clock（10 個 0xFF）喚醒卡
 *    3. CS 拉低，送 CMD0 (GO_IDLE_STATE)
 *    4. 讀 R1 回應，應該 = 0x01（in idle state）
 *    5. CS 拉高，補一個 dummy byte
 *  ─────────────────────────────────────────────
 *
 *  Created on: Jun 16, 2026
 *      Author: xushu
 */

#include "device_sdcard.h"
#include "driver_spi1.h"
#include "stm32f407xx.h"

/* CS = PB10，軟體控制。BSRR 高 16 bit reset、低 16 bit set。 */
#define SD_CS_HIGH()   (GPIOB->BSRR = (1U << 10))        // set   PB10 = 1
#define SD_CS_LOW()    (GPIOB->BSRR = (1U << (10 + 16))) // reset PB10 = 0

/* SPI 鮑率（PCLK2 = 16MHz） */
#define SD_BAUD_INIT   5   /* /64 = 250kHz，符合 init ≤ 400kHz 規定 */

/* 最後一次 CMD0 收到的原始 R1，給 main 印出來 debug 用：
 *   0xFF = 完全沒回應（卡沒電 / MISO 沒通 / CS 沒拉低）
 *   0x01 = 正常 idle
 *   其他 = R1 error bit（例如 0x05 = illegal command） */
volatile uint8_t sd_last_r1 = 0xFF;

/* ============================================================================
 *                              CS GPIO
 * ============================================================================ */

void sd_cs_init(void)
{
    RCC->AHB1ENR |= (1 << 1);                  // GPIOB clock

    GPIOB->MODER  &= ~(0x3 << (10 * 2));
    GPIOB->MODER  |=  (0x1 << (10 * 2));        // PB10 = output
    GPIOB->OTYPER &= ~(1 << 10);               // 推挽
    GPIOB->OSPEEDR |= (0x3 << (10 * 2));        // high speed
    GPIOB->PUPDR  &= ~(0x3 << (10 * 2));        // 無上下拉

    SD_CS_HIGH();                              // 預設不選中
}

/* ============================================================================
 *                              Private helpers
 * ============================================================================ */

/* 送一個 CMD，回傳 R1。
 * 格式：6 bytes = [0x40|cmd][arg31..24][arg23..16][arg15..8][arg7..0][CRC]
 * R1 的特徵：MSB = 0，所以連讀最多 8 次直到收到 MSB=0 的 byte。 */
static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    spi1_transfer_byte(0x40 | cmd);
    spi1_transfer_byte((uint8_t)(arg >> 24));
    spi1_transfer_byte((uint8_t)(arg >> 16));
    spi1_transfer_byte((uint8_t)(arg >> 8));
    spi1_transfer_byte((uint8_t)(arg));
    spi1_transfer_byte(crc);

    /* 等 R1：MSB=0 才是有效回應，最多試 8 次 */
    uint8_t r1;
    for (int i = 0; i < 8; i++) {
        r1 = spi1_transfer_byte(0xFF);
        if ((r1 & 0x80) == 0) {
            break;
        }
    }
    return r1;
}

/* ============================================================================
 *                              Public API
 * ============================================================================ */

uint8_t sd_init(void)
{
    /* 1. 降速到 250kHz（init 規格 ≤ 400kHz） */
    spi1_set_baud(SD_BAUD_INIT);

    /* 2. CS 拉高，送 ≥74 clock 喚醒卡（10 byte × 8 = 80 clock） */
    SD_CS_HIGH();
    for (int i = 0; i < 10; i++) {
        spi1_transfer_byte(0xFF);
    }

    /* 3. CS 拉低，送 CMD0 (GO_IDLE_STATE)
     *    arg = 0，CRC = 0x95（CMD0 在 SPI 模式下 CRC 是固定值） */
    SD_CS_LOW();
    uint8_t r1 = sd_send_cmd(0, 0x00000000, 0x95);
    sd_last_r1 = r1;                 // 存起來給 main debug 印出

    /* 5. CS 拉高，補一個 dummy byte 讓卡完成內部狀態切換 */
    SD_CS_HIGH();
    spi1_transfer_byte(0xFF);

    /* 4. R1 應該 = 0x01（in idle state） */
    if (r1 != 0x01) {
        return SD_ERR_CMD0;
    }

    /* 5. CMD0 成功，換回原本速度 1MHz */
    spi1_set_baud(3);   // /16 = 1MHz
    
    return SD_OK;
}
