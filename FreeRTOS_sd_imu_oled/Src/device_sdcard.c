/*
 * device_sdcard.c
 *
 *  SD 卡 (SPI 模式) driver — exp13
 *  ─────────────────────────────────────────────
 *  完整初始化流程：
 *    1. SPI 鮑率降到 ≤ 400kHz（規格要求，這裡用 250kHz），整段 init 都維持
 *    2. CS 拉高，送 ≥74 個 clock（10 個 0xFF）喚醒卡
 *    3. CMD0   (GO_IDLE_STATE)  → R1 = 0x01（進 idle state）
 *    4. CMD8   (SEND_IF_COND)   → 判斷 v1.x / v2.0+，並確認電壓範圍
 *    5. ACMD41 (SD_SEND_OP_COND)→ 反覆送直到 R1 = 0x00（卡離開 idle）
 *    6. CMD58  (READ_OCR)       → 讀 OCR，CCS bit 判斷 SDHC / SDSC
 *    7. 鮑率加速到 4MHz
 *  讀寫：
 *    sd_read_block  (CMD17) / sd_write_block (CMD24)，單一 512B block。
 *    SDHC 用 block 號定址、SDSC 用 byte 位址，由 sd_block_to_addr() 處理。
 *  ─────────────────────────────────────────────
 *
 *  Created on: Jun 16, 2026
 *      Author: xushu
 */

#include "device_sdcard.h"
#include "driver_spi1.h"
#include "systick.h"
#include "stm32f407xx.h"

/* CS = PB10，軟體控制。BSRR 高 16 bit reset、低 16 bit set。 */
#define SD_CS_HIGH()   (GPIOB->BSRR = (1U << 10))        // set   PB10 = 1
#define SD_CS_LOW()    (GPIOB->BSRR = (1U << (10 + 16))) // reset PB10 = 0

/* SPI 鮑率（PCLK2 = 16MHz） */
#define SD_BAUD_INIT   5   /* /64 = 250kHz，符合 init ≤ 400kHz 規定 */
#define SD_BAUD_FAST   1   /* /4  = 4MHz，init 完成後加速（麵包板保守值） */

/* ACMD41 等卡完成 init 的逾時（ms）。規格上限約 1 秒，留點餘裕。 */
#define SD_ACMD41_TIMEOUT_MS  1500

/* 讀：等 data token 的逾時；寫：等卡 busy 結束的逾時（ms） */
#define SD_READ_TIMEOUT_MS    200
#define SD_WRITE_TIMEOUT_MS   500

/* 資料 token */
#define SD_TOKEN_START        0xFE   /* 單 block 讀/寫的起始 token */

/* 最後一次命令收到的原始 R1，給 main 印出來 debug 用：
 *   0xFF = 完全沒回應（卡沒電 / MISO 沒通 / CS 沒拉低）
 *   0x01 = 正常 idle
 *   其他 = R1 error bit（例如 0x05 = illegal command） */
volatile uint8_t sd_last_r1 = 0xFF;

/* 卡片類型（CMD8 判定結果），見 .h 的 SD_TYPE_* */
volatile uint8_t sd_card_type = SD_TYPE_UNKNOWN;

/* 最後一次 CMD55 (APP_CMD) 的 R1，debug 用：
 *   正常應該 = 0x01（idle）。若 = 0x05（illegal cmd）代表 CMD55 沒被認得，
 *   那 ACMD41 其實是被當成普通命令，永遠不會完成 init。 */
volatile uint8_t sd_last_cmd55_r1 = 0xFF;

/* CMD8 的 4-byte trailing（R7 後段），debug 用：
 *   r7[2] = 電壓範圍（0x01 = 2.7~3.6V）
 *   r7[3] = check pattern echo（應該 = 0xAA） */
volatile uint8_t sd_r7[4] = { 0 };

/* CMD58 讀回的 OCR（R3 後段，32-bit、MSB first），debug 用：
 *   ocr[0] bit7 = 卡 power-up 完成；ocr[0] bit6 = CCS（1 = SDHC/SDXC） */
volatile uint8_t sd_ocr[4] = { 0 };

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

/* 拉高 CS 並補一個 dummy byte，讓卡釋放 DO（每次交易收尾用）。 */
static void sd_deselect(void)
{
    SD_CS_HIGH();
    spi1_transfer_byte(0xFF);
}

/* 送一個命令 frame（6 byte）+ 讀 R1。不碰 CS —— 由呼叫者自己管。
 *   格式：6 bytes = [0x40|cmd][arg31..24][arg23..16][arg15..8][arg7..0][CRC]
 *   R1 的特徵：MSB = 0，所以連讀最多 8 次直到收到 MSB=0 的 byte。
 *
 *   讀寫 block 要在資料階段維持 CS 拉低，所以才把這段抽出來共用。 */
static uint8_t sd_cmd_frame(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    spi1_transfer_byte(0xFF);                  // 同步用 dummy（讓卡準備好）

    spi1_transfer_byte(0x40 | cmd);
    spi1_transfer_byte((uint8_t)(arg >> 24));
    spi1_transfer_byte((uint8_t)(arg >> 16));
    spi1_transfer_byte((uint8_t)(arg >> 8));
    spi1_transfer_byte((uint8_t)(arg));
    spi1_transfer_byte(crc);

    /* 等 R1：MSB=0 才是有效回應，最多試 8 次 */
    uint8_t r1 = 0xFF;
    for (int i = 0; i < 8; i++) {
        r1 = spi1_transfer_byte(0xFF);
        if ((r1 & 0x80) == 0) {
            break;
        }
    }
    return r1;
}

/* 送一個短命令並回傳 R1，CS 自己拉低 / 拉高（init 階段那些命令用）。
 *   resp4 != NULL 時，在 R1 之後再讀 4 個 byte（給 R3/R7 用，例如 CMD8、CMD58）。 */
static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc, uint8_t *resp4)
{
    SD_CS_LOW();
    uint8_t r1 = sd_cmd_frame(cmd, arg, crc);

    /* R3 / R7：R1 之後固定再跟 4 個 byte */
    if (resp4) {
        for (int i = 0; i < 4; i++) {
            resp4[i] = spi1_transfer_byte(0xFF);
        }
    }

    sd_deselect();
    return r1;
}

/* 送一個 ACMD：規定先送 CMD55 (APP_CMD) 告訴卡「下一個是 app command」，
 * 再送真正的 ACMDxx。回傳 ACMD 的 R1。CRC 在這些命令是被忽略的，送 0x01 即可。 */
static uint8_t sd_send_acmd(uint8_t acmd, uint32_t arg)
{
    sd_last_cmd55_r1 = sd_send_cmd(55, 0x00000000, 0x01, 0);  // CMD55；idle 時 = 0x01
    return sd_send_cmd(acmd, arg, 0x01, 0);
}

/* ============================================================================
 *                              Public API
 * ============================================================================ */

uint8_t sd_init(void)
{
    sd_card_type = SD_TYPE_UNKNOWN;

    /* 1. 降速到 250kHz（init 規格 ≤ 400kHz），整段 init 都維持 */
    spi1_set_baud(SD_BAUD_INIT);

    /* 2. CS 拉高，送 ≥74 clock 喚醒卡（10 byte × 8 = 80 clock） */
    SD_CS_HIGH();
    for (int i = 0; i < 10; i++) {
        spi1_transfer_byte(0xFF);
    }

    /* 3. CMD0 (GO_IDLE_STATE)，重試幾次
     *    arg = 0，CRC = 0x95（CMD0 在 SPI 模式下 CRC 是固定值）
     *    線路有雜訊時第一發常常讀到 garbage（例如 0x7E），多送幾次同步上即可。 */
    uint8_t r1 = 0xFF;
    for (int i = 0; i < 10; i++) {
        r1 = sd_send_cmd(0, 0x00000000, 0x95, 0);
        if (r1 == 0x01) {
            break;
        }
        delay_ms(1);
    }
    sd_last_r1 = r1;
    if (r1 != 0x01) {                          // 期望 0x01 = in idle state
        return SD_ERR_CMD0;
    }

    /* 4. CMD8 (SEND_IF_COND)
     *    arg = 0x000001AA：VHS=1（2.7~3.6V）+ check pattern 0xAA
     *    CRC = 0x87（CMD8 在 SPI 模式下也是固定 CRC）
     *    這是 R7 回應：R1 + 4 byte。 */
    uint8_t r7[4];
    r1 = sd_send_cmd(8, 0x000001AA, 0x87, r7);
    sd_last_r1 = r1;
    sd_r7[0] = r7[0];
    sd_r7[1] = r7[1];
    sd_r7[2] = r7[2];
    sd_r7[3] = r7[3];

    if (r1 == 0x01) {
        /* v2.0+ 卡：要確認電壓與 pattern 都回對，否則卡不可用 */
        if (r7[2] == 0x01 && r7[3] == 0xAA) {
            sd_card_type = SD_TYPE_V2;
        } else {
            return SD_ERR_CMD8;                // 電壓不支援 / pattern 對不上
        }
    } else if (r1 & 0x04) {
        /* illegal command bit = 1 → 不認得 CMD8 → v1.x SD 卡 (或 MMC) */
        sd_card_type = SD_TYPE_V1;
    } else {
        return SD_ERR_CMD8;                    // 其他非預期 R1
    }

    /* 5. ACMD41 (SD_SEND_OP_COND)：反覆送，直到卡完成內部 init（R1 = 0x00）。
     *    bit30 HCS = 1 告訴卡「主機支援高容量(SDHC)」，只有 v2 卡要設；
     *    bit20 (0x00300000) = 3.2~3.4V 電壓視窗，有些挑的卡光 HCS 不夠，要連這個一起給。 */
    uint32_t acmd41_arg = (sd_card_type == SD_TYPE_V2) ? 0x40300000 : 0x00300000;
    uint32_t start = systick_get_ms();
    do {
        r1 = sd_send_acmd(41, acmd41_arg);
        if (r1 == 0x00) {
            break;                             // 0 = 離開 idle，init 完成
        }
        if (systick_get_ms() - start > SD_ACMD41_TIMEOUT_MS) {
            sd_last_r1 = r1;
            return SD_ERR_ACMD41;              // 逾時還沒好
        }
        delay_ms(1);                           // 別把匯流排塞滿，給卡喘息
    } while (1);
    sd_last_r1 = r1;

    /* 6. CMD58 (READ_OCR)：讀 OCR，看 CCS bit(30) 判斷定址方式。
     *    這是 R3 回應：R1 + 4 byte OCR（MSB first）。 */
    uint8_t ocr[4];
    r1 = sd_send_cmd(58, 0x00000000, 0x01, ocr);
    sd_last_r1 = r1;
    if (r1 != 0x00) {
        return SD_ERR_CMD58;
    }
    sd_ocr[0] = ocr[0];
    sd_ocr[1] = ocr[1];
    sd_ocr[2] = ocr[2];
    sd_ocr[3] = ocr[3];

    /* CCS = OCR bit30 = ocr[0] 的 bit6：
     *   1 → SDHC/SDXC：用 block(512B) 當位址單位
     *   0 → SDSC：用 byte 當位址單位 */
    if (sd_card_type == SD_TYPE_V2 && (ocr[0] & 0x40)) {
        sd_card_type = SD_TYPE_SDHC;
    }

    /* 7. init 全部完成，鮑率加速到 4MHz */
    spi1_set_baud(SD_BAUD_FAST);

    return SD_OK;
}

/* block 號 → 命令參數：SDHC/SDXC 用 block 號，SDSC 用 byte 位址(block*512)。 */
static uint32_t sd_block_to_addr(uint32_t block)
{
    return (sd_card_type == SD_TYPE_SDHC) ? block : (block * SD_BLOCK_SIZE);
}

uint8_t sd_read_block(uint32_t block, uint8_t *buf)
{
    SD_CS_LOW();

    /* CMD17 (READ_SINGLE_BLOCK)，CRC 被忽略送 0x01 */
    uint8_t r1 = sd_cmd_frame(17, sd_block_to_addr(block), 0x01);
    if (r1 != 0x00) {
        sd_deselect();
        return SD_ERR_READ;
    }

    /* 等 data token：busy 時卡回 0xFF，收到非 0xFF 就停。
     * 正常是 0xFE；其他值代表 error token。 */
    uint8_t token = 0xFF;
    uint32_t start = systick_get_ms();
    do {
        token = spi1_transfer_byte(0xFF);
        if (token != 0xFF) {
            break;
        }
        if (systick_get_ms() - start > SD_READ_TIMEOUT_MS) {
            sd_deselect();
            return SD_ERR_READ;
        }
    } while (1);

    if (token != SD_TOKEN_START) {             // error token，不是 0xFE
        sd_deselect();
        return SD_ERR_READ;
    }

    /* 讀 512 byte 資料 + 2 byte CRC（丟掉，CRC 沒開） */
    for (int i = 0; i < SD_BLOCK_SIZE; i++) {
        buf[i] = spi1_transfer_byte(0xFF);
    }
    spi1_transfer_byte(0xFF);
    spi1_transfer_byte(0xFF);

    sd_deselect();
    return SD_OK;
}

uint8_t sd_write_block(uint32_t block, const uint8_t *buf)
{
    SD_CS_LOW();

    /* CMD24 (WRITE_SINGLE_BLOCK) */
    uint8_t r1 = sd_cmd_frame(24, sd_block_to_addr(block), 0x01);
    if (r1 != 0x00) {
        sd_deselect();
        return SD_ERR_WRITE;
    }

    /* token 前先送一個 dummy（規格建議的 Nwr 間隔），再送 start token */
    spi1_transfer_byte(0xFF);
    spi1_transfer_byte(SD_TOKEN_START);

    /* 送 512 byte 資料 + 2 byte 假 CRC */
    for (int i = 0; i < SD_BLOCK_SIZE; i++) {
        spi1_transfer_byte(buf[i]);
    }
    spi1_transfer_byte(0xFF);
    spi1_transfer_byte(0xFF);

    /* data response token：低 5 bit = xxx00101 (0x05) 代表卡已接受 */
    uint8_t resp = spi1_transfer_byte(0xFF);
    if ((resp & 0x1F) != 0x05) {
        sd_deselect();
        return SD_ERR_WRITE;
    }

    /* 卡寫 flash 期間會把 MISO 拉低(回 0x00) = busy，等到回 0xFF 才算寫完 */
    uint32_t start = systick_get_ms();
    while (spi1_transfer_byte(0xFF) == 0x00) {
        if (systick_get_ms() - start > SD_WRITE_TIMEOUT_MS) {
            sd_deselect();
            return SD_ERR_WRITE;
        }
    }

    sd_deselect();
    return SD_OK;
}
