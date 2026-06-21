/*
 * driver_rcc.c
 *
 *  PLL → 168MHz 時脈初始化。詳見 driver_rcc.h。
 *
 *  來源策略：以外部 8MHz 晶振(HSE)為主，HSE 在 timeout 內沒起振就 fallback
 *  回內建 16MHz RC(HSI)，永不卡死。兩條路最後都是：
 *      VCO = 336MHz → P=2 → SYSCLK=168MHz、Q=7 → 48MHz
 *  差別只在 VCO 輸入分頻：
 *      HSE 8MHz ：M=8 → 1MHz × N=336 = 336MHz
 *      HSI 16MHz：M=8 → 2MHz × N=168 = 336MHz
 *  下游 PCLK1=42MHz / PCLK2=84MHz 不論走哪條都一致，driver 常數免動。
 *
 *  Created on: Jun 21, 2026
 *      Author: xushu
 */

#include "driver_rcc.h"
#include "stm32f407xx.h"

/* HSE 起振 timeout（busy-loop 次數，此時還在 16MHz、SysTick 尚未起）。
 * 8MHz 晶振典型起振 < 2ms，這裡留到約數十毫秒的餘裕，夠保守。 */
#define HSE_TIMEOUT     0x80000U

/* 兩條路共用的 PLL 參數 */
#define PLL_M           8U
#define PLL_P           2U          /* PLLP 欄位 = (P/2 - 1) = 0 */
#define PLL_Q           7U
#define PLL_N_HSE       336U        /* HSE 8MHz 用 */
#define PLL_N_HSI       168U        /* HSI 16MHz 用（fallback）*/

/* 振盪器標稱頻率（算 SYSCLK 用）*/
#define HSE_VALUE_HZ    8000000U
#define HSI_VALUE_HZ    16000000U

/* 快取的匯流排頻率：rcc_update_clocks() 從暫存器算一次後存這裡，
 * driver 透過 rcc_*_hz() 讀取（單一真相來源 = 硬體，改 PLL 不用再手改 driver）。
 * 預設值對應 reset 後的 HSI 16MHz。 */
static uint32_t s_hclk_hz  = HSI_VALUE_HZ;   /* AHB  (HCLK)  */
static uint32_t s_pclk1_hz = HSI_VALUE_HZ;   /* APB1 (PCLK1) */
static uint32_t s_pclk2_hz = HSI_VALUE_HZ;   /* APB2 (PCLK2) */

/* 試著起 HSE，回傳 1 = 起振成功、0 = timeout（要 fallback）。 */
static int hse_try_start(void)
{
    RCC->CR |= RCC_CR_HSEON;
    for (volatile uint32_t i = 0; i < HSE_TIMEOUT; i++) {
        if (RCC->CR & RCC_CR_HSERDY) {
            return 1;
        }
    }
    return (RCC->CR & RCC_CR_HSERDY) ? 1 : 0;
}

/* 從 RCC 暫存器反推目前實際的 HCLK / PCLK1 / PCLK2，存進快取。
 * 這就是 HAL 的 HAL_RCC_GetPCLKxFreq() / CMSIS SystemCoreClockUpdate() 的做法：
 * 真相來源是硬體本身，所以改了 PLL 設定這裡自動跟著對，不會 drift。 */
static void rcc_update_clocks(void)
{
    /* AHB / APB 分頻位元 → 右移位數查表（RM0090 §7.3.3，含 AHB 跳過 /32）*/
    static const uint8_t ahb_shift[16] = { 0,0,0,0,0,0,0,0, 1,2,3,4,6,7,8,9 };
    static const uint8_t apb_shift[8]  = { 0,0,0,0, 1,2,3,4 };

    uint32_t cfgr = RCC->CFGR;
    uint32_t sysclk;

    switch (cfgr & RCC_CFGR_SWS_Msk) {
    case RCC_CFGR_SWS_PLL: {                    /* SYSCLK = PLL：從 PLLCFGR 算 */
        uint32_t pll = RCC->PLLCFGR;
        uint32_t m   = pll & 0x3FU;
        uint32_t n   = (pll >> 6) & 0x1FFU;
        uint32_t p   = ((((pll >> 16) & 0x3U) + 1U) * 2U);
        uint32_t src = (pll & RCC_PLLCFGR_PLLSRC_HSE) ? HSE_VALUE_HZ : HSI_VALUE_HZ;
        sysclk = (src / m) * n / p;             /* M=8 → 中間值不溢位 */
        break;
    }
    case (1U << 2):                             /* SWS=01：HSE 直接當 SYSCLK */
        sysclk = HSE_VALUE_HZ;
        break;
    default:                                    /* SWS=00：HSI */
        sysclk = HSI_VALUE_HZ;
        break;
    }

    s_hclk_hz  = sysclk     >> ahb_shift[(cfgr >> 4)  & 0xFU];
    s_pclk1_hz = s_hclk_hz  >> apb_shift[(cfgr >> 10) & 0x7U];
    s_pclk2_hz = s_hclk_hz  >> apb_shift[(cfgr >> 13) & 0x7U];
}

uint32_t rcc_hclk_hz(void)  { return s_hclk_hz;  }
uint32_t rcc_pclk1_hz(void) { return s_pclk1_hz; }
uint32_t rcc_pclk2_hz(void) { return s_pclk2_hz; }

void rcc_clock_init_168mhz(void)
{
    uint32_t pll_n;
    uint32_t pll_src;

    /* 1. 選時脈來源：先試 HSE，失敗就退回 HSI（HSI reset 後本來就開著）。 */
    if (hse_try_start()) {
        pll_n   = PLL_N_HSE;
        pll_src = RCC_PLLCFGR_PLLSRC_HSE;       /* PLLSRC bit22 = 1 */
    } else {
        RCC->CR |= RCC_CR_HSION;                /* 保險：確認 HSI 開著 */
        while (!(RCC->CR & RCC_CR_HSIRDY)) { }
        RCC->CR &= ~RCC_CR_HSEON;               /* 關掉沒起來的 HSE */
        pll_n   = PLL_N_HSI;
        pll_src = 0U;                           /* PLLSRC = 0 = HSI */
    }

    /* 2. 開 PWR clock，把電壓調節器設到 Scale 1（168MHz 必須）。 */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS_SCALE1;

    /* 3. bus 分頻：APB1=42MHz、APB2=84MHz（先設好，避免切到 PLL 後超頻）。
     *    AHB=/1。注意 APB1 上限 42MHz、APB2 上限 84MHz，不能再快。 */
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1
              |  RCC_CFGR_PPRE1_DIV4
              |  RCC_CFGR_PPRE2_DIV2;

    /* 4. 設 PLLCFGR 前必須先關 PLL（reset 後本就關著，這裡顯式防呆 / 防 re-init）。 */
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) { }

    /* 5. 設定主 PLL：M/P/Q 固定，N 與來源依上面選定。整段一次寫好。 */
    RCC->PLLCFGR = (PLL_M               << RCC_PLLCFGR_PLLM_Pos)
                 | (pll_n               << RCC_PLLCFGR_PLLN_Pos)
                 | (((PLL_P / 2U) - 1U) << RCC_PLLCFGR_PLLP_Pos)
                 | pll_src
                 | (PLL_Q               << RCC_PLLCFGR_PLLQ_Pos);

    /* 6. 起 PLL，等鎖定。 */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }

    /* 7. flash 等待週期：168MHz @ 3.3V 要 5WS，並開 prefetch + I/D cache。
     *    關鍵是要在「切到 PLL 之前」設好（下一步才真的升到 168MHz），否則取指失敗。 */
    FLASH->ACR = FLASH_ACR_LATENCY_5WS
               | FLASH_ACR_PRFTEN
               | FLASH_ACR_ICEN
               | FLASH_ACR_DCEN;

    /* 8. 把 SYSCLK 切到 PLL，等硬體確認（SWS 反映實際來源）。 */
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL) { }

    /* 9. 算一次實際的 HCLK/PCLK1/PCLK2 存快取，driver 用 rcc_*_hz() 讀。
     * 之後 SYSCLK=168MHz、PCLK1=42MHz、PCLK2=84MHz——但這些值由暫存器算出，
     * 改 PLL 設定不必再手動同步任何 driver。 */
    rcc_update_clocks();
}
