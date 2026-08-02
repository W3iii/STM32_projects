/*
 * device_tempsensor.c
 *
 *  STM32F407 內建溫度感測器 — ADC1 內部 channel 16。詳見 device_tempsensor.h。
 *
 *  精度改良（VREFINT 補償）：
 *    原本假設 VDDA = 3.3V 去換算感測器電壓，但實際供電常不到 3.3V（Discovery
 *    板尤其），會把溫度系統性算高。這裡改讀晶片內建的 VREFINT（channel 17，
 *    固定 ~1.21V 帶隙基準）當「已知電壓尺」：
 *        adc_ref  = VREFINT / VDDA × 4095
 *        adc_temp = V_sense / VDDA × 4095
 *    兩式相除，VDDA 約分掉：
 *        V_sense = adc_temp / adc_ref × VREFINT
 *    這樣不必知道 VDDA 究竟幾伏，供電飄動也自動補償。
 *
 *  Created on: Aug 2, 2026
 *      Author: xushu
 */

#include "device_tempsensor.h"
#include "stm32f407xx.h"

/* ========================================================================= */
/*                  溫度換算常數（F407 datasheet 典型值）                     */
/* ========================================================================= */
/* F407 無出廠校正，只能用 datasheet 典型值，絕對精度有限（±1.5°C 起，且
 * V25 個體差異可達 ±18°C）。要更準：在已知溫度下量一次做單點校正微調 TS_V25。 */
#define TS_V25          0.76f       /* 25°C 時的感測器輸出電壓（V） */
#define TS_AVG_SLOPE    0.0025f     /* 斜率 2.5 mV/°C */
#define TS_VREFINT_V    1.21f       /* VREFINT 帶隙基準典型值（V），VDDA 尺規 */
#define ADC_MAX         4095.0f     /* 12-bit 滿刻度 */

/* ========================================================================= */
/*                       Register bit definitions                            */
/* (stm32f407xx.h 沒提供 ADC bit macro，這裡自己定義，同 i2c driver 風格)     */
/* ========================================================================= */

/* CR2 */
#define ADC_CR2_ADON        (1U << 0)    /* A/D converter ON */
#define ADC_CR2_SWSTART     (1U << 30)   /* Start conversion of regular channels */

/* SR */
#define ADC_SR_EOC          (1U << 1)    /* Regular channel end of conversion */

/* CCR (ADC common) */
#define ADC_CCR_ADCPRE_DIV4 (1U << 16)   /* ADCPRE=01：ADCCLK = PCLK2/4 */
#define ADC_CCR_TSVREFE     (1U << 23)   /* 開啟溫度感測器 + VREFINT 內部通道 */

#define TS_ADC_CHANNEL      16U          /* 內建溫度感測器 = ADC1_IN16 */
#define VREF_ADC_CHANNEL    17U          /* 內建 VREFINT      = ADC1_IN17 */
#define ADC_EOC_TIMEOUT     100000U      /* 轉換等待上限（迴圈次數），防呆不卡死 */

/* 把 channel ch（10~18）的取樣時間設成 480 cycles（SMPR1 每通道 3 bit）。
 * 內部感測器要求較長取樣時間（≥10us）；ADCCLK=21MHz 下 480 cycles ≈ 22.9us。 */
static void adc_set_sample_480(uint32_t ch)
{
    uint32_t shift = (ch - 10U) * 3U;
    ADC1->SMPR1 &= ~(0x7U << shift);
    ADC1->SMPR1 |=  (0x7U << shift);
}

void tempsensor_init(void)
{
    /* 內建感測器 / VREFINT 是 ADC 內部通道，不需要任何 GPIO / 類比腳設定。 */

    /* ADC1 時脈（APB2）。ADCCLK 上限 36MHz：PCLK2=84MHz，選 /4 = 21MHz。
     * 同時開啟溫度感測器 + VREFINT（TSVREFE）。 */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC_COMMON->CCR &= ~(0x3U << 16);
    ADC_COMMON->CCR |=  ADC_CCR_ADCPRE_DIV4 | ADC_CCR_TSVREFE;

    /* 規則序列長度 1（SQR1 L=0）；實際 channel 每次讀時再寫 SQR3。 */
    ADC1->SQR1 = 0;
    adc_set_sample_480(TS_ADC_CHANNEL);
    adc_set_sample_480(VREF_ADC_CHANNEL);

    /* 12-bit（CR1 RES=00，reset 值即是）、單次（CR2 CONT=0），開 ADC。 */
    ADC1->CR1 = 0;
    ADC1->CR2 = ADC_CR2_ADON;
}

/* 觸發單通道轉換，回傳 12-bit raw（0..4095）。EOC 逾時就回目前 DR（不卡死）。 */
static uint16_t adc_read_channel(uint32_t ch)
{
    ADC1->SQR3 = ch;                        /* SQ1 = 這個通道 */
    ADC1->SR = 0;                           /* 清 EOC 等旗標 */
    ADC1->CR2 |= ADC_CR2_SWSTART;           /* 觸發規則轉換 */

    uint32_t guard = 0;
    while (!(ADC1->SR & ADC_SR_EOC)) {
        if (++guard >= ADC_EOC_TIMEOUT) {
            break;                          /* 硬體異常：不永遠等 */
        }
    }
    return (uint16_t)(ADC1->DR & 0x0FFFU);  /* 讀 DR 會清 EOC */
}

uint16_t tempsensor_read_raw(void)
{
    return adc_read_channel(TS_ADC_CHANNEL);
}

float tempsensor_vdda(void)
{
    uint16_t adc_ref = adc_read_channel(VREF_ADC_CHANNEL);
    if (adc_ref == 0) {
        return 0.0f;                        /* 異常：避免除以 0 */
    }
    return TS_VREFINT_V * ADC_MAX / (float)adc_ref;
}

float tempsensor_read_c(void)
{
    uint16_t adc_temp = adc_read_channel(TS_ADC_CHANNEL);
    uint16_t adc_ref  = adc_read_channel(VREF_ADC_CHANNEL);
    if (adc_ref == 0) {
        return -999.0f;                     /* VREFINT 讀不到：回離譜值而非除以 0 */
    }

    /* VDDA 約分掉：V_sense = adc_temp / adc_ref × VREFINT */
    float v_sense = (float)adc_temp / (float)adc_ref * TS_VREFINT_V;
    return (v_sense - TS_V25) / TS_AVG_SLOPE + 25.0f;
}
