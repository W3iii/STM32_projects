/*
 * stm32f407xx.h
 *
 *  Created on: Apr 19, 2026
 *      Author: xushu
 */

#ifndef STM32F407XX_H_
#define STM32F407XX_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                          Memory Map                                       */
/* ========================================================================= */
/*
 * 0x00000000 - 0x07FFFFFF  Code (aliased to Flash/SRAM/etc)
 * 0x08000000 - 0x080FFFFF  Flash (1 MB)
 * 0x20000000 - 0x2001FFFF  SRAM (128 KB) + CCM (64 KB @ 0x10000000)
 * 0x40000000 - 0x4007FFFF  Peripherals
 * 0xE0000000 - 0xE00FFFFF  Cortex-M4 internal (SysTick, NVIC, SCB)
 */

/* ========================================================================= */
/*                          Base Addresses                                   */
/* ========================================================================= */

/* --- AHB1 peripherals --- */
#define GPIOA_BASE          0x40020000UL
#define GPIOB_BASE          0x40020400UL
#define GPIOC_BASE          0x40020800UL
#define GPIOD_BASE          0x40020C00UL
#define GPIOE_BASE          0x40021000UL
#define RCC_BASE            0x40023800UL
#define DMA1_BASE           0x40026000UL
#define DMA2_BASE           0x40026400UL

/* --- APB1 peripherals --- */
#define TIM2_BASE           0x40000000UL
#define TIM3_BASE           0x40000400UL
#define TIM4_BASE           0x40000800UL
#define TIM5_BASE           0x40000C00UL
#define SPI2_BASE           0x40003800UL
#define SPI3_BASE           0x40003C00UL
#define USART2_BASE         0x40004400UL
#define USART3_BASE         0x40004800UL
#define I2C1_BASE           0x40005400UL
#define I2C2_BASE           0x40005800UL
#define I2C3_BASE           0x40005C00UL

/* --- APB2 peripherals --- */
#define TIM1_BASE           0x40010000UL
#define USART1_BASE         0x40011000UL
#define USART6_BASE         0x40011400UL
#define ADC1_BASE           0x40012000UL
#define ADC_COMMON_BASE     0x40012300UL
#define SPI1_BASE           0x40013000UL
#define SYSCFG_BASE         0x40013800UL
#define EXTI_BASE           0x40013C00UL

/* --- Cortex-M4 internal --- */
#define SYSTICK_BASE        0xE000E010UL
#define NVIC_BASE           0xE000E100UL
#define SCB_BASE            0xE000ED00UL

/* ========================================================================= */
/*                    Peripheral Struct Definitions                          */
/* ========================================================================= */

/* -------------------------------------------------------------------------
 * GPIO (RM0090 §8.4)
 * Each GPIO port: 10 registers, offsets 0x00 ~ 0x24
 * ------------------------------------------------------------------------- */
typedef struct {
    volatile uint32_t MODER;        /* 0x00  Mode register                  */
    volatile uint32_t OTYPER;       /* 0x04  Output type register           */
    volatile uint32_t OSPEEDR;      /* 0x08  Output speed register          */
    volatile uint32_t PUPDR;        /* 0x0C  Pull-up/pull-down register     */
    volatile uint32_t IDR;          /* 0x10  Input data register            */
    volatile uint32_t ODR;          /* 0x14  Output data register           */
    volatile uint32_t BSRR;         /* 0x18  Bit set/reset register         */
    volatile uint32_t LCKR;         /* 0x1C  Lock register                  */
    volatile uint32_t AFRL;         /* 0x20  AF low register  (pin 0~7)     */
    volatile uint32_t AFRH;         /* 0x24  AF high register (pin 8~15)    */
} GPIO_TypeDef;

/* -------------------------------------------------------------------------
 * RCC (RM0090 §7.3)
 * ------------------------------------------------------------------------- */
typedef struct {
    volatile uint32_t CR;           /* 0x00  Clock control                  */
    volatile uint32_t PLLCFGR;      /* 0x04  PLL configuration              */
    volatile uint32_t CFGR;         /* 0x08  Clock configuration            */
    volatile uint32_t CIR;          /* 0x0C  Clock interrupt                */
    volatile uint32_t AHB1RSTR;     /* 0x10  AHB1 reset                    */
    volatile uint32_t AHB2RSTR;     /* 0x14  AHB2 reset                    */
    volatile uint32_t AHB3RSTR;     /* 0x18  AHB3 reset                    */
    uint32_t          _reserved0;   /* 0x1C                                 */
    volatile uint32_t APB1RSTR;     /* 0x20  APB1 reset                    */
    volatile uint32_t APB2RSTR;     /* 0x24  APB2 reset                    */
    uint32_t          _reserved1[2];/* 0x28, 0x2C                           */
    volatile uint32_t AHB1ENR;      /* 0x30  AHB1 clock enable             */
    volatile uint32_t AHB2ENR;      /* 0x34  AHB2 clock enable             */
    volatile uint32_t AHB3ENR;      /* 0x38  AHB3 clock enable             */
    uint32_t          _reserved2;   /* 0x3C                                 */
    volatile uint32_t APB1ENR;      /* 0x40  APB1 clock enable             */
    volatile uint32_t APB2ENR;      /* 0x44  APB2 clock enable             */
    uint32_t          _reserved3[2];/* 0x48, 0x4C                           */
    volatile uint32_t AHB1LPENR;    /* 0x50  AHB1 low-power clock enable   */
    volatile uint32_t AHB2LPENR;    /* 0x54  AHB2 low-power clock enable   */
    volatile uint32_t AHB3LPENR;    /* 0x58  AHB3 low-power clock enable   */
    uint32_t          _reserved4;   /* 0x5C                                 */
    volatile uint32_t APB1LPENR;    /* 0x60  APB1 low-power clock enable   */
    volatile uint32_t APB2LPENR;    /* 0x64  APB2 low-power clock enable   */
    uint32_t          _reserved5[2];/* 0x68, 0x6C                           */
    volatile uint32_t BDCR;         /* 0x70  Backup domain control         */
    volatile uint32_t CSR;          /* 0x74  Clock control & status        */
    uint32_t          _reserved6[2];/* 0x78, 0x7C                           */
    volatile uint32_t SSCGR;        /* 0x80  Spread spectrum clock gen     */
    volatile uint32_t PLLI2SCFGR;   /* 0x84  PLLI2S configuration          */
} RCC_TypeDef;

/* -------------------------------------------------------------------------
 * USART (RM0090 §30.6)
 * Covers USART1, USART2, USART3, USART6
 * ------------------------------------------------------------------------- */
typedef struct {
    volatile uint32_t SR;           /* 0x00  Status register                */
    volatile uint32_t DR;           /* 0x04  Data register                  */
    volatile uint32_t BRR;          /* 0x08  Baud rate register             */
    volatile uint32_t CR1;          /* 0x0C  Control register 1             */
    volatile uint32_t CR2;          /* 0x10  Control register 2             */
    volatile uint32_t CR3;          /* 0x14  Control register 3             */
    volatile uint32_t GTPR;         /* 0x18  Guard time and prescaler       */
} USART_TypeDef;

/* -------------------------------------------------------------------------
 * DMA (RM0090 §10.5)
 *
 * DMA has 2 controllers, each with 8 streams (0~7).
 * Common registers (ISR/IFCR) at offset 0x00~0x0C,
 * then per-stream registers starting at 0x10, stride = 0x18.
 * ------------------------------------------------------------------------- */

/* Per-stream registers */
typedef struct {
    volatile uint32_t CR;           /* +0x00  Stream configuration          */
    volatile uint32_t NDTR;         /* +0x04  Number of data to transfer    */
    volatile uint32_t PAR;          /* +0x08  Peripheral address            */
    volatile uint32_t M0AR;         /* +0x0C  Memory 0 address              */
    volatile uint32_t M1AR;         /* +0x10  Memory 1 address (double buf) */
    volatile uint32_t FCR;          /* +0x14  FIFO control                  */
} DMA_Stream_TypeDef;

/* DMA controller (common registers) */
typedef struct {
    volatile uint32_t LISR;         /* 0x00  Low interrupt status           */
    volatile uint32_t HISR;         /* 0x04  High interrupt status          */
    volatile uint32_t LIFCR;        /* 0x08  Low interrupt flag clear       */
    volatile uint32_t HIFCR;        /* 0x0C  High interrupt flag clear      */
} DMA_TypeDef;

/*
 * Stream access:
 *   DMA2_Stream7->CR = ...;
 *
 * Stream N base = DMA_BASE + 0x10 + 0x18 * N
 */

/* -------------------------------------------------------------------------
 * TIM - General purpose timer (RM0090 §18)
 * Covers TIM2~TIM5 (32-bit), TIM1 (advanced, slightly different but
 * the common registers share the same offsets up to 0x54)
 * ------------------------------------------------------------------------- */
typedef struct {
    volatile uint32_t CR1;          /* 0x00  Control register 1             */
    volatile uint32_t CR2;          /* 0x04  Control register 2             */
    volatile uint32_t SMCR;         /* 0x08  Slave mode control             */
    volatile uint32_t DIER;         /* 0x0C  DMA/interrupt enable           */
    volatile uint32_t SR;           /* 0x10  Status register                */
    volatile uint32_t EGR;          /* 0x14  Event generation               */
    volatile uint32_t CCMR1;        /* 0x18  Capture/compare mode 1         */
    volatile uint32_t CCMR2;        /* 0x1C  Capture/compare mode 2         */
    volatile uint32_t CCER;         /* 0x20  Capture/compare enable         */
    volatile uint32_t CNT;          /* 0x24  Counter                        */
    volatile uint32_t PSC;          /* 0x28  Prescaler                      */
    volatile uint32_t ARR;          /* 0x2C  Auto-reload register           */
    volatile uint32_t RCR;          /* 0x30  Repetition counter (TIM1 only) */
    volatile uint32_t CCR1;         /* 0x34  Capture/compare 1              */
    volatile uint32_t CCR2;         /* 0x38  Capture/compare 2              */
    volatile uint32_t CCR3;         /* 0x3C  Capture/compare 3              */
    volatile uint32_t CCR4;         /* 0x40  Capture/compare 4              */
    volatile uint32_t BDTR;         /* 0x44  Break and dead-time (TIM1)     */
    volatile uint32_t DCR;          /* 0x48  DMA control register           */
    volatile uint32_t DMAR;         /* 0x4C  DMA address for full transfer  */
    volatile uint32_t OR;           /* 0x50  Option register (TIM2/5)       */
} TIM_TypeDef;

/* -------------------------------------------------------------------------
 * EXTI (RM0090 §12.3)
 * ------------------------------------------------------------------------- */
typedef struct {
    volatile uint32_t IMR;          /* 0x00  Interrupt mask                 */
    volatile uint32_t EMR;          /* 0x04  Event mask                     */
    volatile uint32_t RTSR;         /* 0x08  Rising trigger selection       */
    volatile uint32_t FTSR;         /* 0x0C  Falling trigger selection      */
    volatile uint32_t SWIER;        /* 0x10  Software interrupt event       */
    volatile uint32_t PR;           /* 0x14  Pending register               */
} EXTI_TypeDef;

/* -------------------------------------------------------------------------
 * SYSCFG (RM0090 §9.2)
 * ------------------------------------------------------------------------- */
typedef struct {
    volatile uint32_t MEMRMP;       /* 0x00  Memory remap                   */
    volatile uint32_t PMC;          /* 0x04  Peripheral mode config         */
    volatile uint32_t EXTICR[4];    /* 0x08  EXTI config registers 1~4      */
    uint32_t          _reserved[2]; /* 0x18, 0x1C                           */
    volatile uint32_t CMPCR;        /* 0x20  Compensation cell control      */
} SYSCFG_TypeDef;

/* -------------------------------------------------------------------------
 * I2C (RM0090 §27.6)
 * ------------------------------------------------------------------------- */
typedef struct {
    volatile uint32_t CR1;          /* 0x00  Control register 1             */
    volatile uint32_t CR2;          /* 0x04  Control register 2             */
    volatile uint32_t OAR1;         /* 0x08  Own address register 1         */
    volatile uint32_t OAR2;         /* 0x0C  Own address register 2         */
    volatile uint32_t DR;           /* 0x10  Data register                  */
    volatile uint32_t SR1;          /* 0x14  Status register 1              */
    volatile uint32_t SR2;          /* 0x18  Status register 2              */
    volatile uint32_t CCR;          /* 0x1C  Clock control register         */
    volatile uint32_t TRISE;        /* 0x20  Rise time register             */
    volatile uint32_t FLTR;         /* 0x24  Digital noise filter           */
} I2C_TypeDef;

/* -------------------------------------------------------------------------
 * SPI (RM0090 §28.5)
 * ------------------------------------------------------------------------- */
typedef struct {
    volatile uint32_t CR1;          /* 0x00  Control register 1             */
    volatile uint32_t CR2;          /* 0x04  Control register 2             */
    volatile uint32_t SR;           /* 0x08  Status register                */
    volatile uint32_t DR;           /* 0x0C  Data register                  */
    volatile uint32_t CRCPR;        /* 0x10  CRC polynomial register        */
    volatile uint32_t RXCRCR;       /* 0x14  RX CRC register                */
    volatile uint32_t TXCRCR;       /* 0x18  TX CRC register                */
    volatile uint32_t I2SCFGR;      /* 0x1C  I2S configuration              */
    volatile uint32_t I2SPR;        /* 0x20  I2S prescaler                  */
} SPI_TypeDef;

/* -------------------------------------------------------------------------
 * ADC (RM0090 §13.13)
 * Per-ADC registers + common registers
 * ------------------------------------------------------------------------- */
typedef struct {
    volatile uint32_t SR;           /* 0x00  Status register                */
    volatile uint32_t CR1;          /* 0x04  Control register 1             */
    volatile uint32_t CR2;          /* 0x08  Control register 2             */
    volatile uint32_t SMPR1;        /* 0x0C  Sample time register 1         */
    volatile uint32_t SMPR2;        /* 0x10  Sample time register 2         */
    volatile uint32_t JOFR1;        /* 0x14  Injected channel offset 1      */
    volatile uint32_t JOFR2;        /* 0x18  Injected channel offset 2      */
    volatile uint32_t JOFR3;        /* 0x1C  Injected channel offset 3      */
    volatile uint32_t JOFR4;        /* 0x20  Injected channel offset 4      */
    volatile uint32_t HTR;          /* 0x24  Watchdog higher threshold      */
    volatile uint32_t LTR;          /* 0x28  Watchdog lower threshold       */
    volatile uint32_t SQR1;         /* 0x2C  Regular sequence register 1    */
    volatile uint32_t SQR2;         /* 0x30  Regular sequence register 2    */
    volatile uint32_t SQR3;         /* 0x34  Regular sequence register 3    */
    volatile uint32_t JSQR;         /* 0x38  Injected sequence register     */
    volatile uint32_t JDR1;         /* 0x3C  Injected data register 1       */
    volatile uint32_t JDR2;         /* 0x40  Injected data register 2       */
    volatile uint32_t JDR3;         /* 0x44  Injected data register 3       */
    volatile uint32_t JDR4;         /* 0x48  Injected data register 4       */
    volatile uint32_t DR;           /* 0x4C  Regular data register          */
} ADC_TypeDef;

/* ADC common registers (shared by ADC1/2/3) */
typedef struct {
    volatile uint32_t CSR;          /* 0x00  Common status                  */
    volatile uint32_t CCR;          /* 0x04  Common control                 */
    volatile uint32_t CDR;          /* 0x08  Common regular data            */
} ADC_Common_TypeDef;

/* -------------------------------------------------------------------------
 * SysTick (ARM Cortex-M4 TRM)
 * Part of the Cortex-M4 core, not STM32-specific
 * ------------------------------------------------------------------------- */
typedef struct {
    volatile uint32_t CTRL;         /* 0x00  Control and status             */
    volatile uint32_t LOAD;         /* 0x04  Reload value                   */
    volatile uint32_t VAL;          /* 0x08  Current value                  */
    volatile uint32_t CALIB;        /* 0x0C  Calibration value              */
} SysTick_TypeDef;

/* -------------------------------------------------------------------------
 * NVIC (ARM Cortex-M4 TRM)
 * Nested Vectored Interrupt Controller
 * Only commonly used registers included
 * ------------------------------------------------------------------------- */
typedef struct {
    volatile uint32_t ISER[8];      /* 0x000  Interrupt Set-Enable          */
    uint32_t          _res0[24];    /* 0x020 ~ 0x07C                        */
    volatile uint32_t ICER[8];      /* 0x080  Interrupt Clear-Enable        */
    uint32_t          _res1[24];    /* 0x0A0 ~ 0x0FC                        */
    volatile uint32_t ISPR[8];      /* 0x100  Interrupt Set-Pending         */
    uint32_t          _res2[24];    /* 0x120 ~ 0x17C                        */
    volatile uint32_t ICPR[8];      /* 0x180  Interrupt Clear-Pending       */
    uint32_t          _res3[24];    /* 0x1A0 ~ 0x1FC                        */
    volatile uint32_t IABR[8];      /* 0x200  Interrupt Active Bit          */
    uint32_t          _res4[56];    /* 0x220 ~ 0x2FC                        */
    volatile uint8_t  IPR[240];     /* 0x300  Interrupt Priority (byte)     */
    uint32_t          _res5[644];   /* 0x3F0 ~ 0xDFC                        */
    volatile uint32_t STIR;         /* 0xE00  Software Trigger Interrupt    */
} NVIC_TypeDef;

/* -------------------------------------------------------------------------
 * SCB (ARM Cortex-M4 TRM)
 * System Control Block — commonly used subset
 * ------------------------------------------------------------------------- */
typedef struct {
    volatile uint32_t CPUID;        /* 0x00  CPUID base register            */
    volatile uint32_t ICSR;         /* 0x04  Interrupt control/state        */
    volatile uint32_t VTOR;         /* 0x08  Vector table offset            */
    volatile uint32_t AIRCR;        /* 0x0C  App interrupt/reset control    */
    volatile uint32_t SCR;          /* 0x10  System control register        */
    volatile uint32_t CCR;          /* 0x14  Configuration and control      */
    volatile uint8_t  SHP[12];      /* 0x18  System handler priority        */
    volatile uint32_t SHCSR;        /* 0x24  System handler ctrl/state      */
    volatile uint32_t CFSR;         /* 0x28  Configurable fault status      */
    volatile uint32_t HFSR;         /* 0x2C  Hard fault status              */
    volatile uint32_t DFSR;         /* 0x30  Debug fault status             */
    volatile uint32_t MMFAR;        /* 0x34  MemManage fault address        */
    volatile uint32_t BFAR;         /* 0x38  Bus fault address              */
    volatile uint32_t AFSR;         /* 0x3C  Auxiliary fault status         */
} SCB_TypeDef;

/* ========================================================================= */
/*                        Peripheral Instances                               */
/* ========================================================================= */

/* --- GPIO --- */
#define GPIOA       ((GPIO_TypeDef *)  GPIOA_BASE)
#define GPIOB       ((GPIO_TypeDef *)  GPIOB_BASE)
#define GPIOC       ((GPIO_TypeDef *)  GPIOC_BASE)
#define GPIOD       ((GPIO_TypeDef *)  GPIOD_BASE)
#define GPIOE       ((GPIO_TypeDef *)  GPIOE_BASE)

/* --- RCC --- */
#define RCC         ((RCC_TypeDef *)   RCC_BASE)

/* --- USART --- */
#define USART1      ((USART_TypeDef *) USART1_BASE)
#define USART2      ((USART_TypeDef *) USART2_BASE)
#define USART3      ((USART_TypeDef *) USART3_BASE)
#define USART6      ((USART_TypeDef *) USART6_BASE)

/* --- DMA (common registers) --- */
#define DMA1        ((DMA_TypeDef *)   DMA1_BASE)
#define DMA2        ((DMA_TypeDef *)   DMA2_BASE)

/* --- DMA streams --- */
/*
 * Formula: DMA_BASE + 0x10 + 0x18 * stream_number
 *
 * DMA1: Streams 0~7
 * DMA2: Streams 0~7
 */
#define DMA1_Stream0  ((DMA_Stream_TypeDef *)(DMA1_BASE + 0x10 + 0x18 * 0))
#define DMA1_Stream1  ((DMA_Stream_TypeDef *)(DMA1_BASE + 0x10 + 0x18 * 1))
#define DMA1_Stream2  ((DMA_Stream_TypeDef *)(DMA1_BASE + 0x10 + 0x18 * 2))
#define DMA1_Stream3  ((DMA_Stream_TypeDef *)(DMA1_BASE + 0x10 + 0x18 * 3))
#define DMA1_Stream4  ((DMA_Stream_TypeDef *)(DMA1_BASE + 0x10 + 0x18 * 4))
#define DMA1_Stream5  ((DMA_Stream_TypeDef *)(DMA1_BASE + 0x10 + 0x18 * 5))
#define DMA1_Stream6  ((DMA_Stream_TypeDef *)(DMA1_BASE + 0x10 + 0x18 * 6))
#define DMA1_Stream7  ((DMA_Stream_TypeDef *)(DMA1_BASE + 0x10 + 0x18 * 7))
#define DMA2_Stream0  ((DMA_Stream_TypeDef *)(DMA2_BASE + 0x10 + 0x18 * 0))
#define DMA2_Stream1  ((DMA_Stream_TypeDef *)(DMA2_BASE + 0x10 + 0x18 * 1))
#define DMA2_Stream2  ((DMA_Stream_TypeDef *)(DMA2_BASE + 0x10 + 0x18 * 2))
#define DMA2_Stream3  ((DMA_Stream_TypeDef *)(DMA2_BASE + 0x10 + 0x18 * 3))
#define DMA2_Stream4  ((DMA_Stream_TypeDef *)(DMA2_BASE + 0x10 + 0x18 * 4))
#define DMA2_Stream5  ((DMA_Stream_TypeDef *)(DMA2_BASE + 0x10 + 0x18 * 5))
#define DMA2_Stream6  ((DMA_Stream_TypeDef *)(DMA2_BASE + 0x10 + 0x18 * 6))
#define DMA2_Stream7  ((DMA_Stream_TypeDef *)(DMA2_BASE + 0x10 + 0x18 * 7))

/* --- TIM --- */
#define TIM1        ((TIM_TypeDef *)   TIM1_BASE)
#define TIM2        ((TIM_TypeDef *)   TIM2_BASE)
#define TIM3        ((TIM_TypeDef *)   TIM3_BASE)
#define TIM4        ((TIM_TypeDef *)   TIM4_BASE)
#define TIM5        ((TIM_TypeDef *)   TIM5_BASE)

/* --- EXTI --- */
#define EXTI        ((EXTI_TypeDef *)  EXTI_BASE)

/* --- SYSCFG --- */
#define SYSCFG      ((SYSCFG_TypeDef *)SYSCFG_BASE)

/* --- I2C --- */
#define I2C1        ((I2C_TypeDef *)   I2C1_BASE)
#define I2C2        ((I2C_TypeDef *)   I2C2_BASE)
#define I2C3        ((I2C_TypeDef *)   I2C3_BASE)

/* --- SPI --- */
#define SPI1        ((SPI_TypeDef *)   SPI1_BASE)
#define SPI2        ((SPI_TypeDef *)   SPI2_BASE)
#define SPI3        ((SPI_TypeDef *)   SPI3_BASE)

/* --- ADC --- */
#define ADC1        ((ADC_TypeDef *)   ADC1_BASE)
#define ADC_COMMON  ((ADC_Common_TypeDef *) ADC_COMMON_BASE)

/* --- Cortex-M4 core --- */
#define SYSTICK     ((SysTick_TypeDef *)SysTick_BASE)
#define NVIC        ((NVIC_TypeDef *)  NVIC_BASE)
#define SCB         ((SCB_TypeDef *)   SCB_BASE)

/* ========================================================================= */
/*                        IRQ Numbers (Position)                             */
/* ========================================================================= */
/*
 * Usage with NVIC:
 *   NVIC->ISER[IRQn / 32] |= (1 << (IRQn % 32));
 *
 * Example: Enable DMA2_Stream7 (IRQ 70)
 *   NVIC->ISER[70 / 32] |= (1 << (70 % 32));
 *   → NVIC->ISER[2] |= (1 << 6);
 */
#define IRQ_EXTI0               6
#define IRQ_EXTI1               7
#define IRQ_EXTI2               8
#define IRQ_EXTI3               9
#define IRQ_EXTI4               10
#define IRQ_DMA1_Stream0        11
#define IRQ_DMA1_Stream1        12
#define IRQ_DMA1_Stream2        13
#define IRQ_DMA1_Stream3        14
#define IRQ_DMA1_Stream4        15
#define IRQ_DMA1_Stream5        16
#define IRQ_DMA1_Stream6        17
#define IRQ_ADC                 18
#define IRQ_EXTI9_5             23
#define IRQ_TIM1_UP_TIM10       25
#define IRQ_TIM1_CC             27
#define IRQ_TIM2                28
#define IRQ_TIM3                29
#define IRQ_TIM4                30
#define IRQ_I2C1_EV             31
#define IRQ_I2C1_ER             32
#define IRQ_I2C2_EV             33
#define IRQ_I2C2_ER             34
#define IRQ_SPI1                35
#define IRQ_SPI2                36
#define IRQ_USART1              37
#define IRQ_USART2              38
#define IRQ_USART3              39
#define IRQ_EXTI15_10           40
#define IRQ_TIM5                50
#define IRQ_SPI3                51
#define IRQ_DMA1_Stream7        47
#define IRQ_DMA2_Stream0        56
#define IRQ_DMA2_Stream1        57
#define IRQ_DMA2_Stream2        58
#define IRQ_DMA2_Stream3        59
#define IRQ_DMA2_Stream4        60
#define IRQ_DMA2_Stream5        68
#define IRQ_DMA2_Stream6        69
#define IRQ_DMA2_Stream7        70
#define IRQ_USART6              71
#define IRQ_I2C3_EV             72
#define IRQ_I2C3_ER             73

/* ========================================================================= */
/*                    RCC Enable Bit Positions                               */
/* ========================================================================= */

/* AHB1ENR */
#define RCC_AHB1ENR_GPIOAEN     (1U << 0)
#define RCC_AHB1ENR_GPIOBEN     (1U << 1)
#define RCC_AHB1ENR_GPIOCEN     (1U << 2)
#define RCC_AHB1ENR_GPIODEN     (1U << 3)
#define RCC_AHB1ENR_GPIOEEN     (1U << 4)
#define RCC_AHB1ENR_DMA1EN      (1U << 21)
#define RCC_AHB1ENR_DMA2EN      (1U << 22)

/* APB1ENR */
#define RCC_APB1ENR_TIM2EN      (1U << 0)
#define RCC_APB1ENR_TIM3EN      (1U << 1)
#define RCC_APB1ENR_TIM4EN      (1U << 2)
#define RCC_APB1ENR_TIM5EN      (1U << 3)
#define RCC_APB1ENR_SPI2EN      (1U << 14)
#define RCC_APB1ENR_SPI3EN      (1U << 15)
#define RCC_APB1ENR_USART2EN    (1U << 17)
#define RCC_APB1ENR_USART3EN    (1U << 18)
#define RCC_APB1ENR_I2C1EN      (1U << 21)
#define RCC_APB1ENR_I2C2EN      (1U << 22)
#define RCC_APB1ENR_I2C3EN      (1U << 23)

/* APB2ENR */
#define RCC_APB2ENR_TIM1EN      (1U << 0)
#define RCC_APB2ENR_USART1EN    (1U << 4)
#define RCC_APB2ENR_USART6EN    (1U << 5)
#define RCC_APB2ENR_ADC1EN      (1U << 8)
#define RCC_APB2ENR_SPI1EN      (1U << 12)
#define RCC_APB2ENR_SYSCFGEN    (1U << 14)

/* ========================================================================= */
/*                    DMA Flag Bit Positions                                 */
/* ========================================================================= */
/*
 * LISR / LIFCR: Stream 0~3
 * HISR / HIFCR: Stream 4~7
 *
 * Each stream has 5 flags: FEIF, DMEIF, TEIF, HTIF, TCIF
 * The bit positions within LISR/HISR are irregular — see RM0090 §10.5.
 *
 * Quick reference for commonly used TCIF (Transfer Complete):
 */
#define DMA_TCIF_STREAM0        (1U << 5)   /* LISR/LIFCR */
#define DMA_TCIF_STREAM1        (1U << 11)
#define DMA_TCIF_STREAM2        (1U << 21)
#define DMA_TCIF_STREAM3        (1U << 27)
#define DMA_TCIF_STREAM4        (1U << 5)   /* HISR/HIFCR */
#define DMA_TCIF_STREAM5        (1U << 11)
#define DMA_TCIF_STREAM6        (1U << 21)
#define DMA_TCIF_STREAM7        (1U << 27)

/* All flags for a stream (for clearing) */
#define DMA_FLAGS_STREAM0       (0x3FU << 0)
#define DMA_FLAGS_STREAM1       (0x3FU << 6)
#define DMA_FLAGS_STREAM2       (0x3FU << 16)
#define DMA_FLAGS_STREAM3       (0x3FU << 22)
#define DMA_FLAGS_STREAM4       (0x3FU << 0)
#define DMA_FLAGS_STREAM5       (0x3FU << 6)
#define DMA_FLAGS_STREAM6       (0x3FU << 16)
#define DMA_FLAGS_STREAM7       (0x3FU << 22)

/* ========================================================================= */
/*                    USART Bit Positions                                    */
/* ========================================================================= */

/* SR */
#define USART_SR_TXE            (1U << 7)   /* TX data register empty       */
#define USART_SR_TC             (1U << 6)   /* Transmission complete        */
#define USART_SR_RXNE           (1U << 5)   /* RX data register not empty   */
#define USART_SR_IDLE           (1U << 4)   /* Idle line detected           */
#define USART_SR_ORE            (1U << 3)   /* Overrun error                */

/* CR1 */
#define USART_CR1_UE            (1U << 13)  /* USART enable                 */
#define USART_CR1_TE            (1U << 3)   /* Transmitter enable           */
#define USART_CR1_RE            (1U << 2)   /* Receiver enable              */
#define USART_CR1_RXNEIE        (1U << 5)   /* RXNE interrupt enable        */
#define USART_CR1_TXEIE         (1U << 7)   /* TXE interrupt enable         */
#define USART_CR1_TCIE          (1U << 6)   /* TC interrupt enable          */
#define USART_CR1_IDLEIE        (1U << 4)   /* IDLE interrupt enable        */

/* CR3 */
#define USART_CR3_DMAT          (1U << 7)   /* DMA transmitter enable       */
#define USART_CR3_DMAR          (1U << 6)   /* DMA receiver enable          */

/* ========================================================================= */
/*                    GPIO Helpers                                           */
/* ========================================================================= */

/* MODER: 2 bits per pin */
#define GPIO_MODER_INPUT        0x0U
#define GPIO_MODER_OUTPUT       0x1U
#define GPIO_MODER_AF           0x2U
#define GPIO_MODER_ANALOG       0x3U

/*
 * Usage:
 *   GPIOA->MODER &= ~(0x3 << (pin * 2));
 *   GPIOA->MODER |=  (GPIO_MODER_AF << (pin * 2));
 */

/* AF numbers (from Datasheet Table 9) */
#define GPIO_AF0_MCO            0x0U
#define GPIO_AF1_TIM1           0x1U
#define GPIO_AF1_TIM2           0x1U
#define GPIO_AF2_TIM3           0x2U
#define GPIO_AF2_TIM4           0x2U
#define GPIO_AF2_TIM5           0x2U
#define GPIO_AF4_I2C1           0x4U
#define GPIO_AF4_I2C2           0x4U
#define GPIO_AF4_I2C3           0x4U
#define GPIO_AF5_SPI1           0x5U
#define GPIO_AF5_SPI2           0x5U
#define GPIO_AF6_SPI3           0x6U
#define GPIO_AF7_USART1         0x7U
#define GPIO_AF7_USART2         0x7U
#define GPIO_AF7_USART3         0x7U
#define GPIO_AF8_USART6         0x8U

/* ========================================================================= */
/*                    DMA Stream CR Bit Positions                            */
/* ========================================================================= */

/* Channel selection: bits [27:25] */
#define DMA_CR_CHSEL(ch)        ((uint32_t)(ch) << 25)

/* Direction: bits [7:6] */
#define DMA_CR_DIR_P2M          (0x0U << 6)   /* Peripheral → Memory        */
#define DMA_CR_DIR_M2P          (0x1U << 6)   /* Memory → Peripheral        */
#define DMA_CR_DIR_M2M          (0x2U << 6)   /* Memory → Memory            */

/* Increment modes */
#define DMA_CR_MINC             (1U << 10)    /* Memory increment           */
#define DMA_CR_PINC             (1U << 9)     /* Peripheral increment       */

/* Data size: bits [12:11] = MSIZE, [14:13] = PSIZE */
#define DMA_CR_MSIZE_8          (0x0U << 11)
#define DMA_CR_MSIZE_16         (0x1U << 11)
#define DMA_CR_MSIZE_32         (0x2U << 11)
#define DMA_CR_PSIZE_8          (0x0U << 13)
#define DMA_CR_PSIZE_16         (0x1U << 13)
#define DMA_CR_PSIZE_32         (0x2U << 13)

/* Circular mode */
#define DMA_CR_CIRC             (1U << 8)

/* Interrupts */
#define DMA_CR_TCIE             (1U << 4)     /* Transfer complete IE       */
#define DMA_CR_HTIE             (1U << 3)     /* Half transfer IE           */
#define DMA_CR_TEIE             (1U << 2)     /* Transfer error IE          */
#define DMA_CR_DMEIE            (1U << 1)     /* Direct mode error IE       */

/* Enable */
#define DMA_CR_EN               (1U << 0)

/* ========================================================================= */
/*                    Convenience Macros                                     */
/* ========================================================================= */

/* Enable NVIC interrupt */
#define NVIC_EnableIRQ(irqn)    (NVIC->ISER[(irqn) / 32] = (1U << ((irqn) % 32)))

/* Disable NVIC interrupt */
#define NVIC_DisableIRQ(irqn)   (NVIC->ICER[(irqn) / 32] = (1U << ((irqn) % 32)))

/* Set NVIC priority (0~255, lower = higher priority) */
#define NVIC_SetPriority(irqn, prio) (NVIC->IPR[(irqn)] = (uint8_t)((prio) << 4))

#ifdef __cplusplus
}
#endif

#endif /* STM32F407XX_H_ */
