/*
 * dma_mem.h
 *
 *  DMA 能不能存取這塊記憶體？（STM32F407，RM0090 §2.1 bus matrix）
 *
 *  DMA1/DMA2 的 memory port 只接得到 Flash 與主 SRAM（SRAM1 + SRAM2）。
 *  CCM（0x1000_0000，64KB）只接在 CPU 的 D-bus 上，DMA 讀寫它會觸發 TEIF。
 *
 *  本專案的 FreeRTOS heap 在 CCM（HEAP_IN_CCM=1）→ 所有 task stack 也在 CCM。
 *  所以 task 裡的 local 陣列直接丟給 DMA 是會出事的；每個「接外部 buffer
 *  再啟動 DMA」的 driver 入口都先過這一關，擋下來回錯誤碼，不要讓 DMA
 *  跑下去才在 ISR 收到 TEIF。
 *
 *  用白名單而不是只擋 CCM：NULL、外設區、算錯的指標也一併擋掉。
 */

#ifndef DMA_MEM_H_
#define DMA_MEM_H_

#include <stdint.h>

#define DMA_MEM_FLASH_START   0x08000000UL      /* 1MB Flash（const 資料可當 TX 來源）*/
#define DMA_MEM_FLASH_END     0x08100000UL
#define DMA_MEM_SRAM_START    0x20000000UL      /* SRAM1 112KB + SRAM2 16KB，位址連續 */
#define DMA_MEM_SRAM_END      0x20020000UL

/* [p, p+len) 整段都在 DMA 碰得到的區域才回 1。len = 0 視為不合法。 */
static inline int dma_mem_ok(const void *p, uint32_t len)
{
    uint32_t a = (uint32_t)p;

    if (len == 0) {
        return 0;
    }
    if (a >= DMA_MEM_SRAM_START && a < DMA_MEM_SRAM_END) {
        return len <= DMA_MEM_SRAM_END - a;     /* 寫成減法，避免 a + len 溢位 */
    }
    if (a >= DMA_MEM_FLASH_START && a < DMA_MEM_FLASH_END) {
        return len <= DMA_MEM_FLASH_END - a;
    }
    return 0;
}

#endif /* DMA_MEM_H_ */
