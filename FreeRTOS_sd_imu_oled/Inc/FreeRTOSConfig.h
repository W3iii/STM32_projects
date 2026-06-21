/*
 * FreeRTOSConfig.h — STM32F407VGTX (Cortex-M4F), bare-metal datalogger
 * ─────────────────────────────────────────────────────────────────────
 *  Phase 2 config：Sensor/Logger/Display/Console 四 task + Queue + mutex。
 *  mutex / queue / timer 等 API 都已打開。
 *
 *  時脈：HSE 8MHz → PLL → 168MHz SYSCLK（rcc_clock_init_168mhz() 在 main()
 *  最前面設定）。configCPU_CLOCK_HZ 必須等於實際 SYSCLK=168e6，否則 port
 *  算出的 SysTick reload 會錯、整個 RTOS tick / vTaskDelay 都失準。
 *  configTICK_RATE_HZ=1000 讓 1 tick = 1ms，對齊原本的 ms 時間基準。
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* ---- 排程器 ---- */
#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1   /* CM4F 有 CLZ，用硬體挑 task */
#define configUSE_TIME_SLICING                  1
#define configUSE_16_BIT_TICKS                  0

/* ---- 時脈 / tick ---- */
#define configCPU_CLOCK_HZ                      ( 168000000UL ) /* HSE 8M→PLL 168MHz */
#define configTICK_RATE_HZ                      ( 1000 )        /* 1 tick = 1ms */

/* ---- 記憶體 ---- */
#define configMINIMAL_STACK_SIZE                ( 128 )         /* words (idle task) */
#define configMAX_PRIORITIES                    ( 5 )
#define configTOTAL_HEAP_SIZE                   ( 20 * 1024 )   /* heap_4：4 task 堆疊 + qSamples，F407 有 128KB RAM */
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1

/* ---- 命名 / debug ---- */
#define configMAX_TASK_NAME_LEN                 ( 12 )
#define configUSE_TRACE_FACILITY                0
#define configCHECK_FOR_STACK_OVERFLOW          2               /* pattern check（bring-up 期保險）*/
#define configUSE_MALLOC_FAILED_HOOK            1

/* ---- hook ---- */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

/* ---- 同步原語（Phase 2 會用到，先打開）---- */
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               4

/* ---- software timer（timers.c 已納入編譯）---- */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( 3 )
#define configTIMER_QUEUE_LENGTH                8
#define configTIMER_TASK_STACK_DEPTH            ( 256 )

/* ---- 選用 API ---- */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_uxTaskGetStackHighWaterMark     1

/* ---- Cortex-M NVIC 中斷優先權 ----
 *  STM32 用 4 個優先權 bit。SysTick/PendSV 由 port 設成最低優先權。
 *  任何呼叫 FreeRTOS ...FromISR() 的 ISR，其優先權數值必須 >=
 *  MAX_SYSCALL（= 邏輯上較低優先）。Phase 1 的 USART/DMA ISR 不碰
 *  FreeRTOS API，所以沿用預設 priority 0 沒問題（見 plan 的 Phase 2 caveat）。*/
#define configPRIO_BITS                         4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY        15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY   5
#define configKERNEL_INTERRUPT_PRIORITY \
        ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
        ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* ---- assert：抓 priority / heap 設定錯誤，bring-up 期直接卡住好定位 ---- */
#define configASSERT( x )   if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

/* ---- 讓 port 接管 startup .s 裡的 weak handler ---- */
#define vPortSVCHandler         SVC_Handler
#define xPortPendSVHandler      PendSV_Handler
#define xPortSysTickHandler     SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
