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
#define configTOTAL_HEAP_SIZE                   ( 28 * 1024 )   /* heap_4：5 task 堆疊 + qSamples（含建立期尖峰）*/
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1

/* ---- 命名 / debug ---- */
#define configMAX_TASK_NAME_LEN                 ( 12 )
#define configUSE_TRACE_FACILITY                1               /* tasks CLI：uxTaskGetSystemState / run-time stats 依賴此項 */
#define configCHECK_FOR_STACK_OVERFLOW          2               /* pattern check（bring-up 期保險）*/
#define configUSE_MALLOC_FAILED_HOOK            1

/* ---- Run-time stats（tasks CLI：算每個 task 的 CPU%）----
 * 需要一個比 tick(1ms) 更細的計時源。用 TIM2 自由跑（app_runtime_stats.c），
 * 頻率 ~100kHz（= tick 的 100 倍），port 每次量測就讀 TIM2->CNT。 */
#define configGENERATE_RUN_TIME_STATS           1
#define configUSE_STATS_FORMATTING_FUNCTIONS    1
extern void vConfigureTimerForRunTimeStats(void);
extern unsigned long ulGetRunTimeCounterValue(void);
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()  vConfigureTimerForRunTimeStats()
#define portGET_RUN_TIME_COUNTER_VALUE()          ulGetRunTimeCounterValue()

/* ---- hook ---- */
#define configUSE_IDLE_HOOK                     1   /* vApplicationIdleHook：閒置時 WFI 睡覺（見 app_tasks.c）*/
#define configUSE_TICK_HOOK                     0
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

/* ---- 同步原語（Phase 2 會用到，先打開）---- */
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               4

/* ---- software timer ----
 * 關閉：整個 app 沒用到軟體計時器（xTimerCreate 那套）、event group、
 * 也沒有 xTimerPendFunctionCall，所以不需要 Timer Service(daemon) task。
 * 關掉可省下它的堆疊 + timer 命令佇列（給 heap 多點餘裕），`tasks` 也不再列它。
 * 之後若要用軟體計時器，把這裡改回 1 即可（timers.c 已在編譯清單，內部靠此旗標啟用）。 */
#define configUSE_TIMERS                        0
#define configTIMER_TASK_PRIORITY               ( 3 )   /* configUSE_TIMERS=0 時未使用 */
#define configTIMER_QUEUE_LENGTH                8       /* 同上 */
#define configTIMER_TASK_STACK_DEPTH            ( 256 ) /* 同上 */

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

/* ---- assert：抓 priority / heap 設定錯誤。失敗時 vAssertCalled() 會把
 * 觸發的檔案:行號從 UART 印出來再卡住（定義在 main.c），不再靜默死迴圈。 ---- */
extern void vAssertCalled(const char *file, unsigned long line);
#define configASSERT( x )   if( ( x ) == 0 ) vAssertCalled( __FILE__, __LINE__ )

/* ---- 讓 port 接管 startup .s 裡的 weak handler ---- */
#define vPortSVCHandler         SVC_Handler
#define xPortPendSVHandler      PendSV_Handler
#define xPortSysTickHandler     SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
