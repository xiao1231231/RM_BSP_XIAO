/**
 * @file    BMI088_Task.cpp
 * @brief   姿态解算任务 —— 把陀螺 FIFO 里的样本逐个喂给 VQF
 *
 * @note    来源：H7_BSP（zzm）。任务的创建由 CubeMX 生成的 Core/Src/freertos.c
 *          负责（osThreadNew(BMI088_Task, ...)），本文件只提供实现 ——
 *          覆盖 freertos.c 里那个 __weak 空壳，和 TIM_1ms_Task 一个模式。
 *
 * ── 为什么要单独一个任务 ────────────────────────────────────────────
 *   陀螺 2kHz 出数据，SPI 回调只能"收字节"（中断里不能跑 VQF 这种浮点重活）。
 *   于是：中断收完 → 置线程标志 → 本任务被唤醒 → 逐帧 Calculate() 直到队列空。
 *   这样【中断里只搬数据、任务里才算姿态】，两边都不互相拖累。
 *   收不完也没关系：FIFO 里还攒着，下一轮继续（队列深 128，够缓冲）。
 * ────────────────────────────────────────────────────────────────────
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088.h"

#include "cmsis_os2.h"

/* Function prototypes -------------------------------------------------------*/

extern "C" void BMI088_Task(void *argument)
{
    (void)argument;

    /* ★ 本任务保持 CubeMX 配置的 Normal（freertos.c：.priority = osPriorityNormal），
     *   【不】像上游那样把优先级提到 High2。
     *
     *   上游提优先级是为了降低 FIFO 排队延迟；但那会让本任务在"有样本要算"时
     *   完全压住 1ms 任务，而每次唤醒它会把队列里的样本一口气算完（每帧 ~155µs），
     *   连着 3~4 帧就是半毫秒的独占 → 1ms 任务被推迟 → 丢拍。
     *   实测：提优先级时 overrun 涨到 ~45% 的节拍。
     *   与 1ms 任务同级（Normal）时，FreeRTOS 按节拍在两者间轮转，两边都能按时跑；
     *   多出来的那点延迟由陀螺 FIFO（99 帧 ≈ 50ms）吸收，完全不影响姿态质量。
     *   ⚠️ 若在 CubeMX 里改本任务优先级，必须 ≤ Normal（理由同上）。 */

    for (;;)
    {
        /* 0x0001 = 有样本入队（要 Calculate）
         * 0x0002 = FIFO 没读完（要接着发下一笔传输）
         * 任意一个先到就醒 */
        const uint32_t flags =
            osThreadFlagsWait(0x0003, osFlagsWaitAny, osWaitForever);

        if ((flags & 0x0002) != 0U)
        {
            BSP_BMI088.BMI088_Service_Transfer(true);
        }

        if (BSP_BMI088.BMI088_Gyro.Get_Queue_Depth() != 0U)
        {
            uint32_t budget = 3U;
            do
            {
                BSP_BMI088.Calculate();
                /* 连算 3 帧就让出一次 CPU：单帧 ≈155µs，3 帧 ≈465µs，
                 * 既算得动又不会让 1ms 任务等过 1ms */
                if (--budget == 0U)
                {
                    budget = 3U;
                    osDelay(1U);
                }
            } while (BSP_BMI088.BMI088_Gyro.Get_Queue_Depth() != 0U);
        }
    }
}
