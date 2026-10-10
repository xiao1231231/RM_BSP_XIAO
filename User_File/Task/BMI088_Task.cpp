/**
 * @file    BMI088_Task.cpp
 * @brief   消费陀螺软件样本队列，逐帧运行 VQF 并发布姿态
 *
 * @note    来源：H7_BSP（zzm）。任务的创建由 CubeMX 生成的 Core/Src/freertos.c
 *          负责（osThreadNew(BMI088_Task, ...)），本文件只提供实现 ——
 *          覆盖 freertos.c 中的同名 __weak 入口。
 *
 * @note    SPI 完成中断解析 FIFO、入队并通知本任务；IMU_Service_Task 负责
 *          周期通信服务和 FIFO 续传，本任务负责解算与姿态发布。
 *          软件队列数组长 128，最多存 127 帧；队列满时丢弃新样本并计数。
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088.h"
#include "sys_attitude.h"
#include "cmsis_os2.h"

extern "C" void BMI088_Task(void *argument)
{
    (void)argument;

    // 优先级由 CubeMX 配置；当前为 Normal，低于 Motor 和 IMU_Service。

    for (;;)
    {
        // 只等待样本入队通知
        const uint32_t flags =
            osThreadFlagsWait(0x0001U, osFlagsWaitAny, osWaitForever);

        if ((flags & osFlagsError) != 0U)
        {
            osDelay(1U);
            continue;
        }

        if (BSP_BMI088.BMI088_Gyro.Get_Queue_Depth() != 0U)
        {
            uint32_t budget = 3U;
            do
            {
                BSP_BMI088.Calculate();
                Attitude_Task();
                // 每处理 3 帧休眠 1 tick；修改预算后需验证队列积压和掉拍计数。
                if (--budget == 0U)
                {
                    budget = 3U;
                    osDelay(1U);
                }
            } while (BSP_BMI088.BMI088_Gyro.Get_Queue_Depth() != 0U);
        }
    }
}
