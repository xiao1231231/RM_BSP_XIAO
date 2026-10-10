#include "cmsis_os2.h"
#include "sys_attitude.h"
#include "sys_debug.h"

volatile uint32_t usb_output_overrun_count = 0;

extern "C" void USB_Output_Task(void *argument)
{
    (void)argument;

    uint32_t wake = osKernelGetTickCount();

    for (;;)
    {
        // 优先尝试发送标定回执，没有待发回执时发送姿态波形。
        if (!USB_Send_Pending())
        {
            Struct_Attitude sample = {};
            (void)Attitude_Get_Snapshot(&sample);

            USB_Printf("imu:%.2f,%.2f,%.2f",
                       (double)sample.Roll,
                       (double)sample.Pitch,
                       (double)sample.Yaw);
        }

        // 每 20ms 调度一次；回执优先、忙时丢包，姿态波形最高 50Hz。
        wake += 20U;
        const uint32_t now_tick = osKernelGetTickCount();

        if ((int32_t)(now_tick - wake) >= 0)
        {
            // 统计跳过的 20ms 输出周期数
            usb_output_overrun_count +=
                (now_tick - wake) / 20U + 1U;

            wake = now_tick + 20U;
        }

        osDelayUntil(wake);
    }
}
