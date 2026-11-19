#include "cmsis_os2.h"
#include "sys_debug.h"
#include "sys_attitude.h"

volatile uint32_t usb_output_overrun_count = 0;

extern "C" void USB_Output_Task(void *argument)
{
    (void)argument;

    uint32_t wake = osKernelGetTickCount();

    constexpr float rad_to_deg = 57.29577951f;

    for (;;)
    {
        // 整帧读取；姿态未就绪或样本过期时不输出。
        Struct_Attitude attitude = {};
        // 回执不依赖姿态有效；本轮有文本时让波形让位。
        if (!USB_Send_Pending() && Attitude_Get_Snapshot(&attitude))
        {
            // 第 1～3 项：VQF 解算的 Roll、Pitch、Yaw，单位度。
            // 第 4～6 项：安装机体系 X、Y、Z 角速度，未扣零偏，rad/s 转为度/秒。
            USB_Printf("data:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                       (double)attitude.Roll,
                       (double)attitude.Pitch,
                       (double)attitude.Yaw,
                       (double)(attitude.Gyro[0] * rad_to_deg),
                       (double)(attitude.Gyro[1] * rad_to_deg),
                       (double)(attitude.Gyro[2] * rad_to_deg));
        }

        // 每 5ms 调度一次；USB 忙时丢包，输出最高 200Hz。
        wake += 5U;
        const uint32_t now_tick = osKernelGetTickCount();

        if ((int32_t)(now_tick - wake) >= 0)
        {
            // 统计跳过的 5ms 输出周期数
            usb_output_overrun_count +=
                (now_tick - wake) / 5U + 1U;

            wake = now_tick + 5U;
        }

        osDelayUntil(wake);
    }
}
