#include "cmsis_os2.h"
#include "key.h"
#include "sys_attitude.h"

volatile uint32_t key_task_overrun_count = 0;

extern "C" void Key_Task(void *argument)
{
    (void)argument;

    uint32_t wake = osKernelGetTickCount();

    for (;;)
    {
        // 1ms 扫描：消抖和长按检测
        Key_Service();

        // 长按 4 秒，提交一次零偏标定请求
        if (Key_Get_LongPress())
        {
            Attitude_Calibration_Request();
        }

        wake += 1U;
        const uint32_t now_tick = osKernelGetTickCount();

        if ((int32_t)(now_tick - wake) >= 0)
        {
            key_task_overrun_count += (now_tick - wake) + 1U;
            wake = now_tick + 1U;
        }

        osDelayUntil(wake);
    }
}