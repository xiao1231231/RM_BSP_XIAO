#include "cmsis_os2.h"
#include "sys_attitude.h"

volatile uint32_t calibration_task_overrun_count = 0;

extern "C" void Calibration_Task(void *argument)
{
    (void)argument;

    uint32_t wake = osKernelGetTickCount();

    for (;;)
    {
        // 推进标定：静置、采样、检查、保存，结果提交到 USB。
        Attitude_Calibration_Service();

        wake += 1U;
        const uint32_t now_tick = osKernelGetTickCount();

        if ((int32_t)(now_tick - wake) >= 0)
        {
            calibration_task_overrun_count +=
                (now_tick - wake) + 1U;

            wake = now_tick + 1U;
        }

        osDelayUntil(wake);
    }
}
