#include "cmsis_os2.h"
#include "dji_motor.h"
#include "sys_timestamp.h"

// 调试器观察：累计错过的节拍数、实际运行周期（秒）
volatile uint32_t motor_task_overrun_count = 0;
volatile float motor_task_period_s = 0.0f;

extern "C" void Motor_Task(void *argument)
{
    (void)argument;

    uint32_t wake = osKernelGetTickCount();
    uint32_t last_cycle = Sys_Get_Cycle();



    for (;;)
    {
        motor_task_period_s = Sys_Get_DeltaTime(&last_cycle);

        // 各电机速度闭环计算，并发送分组电流指令
        DJI_Motor_Control_Task();

        
        // 推进到下一个 1ms 节拍
        wake += 1U;
        const uint32_t now_tick = osKernelGetTickCount();

        // 已错过唤醒时刻：记录丢拍，重新定位到未来节拍
        if ((int32_t)(now_tick - wake) >= 0)
        {
            motor_task_overrun_count += (now_tick - wake) + 1U;
            wake = now_tick + 1U;
        }

        osDelayUntil(wake);
    }
}
