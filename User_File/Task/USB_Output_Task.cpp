#include "cmsis_os2.h"
#include "dji_motor.h"
#include "sys_debug.h"
#include "sys_timestamp.h"

volatile uint32_t usb_output_overrun_count = 0;

extern "C" void USB_Output_Task(void *argument)
{
    (void)argument;

    uint32_t wake = osKernelGetTickCount();
    const Class_DJI_Motor *motor = DJI_Motor_Get(DJI_MOTOR_3, DJI_MOTOR_TYPE_M3508);

    for (;;)
    {
        // 调参输出：只发送 M3508 ID 3 的实际输出轴 rpm，FireWater 单通道。
        // 未收到反馈或反馈超时就不输出，避免把旧转速当作当前测量值。
        if (motor != nullptr)
        {
            Struct_DJI_Motor_Feedback feedback = {};
            if (motor->Get_Feedback_Snapshot(feedback) &&
                (Sys_Get_Micros() - feedback.Last_Rx_Time_Us) <= DJI_FEEDBACK_TIMEOUT_US)
            {
                USB_Printf("data:%.2f", (double)feedback.Measure.Speed_Rpm);
            }
        }

        // 每 20ms 调度一次；忙时丢包，转速输出最高 50Hz。
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
