#include "cmsis_os2.h"
#include "led.h"
#include "sys_attitude.h"

extern "C" void LED_Task(void *argument)
{
    (void)argument;

    uint32_t heartbeat_start = osKernelGetTickCount();
    Enum_Attitude_Calibration_Status last_status = ATTITUDE_CAL_IDLE;

    for (;;)
    {
        Struct_Attitude_Calibration_Status cal = {};
        Attitude_Calibration_Get_Status(&cal);

        const uint32_t now = osKernelGetTickCount();
        const uint32_t elapsed = now - cal.Start_Tick;

        // 标定结束，重新开始绿灯心跳
        if (cal.Status == ATTITUDE_CAL_IDLE &&
            last_status != ATTITUDE_CAL_IDLE)
        {
            heartbeat_start = now;
        }

        switch (cal.Status)
        {
        case ATTITUDE_CAL_IDLE:
            // 绿灯：500ms 灭、500ms 亮
            if (((now - heartbeat_start) / 500U) % 2U != 0U)
            {
                LED_Green();
            }
            else
            {
                LED_Off();
            }
            break;

        case ATTITUDE_CAL_RUNNING:
            // 蓝灯：250ms 灭、250ms 亮
            if ((elapsed / 250U) % 2U != 0U)
            {
                LED_Blue();
            }
            else
            {
                LED_Off();
            }
            break;

        case ATTITUDE_CAL_SUCCESS:
            // 红灯：125ms 亮、125ms 灭，共 4 次
            if (elapsed < 1000U &&
                (elapsed / 125U) % 2U == 0U)
            {
                LED_Red();
            }
            else
            {
                LED_Off();
            }
            break;

        case ATTITUDE_CAL_FAILED:
            // 红灯长亮 2 秒
            if (elapsed < 2000U)
            {
                LED_Red();
            }
            else
            {
                LED_Off();
            }
            break;

        default:
            LED_Off();
            break;
        }

        last_status = cal.Status;

        // 灯光不需要 1ms 精度
        osDelay(5U);
    }
}