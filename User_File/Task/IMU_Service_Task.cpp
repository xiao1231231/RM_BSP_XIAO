#include "cmsis_os2.h"
#include "bsp_bmi088.h"

volatile uint32_t imu_service_overrun_count = 0;

extern "C" void IMU_Service_Task(void *argument)
{
    (void)argument;

    uint32_t wake = osKernelGetTickCount();
    uint32_t heat_div = 0;
    uint32_t temperature_div = 0;

    for (;;)
    {
        const uint32_t now_tick = osKernelGetTickCount();

        // 到达周期时间，才执行下面的定时工作
        if ((int32_t)(now_tick - wake) >= 0)
        {
            // 1ms：通信调度、兜底轮询和超时恢复
            BMI088_TIM_1ms_Service_PeriodElapsedCallback();

            // 2ms：恒温控制
            if (++heat_div >= 2U)
            {
                heat_div = 0;
                BSP_BMI088.Heater_Control();
            }

            // 128ms：请求读取温度
            if (++temperature_div >= 128U)
            {
                temperature_div = 0;
                BMI088_TIM_128ms_Calculate_PeriodElapsedCallback();
            }

            wake += 1U;

            const uint32_t after_work = osKernelGetTickCount();

            if ((int32_t)(after_work - wake) >= 0)
            {
                imu_service_overrun_count +=
                    (after_work - wake) + 1U;

                wake = after_work + 1U;
            }
        }

        // 计算距离下次周期还有多少 tick
        const uint32_t before_wait = osKernelGetTickCount();
        const int32_t remaining = (int32_t)(wake - before_wait);

        if (remaining <= 0)
        {
            continue;
        }

        // 有续传通知就提前醒来，否则等到下次周期
        const uint32_t flags = osThreadFlagsWait(
            0x0002U,
            osFlagsWaitAny,
            (uint32_t)remaining
        );

        // 超时也是正常返回，先排除错误码再检查通知位
        if ((flags & osFlagsError) == 0U &&
            (flags & 0x0002U) != 0U)
        {
            BSP_BMI088.BMI088_Service_Transfer(true);
        }
    }
}