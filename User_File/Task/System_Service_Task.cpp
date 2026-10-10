#include "cmsis_os2.h"
#include "can.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "sys_timestamp.h"

volatile uint32_t system_service_overrun_count = 0;

extern "C" void System_Service_Task(void *argument)
{
    (void)argument;

    uint32_t wake = osKernelGetTickCount();

    for (;;)
    {
        // 保持 DWT 的 64 位时间累加正常运行
        (void)Sys_Get_Micros();

        // UART 接收错误后的恢复
        BSP_UART_Recover_PeriodElapsedCallback();

        // CAN 总线状态巡检
        BSP_CAN_Service_PeriodElapsedCallback(&hcan1);

        wake += 1U;
        const uint32_t now_tick = osKernelGetTickCount();

        if ((int32_t)(now_tick - wake) >= 0)
        {
            system_service_overrun_count += (now_tick - wake) + 1U;
            wake = now_tick + 1U;
        }

        osDelayUntil(wake);
    }
}