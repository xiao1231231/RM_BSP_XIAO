#include "cmsis_os2.h"
#include "led.h"

extern "C" void LED_Task(void *argument)
{
    (void)argument;

    for (;;)
    {
        // 绿灯心跳：500ms 灭、500ms 亮（RTOS tick 为 1ms）。
        LED_Off();
        osDelay(500U);
        LED_Green();
        osDelay(500U);
    }
}
