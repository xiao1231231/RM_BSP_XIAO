/**
 * @file    key.cpp
 * @brief   用户按键实现（消抖 + 长按检测，见 .h）
 *
 * @note    消抖策略：原始电平必须连续稳定 KEY_DEBOUNCE_MS 才确认切换。
 *          长按策略：按下沿开始计时，按满 KEY_LONG_PRESS_MS 触发一次；
 *          提前松开 = 放弃（普通短按，不产生事件）。
 *          计时用 RTOS 毫秒节拍（本模块时序都是毫秒级，不需要微秒精度，
 *          且 osKernelGetTickCount 是纯变量读取，比 Sys_Get_Micros 便宜）。
 */

#include "key.h"

#include "main.h"       /* KEY_Pin / KEY_GPIO_Port —— CubeMX 标签 */
#include "cmsis_os2.h"  /* osKernelGetTickCount */

/* PA0 上拉输入、按键接地：按下 = 低电平 */
static inline bool Key_Is_Pressed(void)
{
    return HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET;
}

static bool     s_Raw_Last = false;         /* 上一拍的原始电平 */
static uint32_t s_Raw_Stable_Since = 0U;    /* 原始电平最近一次变化的时刻 */
static bool     s_Stable_Pressed = false;   /* 消抖后的稳定状态 */
static uint32_t s_Pressed_Since = 0U;       /* 确认按下的时刻（长按计时起点） */
static bool     s_LongPress_Fired = false;  /* 本次按住是否已触发过 */
static volatile bool s_LongPress_Event = false;

void Key_Init(void)
{
    /* GPIO（上拉输入）已由 CubeMX 生成的 MX_GPIO_Init() 配置，无额外动作 */
}

void Key_Service(void)
{
    const bool raw = Key_Is_Pressed();
    const uint32_t now_ms = osKernelGetTickCount();

    if (raw != s_Raw_Last)
    {
        /* 原始电平变了：重新起算稳定窗口（机械抖动的每一跳都刷新它） */
        s_Raw_Last = raw;
        s_Raw_Stable_Since = now_ms;
        return;
    }
    if (now_ms - s_Raw_Stable_Since < KEY_DEBOUNCE_MS)
    {
        return;                             /* 电平还没稳够消抖时间 */
    }

    if (raw != s_Stable_Pressed)
    {
        /* 确认一次状态切换 */
        s_Stable_Pressed = raw;
        if (raw)
        {
            s_Pressed_Since = now_ms;       /* 按下沿：长按计时开始 */
            s_LongPress_Fired = false;
        }
        return;
    }

    if (s_Stable_Pressed && !s_LongPress_Fired &&
        now_ms - s_Pressed_Since >= KEY_LONG_PRESS_MS)
    {
        s_LongPress_Fired = true;           /* 按住不放不重复触发 */
        s_LongPress_Event = true;
    }
}

bool Key_Get_LongPress(void)
{
    const bool event = s_LongPress_Event;
    s_LongPress_Event = false;
    return event;
}
