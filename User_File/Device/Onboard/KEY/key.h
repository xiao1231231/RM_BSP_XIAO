/**
 * @file    key.h
 * @brief   C 板用户按键（PA0，CubeMX 标签 KEY）—— 消抖 + 长按检测
 *
 * @note    PA0 配置为【上拉输入、按键接地】：按下 = 低电平
 *          （Core/Src/gpio.c 的 GPIO_PULLUP）。本模块 1kHz 扫描消抖，
 *          并把"按住 KEY_LONG_PRESS_MS"识别为一次长按事件。
 */

#ifndef KEY_H
#define KEY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 长按判定阈值：按住这么久算一次"长按" */
#define KEY_LONG_PRESS_MS   4000U
/** 消抖时间：原始电平稳定这么久才确认 */
#define KEY_DEBOUNCE_MS     20U

/** 初始化（GPIO 由 CubeMX 生成代码配好，这里无额外动作，保留统一入口） */
void Key_Init(void);

/** 按键扫描 + 长按检测，由 Key_Task 每 1ms 调用。
 *  长按一次性触发：按住不放不会重复，松开后重新按才算下一次 */
void Key_Service(void);

/** 取走"长按"事件：true = 自上次取走后发生过一次长按（查询即清零） */
bool Key_Get_LongPress(void);

#ifdef __cplusplus
}
#endif

#endif /* KEY_H */
