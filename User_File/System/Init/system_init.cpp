//
// Created by xiao on 2026/9/25.
//
#include "system_init.h"

#include "usart.h"
#include "sys_timestamp.h"
#include "bsp_uart.h"
#include "led.h"
#include "buzzer.h"
#include "sys_attitude.h"

void System_Init(void)
{
    Sys_Timestamp_Init();

    LED_Init();
    Buzzer_Init();

    UART_Init(&huart1, NULL);

    /* 姿态解算（SPI 层 + BMI088 + VQF）。放在调度器启动前 ——
     * 里面是阻塞的：配置逐步读回校验（失败重试）+ 开机零偏标定 1 秒。
     * ⚠️ 这段时间板子必须静止放好 —— 零偏标定要求静止，
     *    否则标出来的就是"运动速度"，喂给滤波器反而更糟。 */
    Attitude_Init();
}
