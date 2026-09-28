//
// Created by xiao on 2026/9/25.
//
#include "system_init.h"

#include "usart.h"
#include "can.h"
#include "sys_timestamp.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "dji_motor.h"
#include "callback.h"
#include "led.h"
#include "buzzer.h"
#include "sys_attitude.h"

void System_Init(void)
{
    Sys_Timestamp_Init();

    LED_Init();
    Buzzer_Init();

    /* USART1：调试串口（波形输出，只发不收 → 回调给 NULL） */
    UART_Init(&huart1, NULL);

    /* USART6：3-pin UART 接口（PG14/PG9），目前还没接外设。
     * 回调给一个只计数的占位实现（callback.cpp），收链路活着即可；
     * 接了真实外设（上位机/视觉）后把解析逻辑写进去。 */
    UART_Init(&huart6, USART6_Frame_Callback);

    /* CAN1：PD0/PD1，1Mbps，过滤器全收 + FIFO0 收报中断（设备由电机模块注册） */
    CAN_Init(&hcan1);

    /* DJI 3508 ×4：一个函数 + 编号枚举建齐，PID 参数集中在这里调。
     * ⚠️ 电调自身的 ID 必须和编号一致（上电"滴"声次数 = 当前编号），
     *    这块板上的电调实测是 3 号；其余电调接上后同样要先核对编号。 */
    DJI_Motor_Init(&hcan1);
    DJI_Motor_Create(DJI_MOTOR_1, 15.0f, 2.0f, 0.0f);
    DJI_Motor_Create(DJI_MOTOR_2, 15.0f, 2.0f, 0.0f);
    DJI_Motor_Create(DJI_MOTOR_3, 15.0f, 2.0f, 0.0f);
    DJI_Motor_Create(DJI_MOTOR_4, 15.0f, 2.0f, 0.0f);
    /* 姿态解算（SPI 层 + BMI088 + VQF）。放在调度器启动前 ——
     * 里面是阻塞的：配置逐步读回校验（失败重试）+ 开机零偏标定 1 秒。
     * ⚠️ 这段时间板子必须静止放好 —— 零偏标定要求静止，
     *    否则标出来的就是"运动速度"，喂给滤波器反而更糟。 */
    Attitude_Init();
}
