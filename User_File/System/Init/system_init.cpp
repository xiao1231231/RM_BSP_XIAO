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
#include "key.h"
#include "buzzer.h"
#include "sys_attitude.h"
#include "usb_device.h"
#include "bsp_usb.h"

void System_Init(void)
{
    Sys_Timestamp_Init();

    LED_Init();
    Buzzer_Init();
    Key_Init();

    /* USART1：调试串口（波形输出，只发不收 → 回调给 NULL） */
    UART_Init(&huart1, NULL);

    /* USART6：3-pin UART 接口（PG14/PG9），目前还没接外设。
     * 回调给一个只计数的占位实现（callback.cpp），收链路活着即可；
     * 接了真实外设（上位机/视觉）后把解析逻辑写进去。 */
    UART_Init(&huart6, USART6_Frame_Callback);

    /* USB 虚拟串口（CDC）：发送走 USB_Transmit；接收回调先给占位计数
     * （usb_frame_count，验证收链路活着），接上位协议时替换。
     * ★ CubeMX 把 MX_USB_DEVICE_Init 放进了 __weak TIM_1ms_Task 空壳，
     *   本工程任务是强实现覆盖那个壳 —— 那里永远不会执行，
     *   所以初始化必须显式调（现在收在 bsp_usb 的 USB_Init 里）。 */
    USB_Init(USB_Frame_Callback);

    /* CAN1：PD0/PD1，1Mbps，过滤器全收 + FIFO0 收报中断（设备由电机模块注册）。
     * ★ CAN 起不来 = 四个电机全部不可用，所以失败必须能听见（长鸣一声）——
     *   否则现象是"电机没反应"，排查会先去怀疑电调和接线。 */
    if (!CAN_Init(&hcan1))
    {
        Buzzer_Beep(2000.0f, 0.5f);
    }

    /* DJI 3508 ×4：一个函数 + 编号枚举建齐，PID 参数集中在这里调。
     * ⚠️ 电调自身的 ID 必须和编号一致（上电"滴"声次数 = 当前编号），
     *    这块板上的电调实测是 3 号；其余电调接上后同样要先核对编号。 */
    DJI_Motor_Init(&hcan1);
    DJI_Motor_Create(DJI_MOTOR_1, 15.0f, 2.0f, 0.0f);
    DJI_Motor_Create(DJI_MOTOR_2, 15.0f, 2.0f, 0.0f);
    DJI_Motor_Create(DJI_MOTOR_3, 15.0f, 2.0f, 0.0f);
    DJI_Motor_Create(DJI_MOTOR_4, 15.0f, 2.0f, 0.0f);
    DJI_Motor_Get(DJI_MOTOR_3)->Set_Target_Speed_Rpm(0.00f);   //测试用，记得删
    /* 姿态解算（SPI 层 + BMI088 + VQF）。放在调度器启动前 ——
     * 里面是阻塞的：配置逐步读回校验（失败重试）+ 开机零偏标定 1 秒。
     * ⚠️ 这段时间板子必须静止放好 —— 零偏标定要求静止，
     *    否则标出来的就是"运动速度"，喂给滤波器反而更糟。 */
    Attitude_Init();
}
