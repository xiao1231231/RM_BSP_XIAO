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
#include "dm4310.h"
#include "dt7.h"
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

    // USART3：DT7/DR16，连接 C 板专用 DBUS 接口
    DT7_Init(&huart3);

    /* USB 虚拟串口（CDC）：发送走 USB_Transmit；接收回调先给占位计数
     * （usb_frame_count，验证收链路活着），接上位协议时替换。
     * 调度器启动前在这里显式调用 USB_Init，初始化不依赖生成任务的弱入口。 */
    USB_Init(USB_Frame_Callback);

    /* CAN1：PD0/PD1；CAN2：PB5/PB6；均为 1Mbps。
     * 过滤器全收 + FIFO0 收报中断，设备由电机模块注册。
     * ★ CAN 起不来 = 总线上电机不可用，所以失败必须能听见（长鸣一声）——
     *   否则现象是"电机没反应"，排查会先去怀疑电调和接线。 */
    const bool can1_ready = CAN_Init(&hcan1);
    const bool can2_ready = CAN_Init(&hcan2);
    if (!can1_ready || !can2_ready)
    {
        Buzzer_Beep(2000.0f, 0.5f);
    }

    /* DJI 3508 ×4：一个函数 + 编号枚举建齐，PID 参数集中在这里调。
     * ⚠️ 电调自身的 ID 必须和编号一致（上电"滴"声次数 = 当前编号），
     *    这块板上的电调实测是 3 号；其余电调接上后同样要先核对编号。 */
    // 3508、GM6020 共用此总线；创建电机前选择 &hcan1 或 &hcan2。
    DJI_Motor_Init(&hcan1);
    DJI_Motor_Create(DJI_MOTOR_1, 38.0f, 15.0f, 0.0f);
    DJI_Motor_Create(DJI_MOTOR_2, 25.0f, 16.0f, 0.0f);
    DJI_Motor_Create(DJI_MOTOR_3, 20.0f, 13.0f, 0.0f);
    DJI_Motor_Create(DJI_MOTOR_4, 20.0f, 15.0f, 0.0f);

    /* GM6020 ×2，拨码 ID 为 5、6，使用电机内部电流环。
     * gimbal_config.h 的 Motor_Id 选中的电机由 Gimbal_Update 设置电流目标。
     * 另一台由 Motor_Task 设置速度目标；速度 PID 参数在这里配置。
     * 固件 >= 1.0.11.2，用 RoboMaster Assistant >= 2.7 开启电流环开关。 */
    Class_DJI_Motor *gm6020_5 = DJI_Motor_Create(
        DJI_MOTOR_5, 15.0f, 0.0f, 0.0f, DJI_MOTOR_TYPE_GM6020);
    Class_DJI_Motor *gm6020_6 = DJI_Motor_Create(
        DJI_MOTOR_6, 0.0f, 0.0f, 0.0f, DJI_MOTOR_TYPE_GM6020);
    if (gm6020_5 == nullptr || gm6020_6 == nullptr)
    {
        Buzzer_Beep(2000.0f, 0.5f);
    }

    // DM4310：映射范围必须与上位机一致；最后三项为主控速度 PID 的 Kp、Ki、Kd。
    // PID 输入 rpm、输出 N·m，1kHz 运行；MIT 内置 Kp/Kd 置零，只发送扭矩。
    // PID 输出量纲与 DJI 电流指令不同，增益数值需分别上板调整。
    // 初始化只保存配置和注册反馈；设置速度目标后，周期控制自动使能。
    // DM4310 的总线独立选择，可与 DJI 共用，也可使用另一路 CAN。
    DM4310_Motor_Init(&hcan1);
    if (DM4310_Motor_Create(0x05U, 0x15U,
                            12.5f, 30.0f, 10.0f,
                            0.05f, 0.0f, 0.0f) == nullptr)
    {
        Buzzer_Beep(2000.0f, 0.5f);
    }

    /* 姿态解算（SPI 层 + BMI088 + VQF）。放在调度器启动前 ——
     * 配置逐步读回校验并有限重试，初始化会阻塞。
     * 零偏优先加载 Flash；仅加载失败时执行约 1 秒开机静止标定，
     * 走开机标定分支时须保持板子静止。 */
    Attitude_Init();
}
