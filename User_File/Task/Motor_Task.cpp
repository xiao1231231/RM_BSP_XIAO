#include "cmsis_os2.h"
#include "Motor_Task.h"
#include "can.h"
#include "dji_motor.h"
#include "dm4310.h"
#include "sys_timestamp.h"
#include "chassis.h"
#include "gimbal.h"
#include "gimbal_config.h"
#include "gimbal_remote.h"
#include "dt7.h"

// 调试器观察：累计错过的节拍数、相邻两次控制完成的间隔（秒）
volatile uint32_t motor_task_overrun_count = 0;
volatile float motor_task_period_s = 0.0f;

static volatile bool motor_control_idle = false;
static volatile bool maintenance_requested = false;
static volatile bool maintenance_ready = false;

bool Motor_Control_Is_Idle()
{
    return motor_control_idle;
}

void Motor_Maintenance_Request()
{
    maintenance_ready = false;
    __DMB();
    maintenance_requested = true;
}

bool Motor_Maintenance_Is_Ready()
{
    return maintenance_ready;
}

void Motor_Maintenance_Release()
{
    maintenance_requested = false;
    maintenance_ready = false;
}

// 只有本任务发送电机帧。先等两路 CAN 的旧邮箱排空，再确认完整停机批次的 TXOK。
static void Motor_Maintenance_Service()
{
    static bool waiting = false;
    static bool all_submitted = false;
    static uint32_t complete_before = 0U;
    static uint32_t submitted = 0U;

    if (!maintenance_requested)
    {
        waiting = false;
        return;
    }
    if (maintenance_ready ||
        HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3U ||
        HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) != 3U)
    {
        return;
    }
    if (waiting && all_submitted &&
        can_tx_complete_count - complete_before == submitted &&
        !DM4310_Motor_Any_Control_Active())
    {
        maintenance_ready = true;
        return;
    }

    complete_before = can_tx_complete_count;
    const uint32_t submit_before = can_tx_submit_count;
    const bool dji_sent = DJI_Motor_Control_Task(true);
    const bool dm_sent = DM4310_Motor_Control_Task(true);
    submitted = can_tx_submit_count - submit_before;
    all_submitted = dji_sent && dm_sent;
    waiting = true;
}

extern "C" void Motor_Task(void *argument)
{
    (void)argument;

    uint32_t wake = osKernelGetTickCount();
    uint32_t last_cycle = Sys_Get_Cycle();

    // 获取已经创建好的4个3508电机，motor[0~3] 是指向它们的指针
    Class_DJI_Motor *motors[4] = {
        DJI_Motor_Get(DJI_MOTOR_1),
        DJI_Motor_Get(DJI_MOTOR_2),
        DJI_Motor_Get(DJI_MOTOR_3),
        DJI_Motor_Get(DJI_MOTOR_4)
    };
    //定义target_rpm数组来存储四个电机目标转速
    float target_rpm[4] = {0};

    // GM6020 独立于 3508：数组下标 0/1 对应 GM6020 的拨码 ID 5/6。
    Class_DJI_Motor *gm6020_motors[2] = {
        DJI_Motor_Get(DJI_MOTOR_5, DJI_MOTOR_TYPE_GM6020),
        DJI_Motor_Get(DJI_MOTOR_6, DJI_MOTOR_TYPE_GM6020)
    };
    float gm6020_target_rpm[2] = {0.0f, 0.0f};

    // DM4310：数组下标 0 对应 CAN ID 0x05，与 system_init.cpp 的创建配置一致。
    Class_DM4310 *dm4310_motors[1] = {
        DM4310_Motor_Get(0x05U)
    };
    float dm4310_target_rpm[1] = {150.0f};

    // 进入循环前初始化一次
    Gimbal_Init();

    for (;;)
    {
        Motor_Maintenance_Service();
        if (!maintenance_requested)
        {
            // 每个控制周期先更新四个电机的目标转速，单位为输出轴 rpm。
            Chassis_Update(target_rpm);
            for (uint8_t i = 0; i < 4U; ++i)
            {
                if (motors[i] != nullptr)
                {
                    motors[i]->Set_Target_Speed_Rpm(target_rpm[i]);
                }
            }
            // 云台使用的 GM6020 由 Gimbal_Update 管理，其余继续使用速度控制。
            for (uint8_t i = 0; i < 2U; ++i)
            {
                if (gm6020_motors[i] != nullptr &&
                    gm6020_motors[i]->Get_Motor_Id() != Gimbal_Config::Motor_Id)
                {
                    gm6020_motors[i]->Set_Target_Speed_Rpm(gm6020_target_rpm[i]);
                }
            }

            // DM4310 使用主控速度 PID，单位 rpm；周期控制自动使能，零速为闭环制动。
            for (uint8_t i = 0; i < 1U; ++i)
            {
                if (dm4310_motors[i] != nullptr)
                {
                    dm4310_motors[i]->Set_Target_Speed_Rpm(dm4310_target_rpm[i]);
                }
            }

            // 左开关中档接管云台；回中、离开中档或遥控超时后保持当前位置目标。
            Struct_DT7_Data remote = {};
            const float yaw_rate = DT7_Get_Snapshot(&remote)
                                   ? Gimbal_Remote_Map_Yaw_Rate_Radps(remote)
                                   : 0.0f;
            Gimbal_Set_Target_Angular_Velocity_Radps(yaw_rate);
            Gimbal_Update();

            // 按各电机的控制模式计算输出并统一发送。
            (void)DJI_Motor_Control_Task();
            (void)DM4310_Motor_Control_Task();

            motor_control_idle = !DJI_Motor_Any_Control_Active() &&
                                 !DM4310_Motor_Any_Control_Active();
        }

        motor_task_period_s = Sys_Get_DeltaTime(&last_cycle);
        // 推进到下一个 1ms 节拍
        wake += 1U;
        const uint32_t now_tick = osKernelGetTickCount();

        // 已错过唤醒时刻：记录丢拍，重新定位到未来节拍
        if ((int32_t)(now_tick - wake) >= 0)
        {
            motor_task_overrun_count += (now_tick - wake) + 1U;
            wake = now_tick + 1U;
        }

        osDelayUntil(wake);
    }
}
