/**
* @file    dm4310.h
 * @brief   DM-J4310P-2EC 电机 MIT 通信接口
 */

#ifndef DM4310_H
#define DM4310_H

#include <stdint.h>
#include "bsp_can.h"
#include "alg_pid.h"

// 支持电机 ID 1～15，静态对象池使用 ID-1 作为下标。
#define DM4310_MOTOR_CNT 15U

// 沿用现有 DM 反馈过期阈值；过期后速度环输出零扭矩并清 PID 历史。
#define DM4310_FEEDBACK_TIMEOUT_US 50000U

enum Enum_DM4310_Control_Mode
{
    DM4310_CONTROL_MIT = 0,
    DM4310_CONTROL_SPEED,
};

// 一帧 MIT 控制命令，均使用物理单位。
struct Struct_DM4310_MIT_Command
{
    float Position_Rad;      // 目标位置，rad。
    float Velocity_Radps;    // 目标角速度，rad/s。
    float K_P;               // 位置增益，范围 0～500。
    float K_D;               // 速度增益，范围 0～5。
    float Torque_Nm;         // 前馈扭矩，N·m。
};

// 电机最近一帧上报的测量值。
struct Struct_DM4310_Measure
{
    float Position_Rad;      // 协议反馈位置，rad。
    float Velocity_Radps;    // 协议反馈角速度，rad/s。
    float Torque_Nm;         // 协议反馈扭矩，N·m。
    uint8_t State;           // 0=失能，1=使能，其他值按手册解释。
    uint8_t Temperature_Mos;
    uint8_t Temperature_Rotor;
};

// 完整反馈快照，字段来自同一次 CAN 接收。
struct Struct_DM4310_Feedback
{
    Struct_DM4310_Measure Measure;
    uint64_t Last_Rx_Time_Us = 0U;
    uint32_t Sequence = 0U;
    bool Received = false;
};


class Class_DM4310
{
public:
    // 调度器启动前初始化一次；映射范围必须与电机设置一致。
    bool Init(CAN_HandleTypeDef *hcan,
              uint8_t Motor_Id, uint16_t Master_Id,
              float P_Max, float V_Max, float T_Max,
              float K_P, float K_I, float K_D);

    // 在控制任务中设置目标，只更新缓存，不发送或使能电机。
    inline void Set_Target_MIT(const Struct_DM4310_MIT_Command &Command)
    {
        if (Control_Mode != DM4310_CONTROL_MIT)
        {
            Speed_PID.Reset();
        }

        Control_Mode = DM4310_CONTROL_MIT;
        Target_MIT = Command;
        Target_Speed_Rpm = 0.0f;
        Out_Torque_Nm = 0.0f;
    }

    // 设置目标转速并选择主控速度 PID；单位 rpm，与反馈角速度对应同一轴。
    // 由 Motor_Task 调用；只更新缓存，周期 Control 自动使能后再进入速度闭环。
    inline void Set_Target_Speed_Rpm(float rpm)
    {
        if (Control_Mode != DM4310_CONTROL_SPEED)
        {
            Speed_PID.Reset();
        }

        Control_Mode = DM4310_CONTROL_SPEED;
        Target_Speed_Rpm = rpm;
        Target_MIT = {};
    }

    inline const Struct_DM4310_MIT_Command &Get_Target_MIT() const { return Target_MIT; }
    inline float Get_Target_Speed_Rpm() const { return Target_Speed_Rpm; }
    // 速度 PID 最近计算出的扭矩，N·m；直接 MIT 模式下为零。
    inline float Get_Out_Torque_Nm() const { return Out_Torque_Nm; }

    // 以下控制、直接发送及启停接口在运行期由 Motor_Task 串行调用。
    // 维护期间仅执行停机流程，勿从其他任务绕过维护握手直接发帧。
    // 1kHz 调用：速度 PID 输出扭矩后经 MIT 发送，或直接发送 MIT 目标。
    bool Control();

    // 直接发送一帧 MIT 命令，不修改周期发送使用的目标缓存。
    bool Transmit_MIT(const Struct_DM4310_MIT_Command &Command);

    // 发送协议命令；返回值仅表示本次 CAN 提交是否成功。
    bool Enable();

    // 清目标、退出速度模式并发送失能指令；提交失败时由调用方重试。
    // 再次设置速度目标才会恢复自动使能；零速目标仍表示闭环制动。
    bool Disable();
    bool Clear_Error();

    // CAN 接收回调调用：解析并发布完整反馈。
    void Feedback_Parse(uint8_t *Data, uint16_t Length);

    // 跨任务读取完整反馈；返回是否收到过有效反馈。
    bool Get_Feedback_Snapshot(Struct_DM4310_Feedback &Out) const;

private:
    // 三种特殊命令共用的发送实现。
    bool Transmit_Command(uint8_t Command);

    CAN_HandleTypeDef *CAN_Handler = nullptr;
    uint8_t Motor_Id = 0U;

    float P_Max = 0.0f;
    float V_Max = 0.0f;
    float T_Max = 0.0f;

    Class_PID Speed_PID;
    Enum_DM4310_Control_Mode Control_Mode = DM4310_CONTROL_MIT;
    float Target_Speed_Rpm = 0.0f;
    float Out_Torque_Nm = 0.0f;

    Struct_DM4310_Feedback Feedback = {};
    Struct_DM4310_MIT_Command Target_MIT = {};
};

// 单路 CAN 管理：调度器启动前绑定一次，不负责启动 CAN 外设。
void DM4310_Motor_Init(CAN_HandleTypeDef *hcan);

// 创建电机并注册反馈；ID 非法、重复创建或初始化失败返回 nullptr。
// 创建只完成初始化，不发送使能指令。
// K_P/K_I/K_D 是主控速度 PID 参数：输入 rpm，输出 N·m，周期 1ms。
// 积分和输出均限幅 ±T_Max；这些参数不写入电机掉电保存配置。
Class_DM4310 *DM4310_Motor_Create(
    uint8_t Motor_Id, uint16_t Master_Id,
    float P_Max, float V_Max, float T_Max,
    float K_P, float K_I, float K_D);

// 获取已经创建的电机；ID 非法或尚未创建返回 nullptr。
Class_DM4310 *DM4310_Motor_Get(uint8_t Motor_Id);

// 控制任务按周期调用：速度模式自动使能，直接 MIT 模式按目标发送。
// 任一电机提交失败时返回 false，仍会尝试发送其余电机。
// Stop=true 时清目标并逐台发送失能命令。
bool DM4310_Motor_Control_Task(bool Stop = false);

// 仅由控制任务调用：有非零目标/输出、使能/故障反馈或过期反馈即返回 true。
bool DM4310_Motor_Any_Control_Active();

#endif /* DM4310_H */
