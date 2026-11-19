#include "dm4310.h"

#include <cmath>
#include "alg_basic.h"
#include "sys_timestamp.h"

static CAN_HandleTypeDef *motor_can_handle = nullptr;
static Class_DM4310 *motors[DM4310_MOTOR_CNT] = {};

// CAN 层通过上下文指针找到对应的电机对象。
static void DM4310_Feedback_Callback(void *Id,
                                      uint8_t *Data, uint16_t Length)
{
    static_cast<Class_DM4310 *>(Id)->Feedback_Parse(Data, Length);
}

void DM4310_Motor_Init(CAN_HandleTypeDef *hcan)
{
    motor_can_handle = hcan;
}

Class_DM4310 *DM4310_Motor_Create(
    uint8_t Motor_Id, uint16_t Master_Id,
    float P_Max, float V_Max, float T_Max,
    float K_P, float K_I, float K_D)
{
    if (motor_can_handle == nullptr ||
        Motor_Id == 0U || Motor_Id > DM4310_MOTOR_CNT)
    {
        return nullptr;
    }

    if (motors[Motor_Id - 1U] != nullptr)
    {
        return nullptr;
    }

    static Class_DM4310 pool[DM4310_MOTOR_CNT];
    Class_DM4310 *motor = &pool[Motor_Id - 1U];

    // 成员 Init 完成参数检查和 CAN 注册，成功后才发布对象。
    if (!motor->Init(motor_can_handle, Motor_Id, Master_Id,
                     P_Max, V_Max, T_Max, K_P, K_I, K_D))
    {
        return nullptr;
    }

    motors[Motor_Id - 1U] = motor;
    return motor;
}

Class_DM4310 *DM4310_Motor_Get(uint8_t Motor_Id)
{
    if (Motor_Id == 0U || Motor_Id > DM4310_MOTOR_CNT)
    {
        return nullptr;
    }

    return motors[Motor_Id - 1U];
}

bool DM4310_Motor_Control_Task(bool Stop)
{
    if (motor_can_handle == nullptr)
    {
        return false;
    }

    bool all_sent = true;

    for (uint8_t i = 0U; i < DM4310_MOTOR_CNT; ++i)
    {
        if (motors[i] != nullptr &&
            !(Stop ? motors[i]->Disable() : motors[i]->Control()))
        {
            all_sent = false;
        }
    }

    return all_sent;
}

bool DM4310_Motor_Any_Control_Active()
{
    for (uint8_t i = 0U; i < DM4310_MOTOR_CNT; ++i)
    {
        const Class_DM4310 *motor = motors[i];
        if (motor == nullptr) { continue; }
        const auto &target = motor->Get_Target_MIT();
        if (motor->Get_Target_Speed_Rpm() != 0.0f ||
            motor->Get_Out_Torque_Nm() != 0.0f ||
            target.Position_Rad != 0.0f || target.Velocity_Radps != 0.0f ||
            target.K_P != 0.0f || target.K_D != 0.0f || target.Torque_Nm != 0.0f)
        {
            return true;
        }
        Struct_DM4310_Feedback feedback = {};
        if (motor->Get_Feedback_Snapshot(feedback) &&
            (feedback.Measure.State != 0U ||
             Sys_Get_Micros() - feedback.Last_Rx_Time_Us > DM4310_FEEDBACK_TIMEOUT_US))
        {
            return true;
        }
    }
    return false;
}

bool Class_DM4310::Init(CAN_HandleTypeDef *hcan,
                        uint8_t Motor_Id, uint16_t Master_Id,
                        float P_Max, float V_Max, float T_Max,
                        float K_P, float K_I, float K_D)
{
    // 同一个对象只注册一次，避免重复占用 CAN 接收表。
    if (CAN_Handler != nullptr)
    {
        return false;
    }

    // 本驱动将电机 ID 限定为 1～15，避免与反馈状态位冲突。
    if (hcan == nullptr ||
        Motor_Id == 0U || Motor_Id > DM4310_MOTOR_CNT ||
        Master_Id > 0x7FFU)
    {
        return false;
    }

    // 映射范围必须为有限正数，后续编解码要用它们进行换算。
    if (!std::isfinite(P_Max) ||
        !std::isfinite(V_Max) ||
        !std::isfinite(T_Max) ||
        P_Max <= 0.0f || V_Max <= 0.0f || T_Max <= 0.0f)
    {
        return false;
    }

    if (!std::isfinite(K_P) || !std::isfinite(K_I) || !std::isfinite(K_D))
    {
        return false;
    }

    CAN_Handler = hcan;
    this->Motor_Id = Motor_Id;
    this->P_Max = P_Max;
    this->V_Max = V_Max;
    this->T_Max = T_Max;
    Feedback = {};
    Target_MIT = {};
    Control_Mode = DM4310_CONTROL_MIT;
    Target_Speed_Rpm = 0.0f;
    Out_Torque_Nm = 0.0f;

    // 与 DJI 一样在 1kHz 任务运行；此处 PID 输出为扭矩，单位 N·m。
    Speed_PID.Init(K_P, K_I, K_D, 0.0f, T_Max, T_Max, 0.001f);
    Speed_PID.Reset();

    // 所有成员准备好后，再让 CAN 接收中断访问这个对象。
    if (!CAN_Register_Device(hcan, Master_Id, this,
                             DM4310_Feedback_Callback))
    {
        CAN_Handler = nullptr;
        return false;
    }

    return true;
}

void Class_DM4310::Feedback_Parse(uint8_t *Data, uint16_t Length)
{
    if (Data == nullptr || Length != 8U)
    {
        return;
    }

    if ((Data[0] & 0x0FU) != Motor_Id)
    {
        return;
    }

    const uint16_t position_raw =
        static_cast<uint16_t>((Data[1] << 8) | Data[2]);

    const uint16_t velocity_raw =
        static_cast<uint16_t>((Data[3] << 4) | (Data[4] >> 4));

    const uint16_t torque_raw =
        static_cast<uint16_t>(((Data[4] & 0x0FU) << 8) | Data[5]);

    Struct_DM4310_Feedback frame = {};

    frame.Measure.Position_Rad = Basic_Math_Int_To_Float(
        position_raw, 0, 65535, -P_Max, P_Max);

    frame.Measure.Velocity_Radps = Basic_Math_Int_To_Float(
        velocity_raw, 0, 4095, -V_Max, V_Max);

    frame.Measure.Torque_Nm = Basic_Math_Int_To_Float(
        torque_raw, 0, 4095, -T_Max, T_Max);

    frame.Measure.State = Data[0] >> 4;
    frame.Measure.Temperature_Mos = Data[6];
    frame.Measure.Temperature_Rotor = Data[7];

    frame.Last_Rx_Time_Us = Sys_Get_Micros();
    frame.Received = true;

    // 在局部变量中完成解析，短临界区只负责提交整帧。
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    frame.Sequence = Feedback.Sequence + 1U;
    Feedback = frame;

    __DMB();
    __set_PRIMASK(primask);
}

bool Class_DM4310::Get_Feedback_Snapshot(
    Struct_DM4310_Feedback &Out) const
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    Out = Feedback;
    const bool received = Out.Received;

    __DMB();
    __set_PRIMASK(primask);

    return received;
}

bool Class_DM4310::Control()
{
    if (Control_Mode == DM4310_CONTROL_MIT)
    {
        return Transmit_MIT(Target_MIT);
    }

    Struct_DM4310_MIT_Command command = {};
    Struct_DM4310_Feedback feedback = {};

    // 无效目标不触发使能，也不进入速度闭环。
    if (!std::isfinite(Target_Speed_Rpm))
    {
        Out_Torque_Nm = 0.0f;
        Speed_PID.Reset();
        return Transmit_MIT(command);
    }

    const bool received = Get_Feedback_Snapshot(feedback);
    const bool fresh = received &&
        Sys_Get_Micros() - feedback.Last_Rx_Time_Us <= DM4310_FEEDBACK_TIMEOUT_US;

    // 速度目标隐含启动请求；收到新鲜的使能反馈后才运行 PID。
    // 每拍只提交一帧，使能尚未生效或 CAN 提交失败时由下一拍重试。
    if (!fresh || feedback.Measure.State != 1U)
    {
        Out_Torque_Nm = 0.0f;
        Speed_PID.Reset();
        if (!fresh || feedback.Measure.State == 0U)
        {
            return Enable();
        }
        // 故障状态不自动清除，继续提交零扭矩。
        return Transmit_MIT(command);
    }

    // 保持目标速度不超过上位机 V_MAX，PID 的目标和反馈统一使用 rpm。
    constexpr float radps_per_rpm = 2.0f * 3.1415926f / 60.0f;
    const float target_radps = Basic_Math_Constrain(
        Target_Speed_Rpm * radps_per_rpm, -V_Max, V_Max);
    Speed_PID.Set_Target(target_radps / radps_per_rpm);
    Speed_PID.Set_Now(feedback.Measure.Velocity_Radps / radps_per_rpm);
    Speed_PID.TIM_Calculate_PeriodElapsedCallback();

    Out_Torque_Nm = Speed_PID.Get_Out();
    if (!std::isfinite(Out_Torque_Nm))
    {
        Out_Torque_Nm = 0.0f;
        Speed_PID.Reset();
    }

    // 仅使用 MIT 的扭矩字段，电机内置的位置和速度增益均为零。
    command.Torque_Nm = Out_Torque_Nm;
    return Transmit_MIT(command);
}

bool Class_DM4310::Transmit_MIT(
    const Struct_DM4310_MIT_Command &Command)
{
    if (CAN_Handler == nullptr)
    {
        return false;
    }

    // 浮点转整数之前，拒绝 NaN 和无穷大。
    if (!std::isfinite(Command.Position_Rad) ||
        !std::isfinite(Command.Velocity_Radps) ||
        !std::isfinite(Command.K_P) ||
        !std::isfinite(Command.K_D) ||
        !std::isfinite(Command.Torque_Nm))
    {
        return false;
    }

    const float position = Basic_Math_Constrain(
        Command.Position_Rad, -P_Max, P_Max);
    const float velocity = Basic_Math_Constrain(
        Command.Velocity_Radps, -V_Max, V_Max);
    const float kp = Basic_Math_Constrain(
        Command.K_P, 0.0f, 500.0f);
    const float kd = Basic_Math_Constrain(
        Command.K_D, 0.0f, 5.0f);
    const float torque = Basic_Math_Constrain(
        Command.Torque_Nm, -T_Max, T_Max);

    const uint16_t position_raw =
        static_cast<uint16_t>(Basic_Math_Float_To_Int(
            position, -P_Max, P_Max, 0, 65535));

    const uint16_t velocity_raw =
        static_cast<uint16_t>(Basic_Math_Float_To_Int(
            velocity, -V_Max, V_Max, 0, 4095));

    const uint16_t kp_raw =
        static_cast<uint16_t>(Basic_Math_Float_To_Int(
            kp, 0.0f, 500.0f, 0, 4095));

    const uint16_t kd_raw =
        static_cast<uint16_t>(Basic_Math_Float_To_Int(
            kd, 0.0f, 5.0f, 0, 4095));

    const uint16_t torque_raw =
        static_cast<uint16_t>(Basic_Math_Float_To_Int(
            torque, -T_Max, T_Max, 0, 4095));

    // 这些边界组合会形成协议特殊命令，不能作为普通 MIT 帧发送。
    if (position_raw == 0xFFFFU &&
        velocity_raw == 0x0FFFU &&
        kp_raw == 0x0FFFU &&
        kd_raw == 0x0FFFU &&
        torque_raw >= 0x0FFBU && torque_raw <= 0x0FFEU)
    {
        return false;
    }

    uint8_t frame[8] = {};

    frame[0] = static_cast<uint8_t>(position_raw >> 8);
    frame[1] = static_cast<uint8_t>(position_raw);
    frame[2] = static_cast<uint8_t>(velocity_raw >> 4);
    frame[3] = static_cast<uint8_t>(
        ((velocity_raw & 0x0FU) << 4) | (kp_raw >> 8));
    frame[4] = static_cast<uint8_t>(kp_raw);
    frame[5] = static_cast<uint8_t>(kd_raw >> 4);
    frame[6] = static_cast<uint8_t>(
        ((kd_raw & 0x0FU) << 4) | (torque_raw >> 8));
    frame[7] = static_cast<uint8_t>(torque_raw);

    return CAN_Transmit(CAN_Handler, Motor_Id, frame, 8U);
}
bool Class_DM4310::Transmit_Command(uint8_t Command)
{
    if (CAN_Handler == nullptr)
    {
        return false;
    }

    const uint8_t frame[8] = {
        0xFFU, 0xFFU, 0xFFU, 0xFFU,
        0xFFU, 0xFFU, 0xFFU, Command
    };

    return CAN_Transmit(CAN_Handler, Motor_Id, frame, 8U);
}

bool Class_DM4310::Enable()
{
    return Transmit_Command(0xFCU);
}

bool Class_DM4310::Disable()
{
    Set_Target_MIT({});
    Speed_PID.Reset();
    return Transmit_Command(0xFDU);
}

bool Class_DM4310::Clear_Error()
{
    return Transmit_Command(0xFBU);
}
