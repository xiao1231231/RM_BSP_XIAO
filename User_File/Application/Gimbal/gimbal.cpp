#include "gimbal.h"
#include "gimbal_config.h"
#include "dji_motor.h"
#include "sys_timestamp.h"
#include "alg_filter_iir.h"

#include <cmath>

namespace
{
struct Gimbal_State
{
    Class_DJI_Motor *Motor = nullptr;
    Class_SMC Smc;
    Class_Filter_IIR_First_Order Speed_Filter;
    float Target_Angle_Deg = Gimbal_Config::Target_Angle_Deg;
    float Target_Angular_Velocity_Radps = 0.0f;
    uint64_t Last_Update_Us = 0U;
    // 最近一拍输出；Integral_Current 同时作为积分状态，避免另存副本。
    Struct_Gimbal_Debug Data = {};
    uint64_t Feedback_Time_Us = 0U;
    bool Valid = false;
} gimbal;

// 临界区只复制数据，计算留在外部；保证 USB 读到同一拍的完整快照。
void Publish(const Struct_Gimbal_Debug &data, uint64_t feedback_time_us, bool valid)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    gimbal.Data = data;
    gimbal.Feedback_Time_Us = feedback_time_us;
    gimbal.Valid = valid;
    __DMB();
    __set_PRIMASK(primask);
}

void Reset_Control()
{
    gimbal.Last_Update_Us = 0U;
    auto data = gimbal.Data;
    data.Integral_Current = 0.0f;
    Publish(data, 0U, false);
}
}

void Gimbal_Init(void)
{
    gimbal.Motor = DJI_Motor_Get(static_cast<Enum_DJI_Motor_Num>(Gimbal_Config::Motor_Id),
                                 DJI_MOTOR_TYPE_GM6020);
    gimbal.Smc.Init(Gimbal_Config::Smc);
    gimbal.Speed_Filter.Init(Gimbal_Config::Speed_Filter_Hz, 1000.0f);
    gimbal.Target_Angular_Velocity_Radps = 0.0f;
    Gimbal_Set_Target_Angle_Deg(Gimbal_Config::Target_Angle_Deg);
    gimbal.Last_Update_Us = 0U;
    Publish({}, 0U, false);
}

void Gimbal_Update(void)
{
    gimbal.Valid = false;
    if (gimbal.Motor == nullptr)
    {
        return;
    }

    // 默认零电流；参数、反馈或计算异常时直接返回。
    gimbal.Motor->Set_Target_Current(0.0f);
    Struct_DJI_Motor_Feedback feedback = {};
    if (!gimbal.Smc.Is_Initialized() || !gimbal.Motor->Get_Feedback_Snapshot(feedback) ||
        (Sys_Get_Micros() - feedback.Last_Rx_Time_Us) > DJI_FEEDBACK_TIMEOUT_US)
    {
        Reset_Control();
        return;
    }

    auto next = gimbal.Data;
    next.Angle_Deg = static_cast<float>(feedback.Measure.Ecd) * (360.0f / 8192.0f)
                     * Gimbal_Config::Direction;
    if (next.Angle_Deg < 0.0f)
    {
        next.Angle_Deg += 360.0f;
    }

    // 单圈绝对角度和实际角速度统一到云台正方向。
    const float raw_speed_rad_s = feedback.Measure.Speed_Rpm * (2.0f * 3.1415926f / 60.0f)
                                 * Gimbal_Config::Direction;
    if (!std::isfinite(raw_speed_rad_s))
    {
        Reset_Control();
        return;
    }

    const uint64_t now_us = Sys_Get_Micros();
    const uint64_t elapsed_us = now_us - gimbal.Last_Update_Us;
    float dt_s = 0.0f;
    if (gimbal.Last_Update_Us != 0U && elapsed_us <= DJI_FEEDBACK_TIMEOUT_US)
    {
        dt_s = static_cast<float>(elapsed_us) * 0.000001f;
    }
    else
    {
        // 首拍、断线恢复或控制中断超过 5 ms：清积分并对齐速度滤波。
        next.Integral_Current = 0.0f;
        gimbal.Speed_Filter.Reset(raw_speed_rad_s);
    }
    gimbal.Last_Update_Us = now_us;

    // 遥控器给的是角速度；用实测控制周期积分为位置目标，回中后目标自然保持。
    if (dt_s > 0.0f)
    {
        gimbal.Target_Angle_Deg = std::fmod(
            gimbal.Target_Angle_Deg +
                gimbal.Target_Angular_Velocity_Radps * dt_s * (180.0f / 3.1415926f),
            360.0f);
        if (gimbal.Target_Angle_Deg < 0.0f)
        {
            gimbal.Target_Angle_Deg += 360.0f;
        }
    }

    // 单圈最短路径，误差落在 [-180, 180)；包含本拍遥控角速度积分后的目标。
    const float angle_error_rad = Basic_Math_Modulus_Normalization(
        gimbal.Target_Angle_Deg - next.Angle_Deg, 360.0f) * (3.1415926f / 180.0f);
    if (!std::isfinite(angle_error_rad))
    {
        Reset_Control();
        return;
    }

    gimbal.Speed_Filter.Set_Now(raw_speed_rad_s);
    gimbal.Speed_Filter.TIM_Calculate_PeriodElapsedCallback();
    // 保持角度时目标角速度为零；角度已到位但仍在转动时也继续制动。
    const float smc_current = gimbal.Smc.Calculate(
    angle_error_rad,
    gimbal.Target_Angular_Velocity_Radps - gimbal.Speed_Filter.Get_Out());

    if ((next.Integral_Current * angle_error_rad < 0.0f) &&
        (std::fabs(angle_error_rad) > Gimbal_Config::Smc.Linear_Threshold))
    {
        // 越过目标确认区后丢弃仍推向过冲方向的旧积分，再积分当前误差。
        next.Integral_Current = 0.0f;
    }
    const float integral_step = Gimbal_Config::Integral_Ki * angle_error_rad * dt_s;
    float next_integral = next.Integral_Current + integral_step;
    if (!std::isfinite(next_integral))
    {
        Reset_Control();
        return;
    }
    next_integral = Basic_Math_Constrain(
        next_integral, -Gimbal_Config::Integral_Max, Gimbal_Config::Integral_Max);

    // 沿用原先的抗饱和逻辑：总输出超限时，拒绝继续向该方向累积积分。
    const float next_current = smc_current + next_integral;
    const float current_max = Gimbal_Config::Smc.Out_Max;
    if (!((next_current > current_max && integral_step > 0.0f) ||
          (next_current < -current_max && integral_step < 0.0f)))
    {
        next.Integral_Current = next_integral;
    }
    next.Control_Current = Basic_Math_Constrain(
        smc_current + next.Integral_Current, -current_max, current_max);

    gimbal.Motor->Set_Target_Current(next.Control_Current * Gimbal_Config::Direction);
    Publish(next, feedback.Last_Rx_Time_Us, true);
}

void Gimbal_Set_Target_Angle_Deg(float angle_deg)
{
    gimbal.Target_Angular_Velocity_Radps = 0.0f;
    if (std::isfinite(angle_deg))
    {
        angle_deg = std::fmod(angle_deg, 360.0f);
        if (angle_deg < 0.0f)
        {
            angle_deg += 360.0f;
        }
    }
    gimbal.Target_Angle_Deg = angle_deg;
}

void Gimbal_Set_Target_Angular_Velocity_Radps(float rate_rad_s)
{
    gimbal.Target_Angular_Velocity_Radps = std::isfinite(rate_rad_s) ? rate_rad_s : 0.0f;
}

bool Gimbal_Get_Debug_Snapshot(Struct_Gimbal_Debug *data)
{
    if (data == nullptr)
    {
        return false;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *data = gimbal.Data;
    const uint64_t rx_time_us = gimbal.Feedback_Time_Us;
    const bool valid = gimbal.Valid;
    __DMB();
    __set_PRIMASK(primask);

    return valid && (Sys_Get_Micros() - rx_time_us) <= DJI_FEEDBACK_TIMEOUT_US;
}
