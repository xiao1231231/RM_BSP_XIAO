#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H

#include "alg_smc.h"

// GM6020 角度控制的调参入口；修改后重新编译生效。
// SMC 使用 rad 和 rad/s，启动目标使用度；电流均为 CAN 指令数值，不是 A。
namespace Gimbal_Config
{
constexpr uint8_t Motor_Id = 5U;          // 已创建的 GM6020 ID：5 或 6。
constexpr float Direction = +1.0f;        // 云台正方向相对编码器方向，只填 +1 或 -1。
constexpr float Target_Angle_Deg = 0.0f;  // 启动目标，单位度；0° 对应 Ecd=0。

constexpr float Speed_Filter_Hz = 0.0f;   // 速度低通截止频率；0 旁路，采样频率 1000 Hz。
constexpr float Integral_Ki = 800.0f;      // 角度误差积分增益，指令/(rad·s)；0 关闭补偿。
constexpr float Integral_Max = 2000.0f;    // 积分补偿限幅，±电流指令。
constexpr int16_t Remote_Deadband_Channel = 20; // 左摇杆水平通道死区。
constexpr float Remote_Max_Yaw_Rate_Deg_Per_S = 60.0f; // 摇杆打满时的目标角速度。
constexpr float Remote_Yaw_Direction = -1.0f; // 右拨为顺时针；方向相反时改为 +1。

constexpr Struct_SMC_Config Smc = {
    11.0f,                              // C：滑模面角度权重，1/s。
    11+.0f,                              // K：线性趋近增益，1/s。
    10.0f,                              // Epsilon：切换项强度，rad/s²；0 关闭。
    0.4f,                              // Phi：切换项的边界层宽度，rad/s。
    8.8f,                             // J：角加速度到电流指令的换算增益。
    21U, 21U,                          // P、Q：非线性幂次 Q/P；相等则退回线性。
    1.0f * 3.1415926f / 180.0f,         // Linear_Threshold：±1° 内使用线性段，单位 rad。
    1700.0f                            // Out_Max：SMC 和最终总输出共用的限幅。
};

static_assert(Motor_Id >= 1U && Motor_Id <= 7U, "Invalid GM6020 ID");
static_assert(Direction == 1.0f || Direction == -1.0f, "Direction must be +1 or -1");
static_assert(Speed_Filter_Hz >= 0.0f && Speed_Filter_Hz < 500.0f, "Invalid filter cutoff");
static_assert(Integral_Ki >= 0.0f && Integral_Max >= 0.0f, "Invalid integral parameters");
static_assert(Remote_Deadband_Channel >= 0 && Remote_Deadband_Channel < 660,
              "Invalid remote deadband");
static_assert(Remote_Max_Yaw_Rate_Deg_Per_S > 0.0f,
              "Invalid remote yaw rate");
static_assert(Remote_Yaw_Direction == 1.0f || Remote_Yaw_Direction == -1.0f,
              "Remote yaw direction must be +1 or -1");
}

#endif // GIMBAL_CONFIG_H
