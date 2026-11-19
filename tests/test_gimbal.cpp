#include "alg_smc.h"
#include "dji_motor.h"
#include "gimbal.h"
#include "gimbal_config.h"
#include "gimbal_remote.h"
#include "sys_timestamp.h"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

Host_DWT host_dwt = {};
static uint64_t now_us = 100U;
static constexpr uint8_t motor_id = Gimbal_Config::Motor_Id;
static std::FILE *trace_file = nullptr;

uint64_t Sys_Get_Micros() { return now_us; }
bool CAN_Register_Device(CAN_HandleTypeDef *, uint32_t, void *,
                         void (*)(void *, uint8_t *, uint16_t)) { return true; }
bool CAN_Transmit(CAN_HandleTypeDef *, uint32_t, const uint8_t *, uint8_t) { return true; }

static void Expect_Near(float actual, float expected, float tolerance = 0.001f)
{
    if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance)
    {
        std::fprintf(stderr, "expected %.9g, got %.9g\n", expected, actual);
        assert(false);
    }
}

static void Receive(Class_DJI_Motor *motor, uint16_t ecd, int16_t rpm = 0)
{
    std::array<uint8_t, 8> frame = {
        static_cast<uint8_t>(ecd >> 8), static_cast<uint8_t>(ecd),
        static_cast<uint8_t>(static_cast<uint16_t>(rpm) >> 8), static_cast<uint8_t>(rpm),
        0, 0, 35, 0};
    motor->Feedback_Parse(frame.data(), frame.size());
}

static Struct_Gimbal_Debug Step(Class_DJI_Motor *motor, uint16_t ecd,
                                int16_t rpm = 0, uint64_t dt_us = 1000U)
{
    now_us += dt_us;
    Receive(motor, ecd, rpm);
    Gimbal_Update();
    motor->Control();
    Struct_Gimbal_Debug data = {};
    assert(Gimbal_Get_Debug_Snapshot(&data));
    assert(std::isfinite(data.Control_Current));
    Expect_Near(motor->Get_Target_Current(), data.Control_Current * Gimbal_Config::Direction);
    assert(std::fabs(data.Control_Current) <= Gimbal_Config::Smc.Out_Max);
    assert(std::fabs(data.Integral_Current) <= Gimbal_Config::Integral_Max);
    if (trace_file != nullptr)
    {
        std::fprintf(trace_file, "%llu\t%a\t%a\t%a\t%d\n",
                     static_cast<unsigned long long>(now_us),
                     static_cast<double>(data.Angle_Deg),
                     static_cast<double>(data.Integral_Current),
                     static_cast<double>(data.Control_Current), motor->Get_Out_Current());
    }
    return data;
}

static void Check_Algorithm()
{
    Class_SMC smc;
    Expect_Near(smc.Calculate(1, 0), 0);  // 未初始化不输出。
    Struct_SMC_Config config = {};
    config.C = 4; config.K = 8; config.Epsilon = 2; config.Phi = 0.2f;
    config.J = 12.5f; config.P = 21; config.Q = 21;
    config.Linear_Threshold = 0.02f; config.Out_Max = 2000;
    assert(smc.Init(config));
    // 线性段的独立解析值，包括制动、边界层内外和目标加速度。
    Expect_Near(smc.Calculate(0.01f, 0), 9.0f);
    Expect_Near(smc.Calculate(0, -0.1f), -27.5f);
    Expect_Near(smc.Calculate(0, 0.1f), 27.5f);
    Expect_Near(smc.Calculate(0, 0, 3), 37.5f);
    Expect_Near(smc.Calculate(1, 0), 425.0f);
    Expect_Near(smc.Calculate(100, 0), config.Out_Max);
    Expect_Near(smc.Calculate(-100, 0), -config.Out_Max);
    Expect_Near(smc.Calculate(std::numeric_limits<float>::quiet_NaN(), 0), 0);
    Expect_Near(smc.Calculate(0.01f, 0), 9.0f);  // 异常输入后可以恢复。
    config.Q = 27;
    assert(smc.Init(config));
    Expect_Near(smc.Calculate(0.2f, -0.1f), -smc.Calculate(-0.2f, 0.1f));
    const float left = smc.Calculate(config.Linear_Threshold - 0.000001f, 0.1f);
    const float right = smc.Calculate(config.Linear_Threshold + 0.000001f, 0.1f);
    Expect_Near(left, right, 0.01f);  // 幂次/线性拼接不跳变。
    Expect_Near(smc.Calculate(std::numeric_limits<float>::max(), 0), 0);
    config.P = 0;
    assert(!smc.Init(config));
    Expect_Near(smc.Calculate(1, 0), 0);  // 初始化失败不能沿用旧配置。
}

static void Check_Gimbal_Remote_Mapping()
{
    Struct_DT7_Data remote = {};
    remote.Valid = 1U;
    remote.Switch_Left = DT7_SWITCH_MIDDLE;
    remote.Channel[2] = 660;
    Expect_Near(Gimbal_Remote_Map_Yaw_Rate_Radps(remote),
                Gimbal_Config::Remote_Yaw_Direction *
                Gimbal_Config::Remote_Max_Yaw_Rate_Deg_Per_S *
                (3.1415926f / 180.0f));

    remote.Channel[2] = -660;
    Expect_Near(Gimbal_Remote_Map_Yaw_Rate_Radps(remote),
                -Gimbal_Config::Remote_Yaw_Direction *
                Gimbal_Config::Remote_Max_Yaw_Rate_Deg_Per_S *
                (3.1415926f / 180.0f));

    remote.Channel[2] = Gimbal_Config::Remote_Deadband_Channel;
    Expect_Near(Gimbal_Remote_Map_Yaw_Rate_Radps(remote), 0.0f);
    remote.Channel[2] = -Gimbal_Config::Remote_Deadband_Channel;
    Expect_Near(Gimbal_Remote_Map_Yaw_Rate_Radps(remote), 0.0f);

    remote.Channel[2] = 660;
    remote.Switch_Left = DT7_SWITCH_UP;
    Expect_Near(Gimbal_Remote_Map_Yaw_Rate_Radps(remote), 0.0f);
    remote.Switch_Left = DT7_SWITCH_MIDDLE;
    remote.Valid = 0U;
    Expect_Near(Gimbal_Remote_Map_Yaw_Rate_Radps(remote), 0.0f);
}

int main(int argc, char **argv)
{
    if (argc > 1)
    {
        trace_file = std::fopen(argv[1], "w");
        assert(trace_file != nullptr);
    }
    Check_Algorithm();
    Check_Gimbal_Remote_Mapping();
    Struct_Gimbal_Debug data = {};
    Gimbal_Init();
    Gimbal_Update();
    assert(!Gimbal_Get_Debug_Snapshot(&data));  // 没有电机。
    assert(!Gimbal_Get_Debug_Snapshot(nullptr));

    CAN_HandleTypeDef bus = {reinterpret_cast<void *>(1)};
    DJI_Motor_Init(&bus);
    auto *motor = DJI_Motor_Create(static_cast<Enum_DJI_Motor_Num>(motor_id),
                                   15, 0, 0, DJI_MOTOR_TYPE_GM6020);
    assert(motor != nullptr);
    Gimbal_Init();
    Gimbal_Update();
    motor->Control();
    assert(!Gimbal_Get_Debug_Snapshot(&data));
    assert(motor->Get_Out_Current() == 0);  // 创建成功但尚无反馈。

    Gimbal_Set_Target_Angle_Deg(1);
    data = Step(motor, 8191U);
    assert(data.Control_Current * Gimbal_Config::Direction > 0);  // 跨零选择短路径。
    Expect_Near(data.Integral_Current, 0);  // 首拍不积分。
    Gimbal_Set_Target_Angle_Deg(359);
    data = Step(motor, 0);
    assert(data.Control_Current > -100 && data.Control_Current < 0);

    // 正负转速都应得到制动力，角度到位时也不能直接置零。
    Gimbal_Set_Target_Angle_Deg(0);
    Step(motor, 0, 0, DJI_FEEDBACK_TIMEOUT_US + 1U);
    data = Step(motor, 0, 60);
    assert(data.Control_Current * Gimbal_Config::Direction < 0);
    data = Step(motor, 0, -60, DJI_FEEDBACK_TIMEOUT_US + 1U);
    assert(data.Control_Current * Gimbal_Config::Direction > 0);

    // 相隔完整圈的目标应等效。
    Gimbal_Init();
    Gimbal_Set_Target_Angle_Deg(450);
    const auto wrapped = Step(motor, 2048U);
    Gimbal_Init();
    Gimbal_Set_Target_Angle_Deg(90);
    const auto direct = Step(motor, 2048U);
    Expect_Near(wrapped.Control_Current, direct.Control_Current);

    Gimbal_Init();
    Gimbal_Set_Target_Angle_Deg(1);
    Step(motor, 0);
    for (unsigned i = 0; i < 100; ++i) { data = Step(motor, 0); }
    assert(data.Integral_Current >= 0);

    // 过冲确认区内保留积分；越过线性区后清除方向相反的旧积分。
    Gimbal_Init();
    Gimbal_Set_Target_Angle_Deg(20);
    Step(motor, 0);  // 首拍不积分。
    for (unsigned i = 0; i < 100; ++i) { data = Step(motor, 0); }
    assert(data.Integral_Current > 0);
    data = Step(motor, 466);  // 约 20.48°，过冲小于 1°。
    assert(data.Integral_Current > 0);
    data = Step(motor, 489);  // 约 21.49°，过冲超过 1°。
    assert(data.Integral_Current < 0);

    // 5ms 边界保持有效，超过边界零输出、清积分；恢复首拍重新计时。
    now_us += DJI_FEEDBACK_TIMEOUT_US;
    assert(Gimbal_Get_Debug_Snapshot(&data));
    ++now_us;
    assert(!Gimbal_Get_Debug_Snapshot(&data));
    Gimbal_Update();
    motor->Control();
    assert(motor->Get_Out_Current() == 0);
    data = Step(motor, 0);
    Expect_Near(data.Integral_Current, 0);

    for (float invalid : {std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::infinity()})
    {
        Gimbal_Set_Target_Angle_Deg(invalid);
        Gimbal_Update();
        motor->Control();
        assert(motor->Get_Out_Current() == 0);
        assert(!Gimbal_Get_Debug_Snapshot(&data));
    }
    Gimbal_Set_Target_Angle_Deg(0);
    data = Step(motor, 0);
    Expect_Near(data.Control_Current, 0);

    // 左摇杆速度命令积分为角度目标；回中后目标角保持，不突然回到启动角。
    Gimbal_Init();
    Gimbal_Set_Target_Angular_Velocity_Radps(-3.1415926f / 2.0f);
    Step(motor, 0); // 首拍只建立控制时间基准。
    for (unsigned i = 0; i < 1000; ++i)
    {
        data = Step(motor, 0);
    }
    assert(data.Control_Current * Gimbal_Config::Direction < 0.0f);
    Gimbal_Set_Target_Angular_Velocity_Radps(0.0f);
    data = Step(motor, 0);
    assert(data.Control_Current * Gimbal_Config::Direction < 0.0f);

    // 保存确定性轨迹摘要，便于本次重构前后直接比较真实执行结果。
    double trace = 0;
    for (unsigned i = 0; i < 2000; ++i)
    {
        const float target = static_cast<float>(i % 361U) - 180.0f;
        Gimbal_Set_Target_Angle_Deg(target);
        data = Step(motor, (i * 71U) % 8192U,
                    static_cast<int16_t>(i % 121U) - 60,
                    i % 101U == 0 ? 6000U : 1000U);
        trace += (i + 1) * (static_cast<double>(data.Control_Current) +
                            data.Integral_Current * 0.125);
    }
    std::printf("Gimbal self-check passed (real SMC/gimbal/driver; fake CAN/time). trace=%.12f\n", trace);
    if (trace_file != nullptr) { std::fclose(trace_file); }
}
