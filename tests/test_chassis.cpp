#include "chassis.h"
#include "dt7.h"
#include "bsp_uart.h"
#include "sys_timestamp.h"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>

Host_DWT host_dwt = {};
static uint64_t now_us = 100U;
static UART_Callback receive_callback = nullptr;

uint64_t Sys_Get_Micros() { return now_us; }
void UART_Init(UART_HandleTypeDef *, UART_Callback callback)
{
    receive_callback = callback;
}

// 模拟收到一帧 DBUS 数据，后续解析、有效性和超时判断使用真实 DT7 驱动。
static void Receive(std::array<int16_t, 4> channels, uint8_t left = DT7_SWITCH_UP)
{
    std::array<uint8_t, DT7_FRAME_LENGTH> frame = {};
    for (unsigned channel = 0; channel < channels.size(); ++channel)
    {
        const unsigned raw = channels[channel] + DT7_CHANNEL_CENTER;
        for (unsigned bit = 0; bit < 11U; ++bit)
        {
            const unsigned offset = channel * 11U + bit;
            frame[offset / 8U] |= ((raw >> bit) & 1U) << (offset % 8U);
        }
    }
    frame[5] |= (left << 6U) | (DT7_SWITCH_MIDDLE << 4U);
    assert(receive_callback != nullptr);
    receive_callback(frame.data(), frame.size());
}

static void Expect_Near(float actual, float expected)
{
    if (!std::isfinite(actual) || std::fabs(actual - expected) >= 0.001f)
    {
        std::fprintf(stderr, "expected %.6f rpm, got %.6f rpm\n", expected, actual);
        assert(false && "unexpected motor target");
    }
}

static void Expect_Wheels(const float targets[4], float fl, float fr, float rl, float rr)
{
    Expect_Near(targets[CHASSIS_FL_ID - 1U], fl * CHASSIS_FL_DIRECTION);
    Expect_Near(targets[CHASSIS_FR_ID - 1U], fr * CHASSIS_FR_DIRECTION);
    Expect_Near(targets[CHASSIS_RL_ID - 1U], rl * CHASSIS_RL_DIRECTION);
    Expect_Near(targets[CHASSIS_RR_ID - 1U], rr * CHASSIS_RR_DIRECTION);
}

int main()
{
    UART_HandleTypeDef uart = {};
    DT7_Init(&uart);
    float targets[4] = {123, 123, 123, 123};
    Chassis_Update(targets);
    Expect_Wheels(targets, 0, 0, 0, 0);  // 尚无遥控数据，覆盖旧目标。

    const float rpm_per_mps = 60.0f / (3.1415926f * CHASSIS_WHEEL_DIAMETER_MM * 0.001f);
    const float forward = CHASSIS_MAX_VX_MPS * rpm_per_mps;
    const float sideways = CHASSIS_MAX_VY_MPS * rpm_per_mps;
    const float rotation = CHASSIS_MAX_WZ_RADPS * rpm_per_mps *
        (CHASSIS_WHEEL_BASE_MM + CHASSIS_WHEEL_TRACK_MM) * 0.0005f;

    Receive({0, 660, 0, 0});  // 右杆前推。
    Chassis_Update(targets);
    Expect_Wheels(targets, forward, forward, forward, forward);
    Receive({0, -660, 0, 0});
    Chassis_Update(targets);
    Expect_Wheels(targets, -forward, -forward, -forward, -forward);
    Receive({660, 0, 0, 0});  // 右杆右推，向右横移。
    Chassis_Update(targets);
    Expect_Wheels(targets, sideways, -sideways, -sideways, sideways);
    Receive({0, 0, 660, 0});  // 左杆右推，顺时针旋转。
    Chassis_Update(targets);
    Expect_Wheels(targets, rotation, -rotation, rotation, -rotation);

    Receive({0, 330, 0, 0});
    Chassis_Update(targets);
    Expect_Wheels(targets, forward / 2, forward / 2, forward / 2, forward / 2);
    for (int16_t value : {-20, 0, 20})
    {
        Receive({value, value, value, 660});  // 左杆前后不参与控制。
        Chassis_Update(targets);
        Expect_Wheels(targets, 0, 0, 0, 0);
    }
    Receive({0, 21, 0, 0});  // 刚越过死区。
    Chassis_Update(targets);
    const float edge = forward * 21.0f / 660.0f;
    Expect_Wheels(targets, edge, edge, edge, edge);
    for (int16_t value : {-700, 700})  // 合法帧可能略超出标称摇杆范围，指令仍限幅。
    {
        Receive({0, value, 0, 0});
        Chassis_Update(targets);
        const float limited = value > 0 ? forward : -forward;
        Expect_Wheels(targets, limited, limited, limited, limited);
    }

    for (uint8_t position : {DT7_SWITCH_MIDDLE, DT7_SWITCH_DOWN})
    {
        Receive({0, 660, 0, 0});
        Chassis_Update(targets);
        Receive({0, 660, 0, 0}, position);
        Chassis_Update(targets);
        Expect_Wheels(targets, 0, 0, 0, 0);
    }

    Receive({0, 660, 0, 0});
    now_us += DT7_TIMEOUT_US - 1U;
    Chassis_Update(targets);
    Expect_Wheels(targets, forward, forward, forward, forward);
    ++now_us;  // 到达真实 DT7 超时边界，必须清掉上一拍的运动目标。
    Chassis_Update(targets);
    Expect_Wheels(targets, 0, 0, 0, 0);
    Receive({0, 660, 0, 0});
    Chassis_Update(targets);
    Expect_Wheels(targets, forward, forward, forward, forward);

    Receive({-660, 660, -660, 0});  // 平移与旋转叠加，覆盖四轮统一限幅。
    Chassis_Update(targets);
    float largest = 0;
    for (float target : targets)
    {
        assert(std::isfinite(target));
        largest = std::fmax(largest, std::fabs(target));
    }
    Expect_Near(largest, CHASSIS_MAX_WHEEL_RPM);
    Chassis_Update(nullptr);

    std::puts("Chassis self-check passed (real chassis/DT7; fake UART/time).");
}
