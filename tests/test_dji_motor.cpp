#include "dji_motor.h"
#include "sys_timestamp.h"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

Host_DWT host_dwt = {};
static uint64_t now_us = 100U;
static bool fail_registration = false;
static bool fail_transmission = false;

struct Receiver
{
    CAN_HandleTypeDef *bus;
    uint32_t id;
    void *context;
    void (*callback)(void *, uint8_t *, uint16_t);
};
struct Transmission
{
    CAN_HandleTypeDef *bus;
    uint32_t id;
    std::array<uint8_t, 8> bytes;
};
static std::vector<Receiver> receivers;
static std::vector<Transmission> transmissions;

uint64_t Sys_Get_Micros() { return now_us; }

bool CAN_Register_Device(CAN_HandleTypeDef *bus, uint32_t id, void *context,
                         void (*callback)(void *, uint8_t *, uint16_t))
{
    if (fail_registration || receivers.size() >= CAN_DEVICE_MAX) { return false; }
    receivers.push_back({bus, id, context, callback});
    return true;
}

bool CAN_Transmit(CAN_HandleTypeDef *bus, uint32_t id, const uint8_t *data, uint8_t length)
{
    assert(length == 8U);
    Transmission frame = {bus, id, {}};
    for (unsigned i = 0; i < 8U; ++i) { frame.bytes[i] = data[i]; }
    transmissions.push_back(frame);
    return !fail_transmission;
}

static void Expect_Near(float actual, float expected)
{
    assert(std::fabs(actual - expected) < 0.001f);
}

static void Receive(uint32_t id, std::array<uint8_t, 8> bytes)
{
    unsigned matches = 0;
    for (const Receiver &receiver : receivers)
    {
        if (receiver.id == id)
        {
            receiver.callback(receiver.context, bytes.data(), bytes.size());
            ++matches;
        }
    }
    assert(matches == 1U);
}

static const Transmission &Sent(uint32_t id)
{
    for (const Transmission &frame : transmissions)
    {
        if (frame.id == id) { return frame; }
    }
    assert(false && "missing control frame");
    return transmissions.front();
}

int main()
{
    CAN_HandleTypeDef bus = {reinterpret_cast<void *>(1)};
    Class_DJI_Motor legacy, gm;
    legacy.Init(&bus, 1U, 1.0f, 0.0f, 0.0f);
    gm.Init(&bus, 1U, 1.0f, 0.0f, 0.0f, DJI_MOTOR_TYPE_GM6020);
    Struct_DJI_Motor_Feedback snapshot;
    assert(!gm.Get_Feedback_Snapshot(snapshot));
    gm.Set_Target_Speed_Rpm(10.0f);
    gm.Control();
    assert(gm.Get_Out_Current() == 0);  // 尚无反馈，不能闭环。

    std::array<uint8_t, 8> positive = {0x1F, 0xFE, 0x00, 0xC0, 0xFE, 0x0C, 42, 0};
    legacy.Feedback_Parse(positive.data(), 8U);
    gm.Feedback_Parse(positive.data(), 8U);
    assert(legacy.Get_Feedback_Snapshot(snapshot));
    Expect_Near(snapshot.Measure.Speed_Rpm, 10.0f);
    assert(gm.Get_Feedback_Snapshot(snapshot));
    Expect_Near(snapshot.Measure.Speed_Rpm, 192.0f);
    assert(snapshot.Measure.Ecd == 8190U);
    assert(snapshot.Measure.Torque_Current == -500);
    assert(snapshot.Measure.Temperature == 42U);
    assert(snapshot.Sequence == 1U && snapshot.Last_Rx_Time_Us == now_us);
    Expect_Near(snapshot.Measure.Total_Angle, 0.0f);
    gm.Set_Target_Speed_Rpm(0.0f);
    gm.Control();
    assert(gm.Get_Out_Current() == -192);  // 零目标仍运行速度环，抵抗正转。

    std::array<uint8_t, 8> negative = {0, 2, 0xFF, 0x88, 1, 0xF4, 43, 0};
    ++now_us;
    gm.Feedback_Parse(negative.data(), 8U);
    legacy.Feedback_Parse(negative.data(), 8U);
    gm.Get_Feedback_Snapshot(snapshot);
    Expect_Near(snapshot.Measure.Speed_Rpm, -120.0f);
    assert(snapshot.Measure.Torque_Current == 500 && snapshot.Sequence == 2U);
    Expect_Near(snapshot.Measure.Total_Angle, 4.0f * DJI_ECD_ANGLE_COEF);
    legacy.Get_Feedback_Snapshot(snapshot);
    Expect_Near(snapshot.Measure.Speed_Rpm, -6.25f);

    gm.Get_Feedback_Snapshot(snapshot);
    const auto before_invalid = snapshot;
    auto invalid = negative;
    invalid[0] = 0x20; invalid[1] = 0;  // 编码器 8192 超出协议范围。
    ++now_us;
    gm.Feedback_Parse(nullptr, 8U);
    gm.Feedback_Parse(positive.data(), 7U);
    gm.Feedback_Parse(positive.data(), 9U);
    gm.Feedback_Parse(invalid.data(), 8U);
    gm.Get_Feedback_Snapshot(snapshot);
    assert(snapshot.Sequence == before_invalid.Sequence);
    assert(snapshot.Last_Rx_Time_Us == before_invalid.Last_Rx_Time_Us);
    Expect_Near(snapshot.Measure.Speed_Rpm, -120.0f);
    gm.Feedback_Parse(positive.data(), 8U);  // 2 -> 8190，反向过零。
    gm.Get_Feedback_Snapshot(snapshot);
    Expect_Near(snapshot.Measure.Total_Angle, 0.0f);

    std::array<uint8_t, 8> zero = {};
    gm.Feedback_Parse(zero.data(), 8U);
    gm.Set_Target_Speed_Rpm(20000.0f);
    gm.Control();
    assert(gm.Get_Out_Current() == 16384);
    gm.Set_Target_Speed_Rpm(-20000.0f);
    gm.Control();
    assert(gm.Get_Out_Current() == -16384);
    gm.Set_Target_Speed_Rpm(std::numeric_limits<float>::quiet_NaN());
    gm.Control();
    assert(gm.Get_Out_Current() == 0);

    Class_DJI_Motor integral;
    integral.Init(&bus, 2U, 0.0f, 1000.0f, 0.0f, DJI_MOTOR_TYPE_GM6020);
    integral.Set_Target_Speed_Rpm(10.0f);
    integral.Feedback_Parse(zero.data(), 8U);
    integral.Control(); integral.Control();
    assert(integral.Get_Out_Current() == 20);
    now_us += DJI_FEEDBACK_TIMEOUT_US;
    integral.Control();
    assert(integral.Get_Out_Current() == 30);  // 恰好 5ms 仍保持原契约。
    ++now_us;
    integral.Control();
    assert(integral.Get_Out_Current() == 0);
    integral.Feedback_Parse(zero.data(), 8U);
    integral.Control();
    assert(integral.Get_Out_Current() == 10);  // 超时清积分，反馈恢复后继续。

    // 工厂、总线注册和分组发送均调用真实驱动。
    assert(DJI_Motor_Create(DJI_MOTOR_1, 1, 0, 0) == nullptr);
    DJI_Motor_Init(&bus);
    assert(DJI_Motor_Create(static_cast<Enum_DJI_Motor_Num>(0), 1, 0, 0) == nullptr);
    assert(DJI_Motor_Create(static_cast<Enum_DJI_Motor_Num>(257), 1, 0, 0) == nullptr);
    assert(DJI_Motor_Get(DJI_MOTOR_1, static_cast<Enum_DJI_Motor_Type>(-1)) == nullptr);
    assert(DJI_Motor_Create(DJI_MOTOR_1, 1, 0, 0,
                            static_cast<Enum_DJI_Motor_Type>(-1)) == nullptr);
    assert(DJI_Motor_Create(DJI_MOTOR_8, 1, 0, 0, DJI_MOTOR_TYPE_GM6020) == nullptr);
    for (unsigned id = 1; id <= 4U; ++id)
    {
        assert(DJI_Motor_Create(static_cast<Enum_DJI_Motor_Num>(id), 1, 0, 0));
    }
    auto *gm1 = DJI_Motor_Create(DJI_MOTOR_1, 1, 0, 0, DJI_MOTOR_TYPE_GM6020);
    auto *gm2 = DJI_Motor_Create(DJI_MOTOR_2, 1, 0, 0, DJI_MOTOR_TYPE_GM6020);
    assert(gm1 && gm2 && gm1 != DJI_Motor_Get(DJI_MOTOR_1));
    assert(DJI_Motor_Get(DJI_MOTOR_1, DJI_MOTOR_TYPE_GM6020) == gm1);
    assert(DJI_Motor_Create(DJI_MOTOR_1, 1, 0, 0, DJI_MOTOR_TYPE_GM6020) == nullptr);
    assert(receivers.size() == 6U);
    for (const auto &receiver : receivers) { assert(receiver.bus == &bus); }
    for (unsigned id = 0x201; id <= 0x206; ++id) { Receive(id, zero); }
    DJI_Motor_Control_Task();
    assert(transmissions.size() == 2U);  // 未创建的高编号组不发送。
    assert(Sent(0x200).bytes == zero && Sent(0x1FE).bytes == zero);
    transmissions.clear();

    fail_registration = true;
    assert(DJI_Motor_Create(DJI_MOTOR_7, 1, 0, 0, DJI_MOTOR_TYPE_GM6020) == nullptr);
    assert(DJI_Motor_Get(DJI_MOTOR_7, DJI_MOTOR_TYPE_GM6020) == nullptr);
    fail_registration = false;
    auto *gm7 = DJI_Motor_Create(DJI_MOTOR_7, 1, 0, 0, DJI_MOTOR_TYPE_GM6020);
    auto *legacy8 = DJI_Motor_Create(DJI_MOTOR_8, 1, 0, 0);
    assert(gm7 && legacy8);
    Receive(0x20B, zero); Receive(0x208, zero);
    // 注册表已满：创建失败后不能残留可发送的对象。
    assert(DJI_Motor_Create(DJI_MOTOR_3, 1, 0, 0, DJI_MOTOR_TYPE_GM6020) == nullptr);
    assert(DJI_Motor_Get(DJI_MOTOR_3, DJI_MOTOR_TYPE_GM6020) == nullptr);

    gm1->Set_Target_Speed_Rpm(1000);
    assert(DJI_Motor_Any_Target_Active());
    gm1->Set_Target_Speed_Rpm(0);
    assert(!DJI_Motor_Any_Target_Active());
    DJI_Motor_Get(DJI_MOTOR_1)->Set_Target_Speed_Rpm(1000);
    assert(DJI_Motor_Any_Target_Active());
    DJI_Motor_Get(DJI_MOTOR_1)->Set_Target_Speed_Rpm(0);

    DJI_Motor_Control_Task();
    assert(transmissions.size() == 4U);
    assert(Sent(0x1FE).bytes == zero && Sent(0x2FE).bytes == zero);
    transmissions.clear();
    gm1->Set_Target_Speed_Rpm(1000);
    gm2->Set_Target_Speed_Rpm(-1000);
    gm7->Set_Target_Speed_Rpm(20000);
    DJI_Motor_Get(DJI_MOTOR_1)->Set_Target_Speed_Rpm(1000);
    legacy8->Set_Target_Speed_Rpm(-20000);
    DJI_Motor_Control_Task();
    assert(transmissions.size() == 4U);
    for (const auto &frame : transmissions) { assert(frame.bus == &bus); }
    assert((Sent(0x1FE).bytes == std::array<uint8_t, 8>{3, 0xE8, 0xFC, 0x18, 0, 0, 0, 0}));
    assert((Sent(0x2FE).bytes == std::array<uint8_t, 8>{0, 0, 0, 0, 0x40, 0, 0, 0}));
    assert((Sent(0x200).bytes == std::array<uint8_t, 8>{3, 0xE8, 0, 0, 0, 0, 0, 0}));
    assert((Sent(0x1FF).bytes == std::array<uint8_t, 8>{0, 0, 0, 0, 0, 0, 0xC0, 0}));

    gm1->Set_Target_Speed_Rpm(0);
    gm2->Set_Target_Speed_Rpm(0);
    gm7->Set_Target_Speed_Rpm(0);
    transmissions.clear(); DJI_Motor_Control_Task();
    assert(Sent(0x1FE).bytes == zero && Sent(0x2FE).bytes == zero);
    now_us += DJI_FEEDBACK_TIMEOUT_US + 1U;
    transmissions.clear(); DJI_Motor_Control_Task();
    for (const auto &frame : transmissions) { assert(frame.bytes == zero); }
    Receive(0x205, zero);
    gm1->Set_Target_Speed_Rpm(1000);
    fail_transmission = true;
    transmissions.clear(); DJI_Motor_Control_Task();
    assert(transmissions.size() == 4U);  // 满邮箱时不忙等，下一拍重发。
    fail_transmission = false;
    transmissions.clear(); DJI_Motor_Control_Task();
    assert(Sent(0x1FE).bytes[0] == 3U && Sent(0x1FE).bytes[1] == 0xE8U);

    std::puts("DJI motor self-check passed (real driver/PID; fake CAN/time).");
}
