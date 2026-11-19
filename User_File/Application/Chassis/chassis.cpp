#include "chassis.h"
#include "dt7.h"
#include <cmath>  // isfinite：检查有限数；fabs：取绝对值；fmax/fmin：取较大/较小值。

// 麦轮逆运动学：把整车希望达到的运动速度，换算成四个轮子的目标转速。
// 适用条件：已确认的 45° 麦轮安装方式，绕底盘几何中心旋转。
// 输入：vx 向前为正（m/s），vy 向左为正（m/s），wz 顶视逆时针为正（rad/s）。
// 输出：wheel_rpm[0～3] 依次为左前、右前、左后、右后的输出轴转速（rpm）。
// 轮子正方向按推动车辆向前定义；轮位到电机 ID、实际电机正负号由上层映射。
void Chassis_Mecanum_Inverse(float vx, float vy, float wz, float wheel_rpm[4])
{
    // 调用方必须提供可写的四元素数组；空指针无法接收结果，直接返回。
    if (wheel_rpm == nullptr)
    {
        return;
    }

    // 先清空输出，后面任何检查失败时，四轮结果都保持为零。
    // 使用临时 rpm 数组完成计算和检查后，才写回输出，避免留下部分结果。
    for (int i = 0; i < 4; ++i)
    {
        wheel_rpm[i] = 0.0f;
    }

    // 机械尺寸和转速上限必须大于零；尺寸宏为零表示还没填写。
    // isfinite 为真表示数值既不是 NaN，也不是正负无穷大。
    // vx、vy、wz 可以为零或负数，负号表示与约定正方向相反。
    if (CHASSIS_WHEEL_DIAMETER_MM <= 0.0f ||
        CHASSIS_WHEEL_BASE_MM <= 0.0f ||
        CHASSIS_WHEEL_TRACK_MM <= 0.0f ||
        CHASSIS_MAX_WHEEL_RPM <= 0.0f ||
        !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz))
    {
        return;
    }

    // 旋转系数 k = 半轴距 + 半轮距，单位 m。
    // 轮位相对中心的前后、左右偏移共同决定旋转时所需的轮速。
    // k * wz 是旋转对应的轮子线速度分量，单位 m/s。
    // 0.0005 = 0.5 * 0.001：先取一半，再把 mm 换成 m。
    const float k = (CHASSIS_WHEEL_BASE_MM + CHASSIS_WHEEL_TRACK_MM) * 0.0005f;

    // 线速度换算成转速：线速度 / 轮子周长 = 圈/秒，再乘 60 得到圈/分钟。
    // 分母是轮子周长：π * 直径，0.001 将直径从 mm 换算成 m。
    // rpm_per_mps 表示每 1 m/s 对应多少 rpm。
    // 这里得到轮子/减速器输出轴 rpm，现有 3508 速度环使用相同单位，无需乘减速比。
    const float rpm_per_mps = 60.0f / (3.1415926f * CHASSIS_WHEEL_DIAMETER_MM * 0.001f);

    // 将前后运动、横移、旋转三部分叠加；括号内的单位都是 m/s。
    // vy 和 k*wz 的正负号由已确认的轮组安装方式、上述坐标定义决定。
    // 只向前：四轮都是正；只向左：负、正、正、负；只逆时针转：负、正、负、正。
    // rpm 是尚未限幅的目标转速，顺序仍为左前、右前、左后、右后。
    const float rpm[4] = {
        (vx - vy - k * wz) * rpm_per_mps,  // 左前
        (vx + vy + k * wz) * rpm_per_mps,  // 右前
        (vx + vy - k * wz) * rpm_per_mps,  // 左后
        (vx - vy + k * wz) * rpm_per_mps   // 右后
    };

    // 找到四轮中最大的转速大小。反转也要参与限幅，所以比较绝对值。
    float max_abs_rpm = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
        // 即使输入是有限数，浮点运算也可能溢出；出现异常就保留四个零输出。
        if (!std::isfinite(rpm[i]))
        {
            return;
        }

        // fabs(-200) = 200；fmax 保留目前见到的最大值。
        max_abs_rpm = std::fmax(max_abs_rpm, std::fabs(rpm[i]));
    }

    // 四轮统一限幅：超过上限时，缩放比例 = 上限 / 最大转速大小。
    // 例如最大值为 200 rpm、上限为 100 rpm，则四轮都乘 0.5。
    // 四轮一起缩放，保留平移与旋转的比例；未超限时乘 1，目标转速保持原值。
    // “条件 ? A : B”表示条件成立取 A，否则取 B。
    const float scale = (max_abs_rpm > CHASSIS_MAX_WHEEL_RPM)
                        ? CHASSIS_MAX_WHEEL_RPM / max_abs_rpm
                        : 1.0f;

    // 将检查通过、完成限幅的四轮目标转速写入调用方的输出数组。
    for (int i = 0; i < 4; ++i)
    {
        wheel_rpm[i] = rpm[i] * scale;
    }
}

// 摇杆原始值约为 -660～660，转换成 -1～1。
static float Chassis_Normalize_Channel(int16_t channel)
{
    // 中心死区，避免摇杆回中后的微小抖动产生运动指令。
    if (channel >= -20 && channel <= 20)
    {
        return 0.0f;
    }

    const float value = channel / 660.0f;

    // 实际通道值可能稍超出标称范围，仍将指令限制到 -1～1。
    return std::fmax(-1.0f, std::fmin(value, 1.0f));
}

void Chassis_Update(float target_rpm[4])
{
    if (target_rpm == nullptr)
    {
        return;
    }

    // 每次先使用零指令；未收到有效遥控数据、超时或不在运行档时，覆盖旧目标停车。
    float vx = 0.0f;  // 向前速度，m/s
    float vy = 0.0f;  // 向左速度，m/s
    float wz = 0.0f;  // 逆时针角速度，rad/s

    Struct_DT7_Data remote = {};

    // 快照接口已包含 100ms 超时检查；左拨杆上档时允许控制底盘。
    if (DT7_Get_Snapshot(&remote) &&
        remote.Switch_Left == DT7_SWITCH_UP)
    {
        // 右摇杆前后：按前推为正映射到前后速度。
        vx = Chassis_Normalize_Channel(remote.Channel[1])
             * CHASSIS_MAX_VX_MPS;

        // 右摇杆左右：按右推为正处理，底盘向左为正，因此取负。
        vy = -Chassis_Normalize_Channel(remote.Channel[0])
             * CHASSIS_MAX_VY_MPS;

        // 左摇杆左右：按右推为正处理，底盘逆时针为正，因此取负。
        wz = -Chassis_Normalize_Channel(remote.Channel[2])
             * CHASSIS_MAX_WZ_RADPS;
    }

    // 接收四轮解算结果：左前、右前、左后、右后。
    float wheel_rpm[4] = {0.0f};
    Chassis_Mecanum_Inverse(vx, vy, wz, wheel_rpm);

    // 按电机 ID 存入目标数组，同时修正实际安装方向。
    // ID 从 1 开始，数组下标从 0 开始，所以需要减 1。
    target_rpm[CHASSIS_FL_ID - 1U] = wheel_rpm[0] * CHASSIS_FL_DIRECTION;

    target_rpm[CHASSIS_FR_ID - 1U] = wheel_rpm[1] * CHASSIS_FR_DIRECTION;

    target_rpm[CHASSIS_RL_ID - 1U] = wheel_rpm[2] * CHASSIS_RL_DIRECTION;

    target_rpm[CHASSIS_RR_ID - 1U] = wheel_rpm[3] * CHASSIS_RR_DIRECTION;
}
