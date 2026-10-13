#include "chassis.h"
#include <cmath>  // isfinite：检查有限数；fabs：取绝对值；fmax：取较大值。

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
