#include "alg_smc.h"

#include <cmath>

// 参考：https://github.com/Fudan-RobotEGA/SMC_Cpp
// 沿用本工程“目标减实际”的误差与输出符号约定。
bool Class_SMC::Init(const Struct_SMC_Config &config)
{
    Initialized = false;
    if (!std::isfinite(config.C) || !std::isfinite(config.K) ||
        !std::isfinite(config.Epsilon) || !std::isfinite(config.Phi) ||
        !std::isfinite(config.J) || !std::isfinite(config.Linear_Threshold) ||
        !std::isfinite(config.Out_Max) ||
        config.C <= 0.0f || config.K < 0.0f || config.Epsilon < 0.0f ||
        config.Phi <= 0.0f || config.J <= 0.0f || config.P == 0U || config.Q == 0U ||
        config.Linear_Threshold <= 0.0f || config.Out_Max <= 0.0f)
    {
        return false;
    }
    Config = config;
    Initialized = true;
    return true;
}

float Class_SMC::Calculate(float error, float error_rate, float target_acceleration) const
{
    if (!Initialized || !std::isfinite(error) || !std::isfinite(error_rate) ||
        !std::isfinite(target_acceleration))
    {
        return 0.0f;
    }

    // 小误差 f(e)=e；大误差使用幂次，两段的值和斜率在 h 处连续。
    // f(e)=sign(e)*h*[1+((|e|/h)^(Q/P)-1)/(Q/P)]，h=Linear_Threshold。
    float shaped_error = error;
    float shaped_slope = 1.0f;
    const float abs_error = std::fabs(error);
    const float power_ratio = static_cast<float>(Config.Q) / static_cast<float>(Config.P);
    if (abs_error > Config.Linear_Threshold && power_ratio != 1.0f)
    {
        const float ratio = abs_error / Config.Linear_Threshold;
        const float power_value = std::pow(ratio, power_ratio);
        const float shaped_abs = Config.Linear_Threshold *
            (1.0f + (power_value - 1.0f) / power_ratio);
        shaped_error = std::copysign(shaped_abs, error);
        shaped_slope = power_value / ratio;
    }
    if (!std::isfinite(shaped_error) || !std::isfinite(shaped_slope))
    {
        return 0.0f;
    }

    const float s = error_rate + Config.C * shaped_error;
    if (!std::isfinite(s))
    {
        return 0.0f;
    }
    const float saturation = s >= Config.Phi ? 1.0f :
                             s <= -Config.Phi ? -1.0f : s / Config.Phi;
    const float speed_term = Config.C * shaped_slope * error_rate;
    const float linear_term = Config.K * s;
    const float switching_term = Config.Epsilon * saturation;
    const float acceleration = target_acceleration + speed_term + linear_term + switching_term;
    if (!std::isfinite(acceleration))
    {
        return 0.0f;
    }

    // J 含实际输出的单位换算，不等同于可直接照搬的机械惯量。
    const float control = Config.J * acceleration;
    if (!std::isfinite(control))
    {
        return 0.0f;
    }
    return control > Config.Out_Max ? Config.Out_Max :
           control < -Config.Out_Max ? -Config.Out_Max : control;
}
