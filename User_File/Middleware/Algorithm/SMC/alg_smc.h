#ifndef ALG_SMC_H
#define ALG_SMC_H

#include <cstdint>

// 幂次滑模面 + 近零线性段；参数默认全零，必须 Init 成功后才能计算。
struct Struct_SMC_Config
{
    float C = 0.0f;                 // 滑模面角度权重，>0，1/s。
    float K = 0.0f;                 // 线性趋近增益，>=0，1/s。
    float Epsilon = 0.0f;           // 切换项强度，>=0，rad/s²。
    float Phi = 0.0f;               // 边界层宽度，>0，rad/s。
    float J = 0.0f;                 // 输出换算增益，>0，输出单位/(rad/s²)。
    uint16_t P = 0U;                // 幂次分母，>0。
    uint16_t Q = 0U;                // 幂次分子，>0；Q=P 退回线性，Q>P 增强大误差纠正。
    float Linear_Threshold = 0.0f;  // 线性段半宽，>0，rad。
    float Out_Max = 0.0f;           // 输出限幅，>0。
};

// 只负责计算，不依赖电机、反馈、时间、滤波或积分。
// 正输出应让轴向正方向加速；角度过零、机械限位和方向换算由调用方处理。
class Class_SMC
{
public:
    // 参数必须为有限数；初始化失败后禁用计算，直到下一次初始化成功。
    bool Init(const Struct_SMC_Config &config);
    bool Is_Initialized() const { return Initialized; }

    // error = 目标角度 - 当前角度，rad。
    // error_rate = 目标角速度 - 当前角速度，rad/s。
    // target_acceleration 为目标角加速度，rad/s²；保持位置时使用默认 0。
    // 返回限幅后的本拍输出；未初始化或计算异常返回 0，不保留上次结果。
    float Calculate(float error, float error_rate, float target_acceleration = 0.0f) const;

private:
    Struct_SMC_Config Config = {};
    bool Initialized = false;
};

#endif // ALG_SMC_H
