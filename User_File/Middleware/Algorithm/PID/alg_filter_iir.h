/**
 * @file    alg_filter_iir.h
 * @brief   一阶 IIR 低通滤波器
 *
 * @note    来源：H7_BSP（USTC-RoboWalker / zzm），只保留了 PID 库依赖的一阶低通
 *          （二阶陷波/低通未移植——本工程暂时没有用例，要用再从上游摘）。
 *
 *          匹配极点法离散化：
 *            y[k] = y[k-1] + α·(x[k] - y[k-1])
 *            α = 1 - exp(-2π·fc/fs)
 *          这比"α = 2π·fc/fs"的近似法在 fs 不够高时更准确。
 */

#ifndef ALG_FILTER_IIR_H
#define ALG_FILTER_IIR_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"

/* Exported types ------------------------------------------------------------*/

/** 一阶 IIR 低通滤波器 */
class Class_Filter_IIR_First_Order
{
public:
    void Init(const float &__Cutoff_Frequency,
              const float &__Sampling_Frequency);

    inline float Get_Out() const;

    inline float Get_Alpha() const;

    inline bool Get_Initialized_Flag() const;

    /** 喂入本拍的新样本（首拍会直接对齐输入，避免从 0 爬升的启动冲击） */
    inline void Set_Now(const float &__Now);

    /** 把滤波状态强制置为某个值（例如清零、或对齐当前值） */
    inline void Reset(const float &__Value);

    /** 执行一拍滤波（先 Set_Now，再调这个） */
    void TIM_Calculate_PeriodElapsedCallback();

protected:
    /* 滤波系数（0~1，越大越"信新数据"） */
    float Alpha = 1.0f;

    /* 内部变量 */

    float Now = 0.0f;
    float Out = 0.0f;
    bool Initialized_Flag = false;
};

/* Exported function declarations --------------------------------------------*/

inline float Class_Filter_IIR_First_Order::Get_Out() const
{
    return Out;
}

inline float Class_Filter_IIR_First_Order::Get_Alpha() const
{
    return Alpha;
}

inline bool Class_Filter_IIR_First_Order::Get_Initialized_Flag() const
{
    return Initialized_Flag;
}

inline void Class_Filter_IIR_First_Order::Set_Now(const float &__Now)
{
    Now = __Now;

    // 首帧直接对齐输入，避免滤波输出从 0 缓慢爬升造成启动冲击。
    if (!Initialized_Flag)
    {
        Out = Now;
        Initialized_Flag = true;
    }
}

inline void Class_Filter_IIR_First_Order::Reset(const float &__Value)
{
    Now = __Value;
    Out = __Value;
    Initialized_Flag = true;
}

#endif /* ALG_FILTER_IIR_H */

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
