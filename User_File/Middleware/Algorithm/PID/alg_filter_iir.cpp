/**
 * @file    alg_filter_iir.cpp
 * @brief   一阶 IIR 低通滤波器实现
 *
 * @note    来源：H7_BSP（USTC-RoboWalker / zzm）。未做逻辑改动。
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_filter_iir.h"

#include <cmath>

/* Function prototypes -------------------------------------------------------*/

void Class_Filter_IIR_First_Order::Init(
    const float &__Cutoff_Frequency,
    const float &__Sampling_Frequency)
{
    if (__Cutoff_Frequency <= 0.0f || __Sampling_Frequency <= 0.0f)
    {
        // 参数无效时旁路滤波器（α=1 即直通），避免产生 NaN 或冻结输出。
        Alpha = 1.0f;
    }
    else
    {
        const float cutoff_frequency = Basic_Math_Constrain(
            __Cutoff_Frequency,
            0.0f,
            __Sampling_Frequency / 2.0f);

        Alpha = 1.0f - std::exp(
                           -2.0f * PI * cutoff_frequency /
                           __Sampling_Frequency);
        Alpha = Basic_Math_Constrain(Alpha, 0.0f, 1.0f);
    }

    Now = 0.0f;
    Out = 0.0f;
    Initialized_Flag = false;
}

void Class_Filter_IIR_First_Order::TIM_Calculate_PeriodElapsedCallback()
{
    if (!Initialized_Flag)
    {
        return;
    }

    Out += Alpha * (Now - Out);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
