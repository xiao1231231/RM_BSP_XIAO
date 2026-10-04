/**
 * @file    alg_pid.h
 * @brief   PID 算法（位置式 + 积分限幅 + 死区 + 变速积分 + 积分分离 + 前馈 + D 滤波）
 *
 * @note    来源：H7_BSP（USTC-RoboWalker / yssickjgd）。本工程的第一个用户是
 *          IMU 恒温，之后电机闭环也用它 —— 所以完整移植（没有被恒温用到的
 *          特性如变速积分也保留，它们是调参时的工具）。
 *
 *          当前参数都写在各自的配置处，不在本文件：恒温这路在
 *          bsp_bmi088_accel.h 的 HEATER_*（来自 basic_framework，按 ARR 等比换算）；
 *          电机速度环在 system_init.cpp 的 DJI_Motor_Create(...) 实参里。
 */

#ifndef ALG_PID_H
#define ALG_PID_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"
#include "alg_filter_iir.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/** 微分先行开关：开启后 D 项作用于测量值而不是误差（目标突变时不炸 D） */
enum Enum_PID_D_First
{
    PID_D_First_DISABLE = 0,
    PID_D_First_ENABLE,
};

/**
 * @brief PID 算法
 * @note  先 Init 后计算；D_T 必须与实际计算周期一致，否则 I/D 项的量纲全错。
 */
class Class_PID
{
public:
    /**
     * @brief 初始化（保留目标、输出及积分/微分历史）
     *
     * @param __K_P                    P 增益
     * @param __K_I                    I 增益（设 0 时下次计算清空积分）
     * @param __K_D                    D 增益
     * @param __K_F                    每周期目标增量前馈系数，不除以 D_T
     * @param __I_Out_Max              积分限幅（输出量纲），0 为不限制
     * @param __Out_Max                输出限幅，0 为不限制
     * @param __D_T                    计算周期，s
     * @param __Dead_Zone              非负死区：区内有效误差归零，区外扣除死区宽度
     * @param __I_Variable_Speed_A/B   变速积分两段阈值（均 0 关闭）
     * @param __I_Separate_Threshold   积分分离阈值（0 关闭，非 0 须为正）
     * @param __D_First                是否微分先行
     * @param __D_Filter_Cutoff        D 支路一阶 IIR 截止频率 Hz，0 关闭
     */
    void Init(const float &__K_P, const float &__K_I, const float &__K_D, const float &__K_F = 0.0f, const float &__I_Out_Max = 0.0f, const float &__Out_Max = 0.0f, const float &__D_T = 0.001f, const float &__Dead_Zone = 0.0f, const float &__I_Variable_Speed_A = 0.0f, const float &__I_Variable_Speed_B = 0.0f, const float &__I_Separate_Threshold = 0.0f, const Enum_PID_D_First &__D_First = PID_D_First_DISABLE, const float &__D_Filter_Cutoff = 0.0f);

    float Get_Target() const;
    float Get_Now() const;
    float Get_Error() const;

    inline float Get_Integral_Error() const;

    inline float Get_Out() const;

    inline float Get_K_P() const;
    inline float Get_K_I() const;
    inline float Get_K_D() const;
    inline float Get_K_F() const;
    inline float Get_I_Out_Max() const;
    inline float Get_Out_Max() const;
    inline float Get_I_Variable_Speed_A() const;
    inline float Get_I_Variable_Speed_B() const;
    inline float Get_I_Separate_Threshold() const;
    inline float Get_D_T() const;
    inline Enum_PID_D_First Get_D_First() const;
    inline float Get_D_Filter_Cutoff() const;

    inline void Set_K_P(const float &__K_P);
    inline void Set_K_I(const float &__K_I);
    inline void Set_K_D(const float &__K_D);
    inline void Set_K_F(const float &__K_F);
    inline void Set_I_Out_Max(const float &__I_Out_Max);
    inline void Set_Out_Max(const float &__Out_Max);
    inline void Set_I_Variable_Speed_A(const float &__I_Variable_Speed_A);
    inline void Set_I_Variable_Speed_B(const float &__I_Variable_Speed_B);
    inline void Set_I_Separate_Threshold(const float &__I_Separate_Threshold);
    inline void Set_Target(const float &__Target);
    inline void Set_Now(const float &__Now);
    inline void Set_Integral_Error(const float &__Integral_Error);

    /** 计算一拍（调用周期必须 = Init 里的 D_T） */
    void TIM_Calculate_PeriodElapsedCallback();

protected:
    /* 初始化相关常量 */

    // 计算周期, s
    float D_T;
    // 非负死区阈值：区内有效误差归零，区外扣除死区宽度；不改 Target
    float Dead_Zone;
    // 微分先行
    Enum_PID_D_First D_First = PID_D_First_DISABLE;
    // D 支路低通请求截止频率, Hz；0 关闭
    float D_Filter_Cutoff = 0.0f;

    // 内部变量

    // 之前的当前值
    float Pre_Now = 0.0f;
    // 之前的目标值
    float Pre_Target = 0.0f;
    // 之前的输出值
    float Pre_Out = 0.0f;
    // 上一拍（死区处理后）的误差
    float Pre_Error = 0.0f;
    // D 支路的低通（D_Filter_Cutoff 为 0 时不参与计算）
    Class_Filter_IIR_First_Order D_Filter;

    // 读变量

    // 输出值
    float Out = 0.0f;

    // 写变量

    // PID的P
    float K_P = 0.0f;
    // PID的I，设为零时在下一次计算中清空积分
    float K_I = 0.0f;
    // PID的D
    float K_D = 0.0f;
    // 每周期目标增量前馈系数：K_F * (Target - Pre_Target)，不除以 D_T
    float K_F = 0.0f;

    // 积分输出幅值上限（输出量纲），每次累加后限幅，0 为不限制
    float I_Out_Max = 0;
    // 输出限幅, 0 为不限制
    float Out_Max = 0;

    // 变速积分内段阈值 A；A、B 均为零时关闭，启用线性段时要求 0 <= A < B
    float I_Variable_Speed_A = 0.0f;
    // 变速积分外段阈值 B（不是区间宽度）
    float I_Variable_Speed_B = 0.0f;
    // 积分分离阈值，正数启用、零关闭；达到阈值时清空积分
    float I_Separate_Threshold = 0.0f;

    // 目标值
    float Target = 0.0f;
    // 当前值
    float Now = 0.0f;

    // 读写变量

    // 积分值
    float Integral_Error = 0.0f;
};

/* Exported function declarations --------------------------------------------*/

inline float Class_PID::Get_Integral_Error() const
{
    return (Integral_Error);
}

inline float Class_PID::Get_Out() const
{
    return (Out);
}

inline void Class_PID::Set_K_P(const float &__K_P)
{
    K_P = __K_P;
}

inline void Class_PID::Set_K_I(const float &__K_I)
{
    K_I = __K_I;
}

inline void Class_PID::Set_K_D(const float &__K_D)
{
    K_D = __K_D;
}

inline void Class_PID::Set_K_F(const float &__K_F)
{
    K_F = __K_F;
}

inline void Class_PID::Set_I_Out_Max(const float &__I_Out_Max)
{
    I_Out_Max = __I_Out_Max;
}

inline void Class_PID::Set_Out_Max(const float &__Out_Max)
{
    Out_Max = __Out_Max;
}

inline void Class_PID::Set_I_Variable_Speed_A(const float &__I_Variable_Speed_A)
{
    I_Variable_Speed_A = __I_Variable_Speed_A;
}

inline void Class_PID::Set_I_Variable_Speed_B(const float &__I_Variable_Speed_B)
{
    I_Variable_Speed_B = __I_Variable_Speed_B;
}

inline void Class_PID::Set_I_Separate_Threshold(const float &__I_Separate_Threshold)
{
    I_Separate_Threshold = __I_Separate_Threshold;
}

inline void Class_PID::Set_Target(const float &__Target)
{
    Target = __Target;
}

inline void Class_PID::Set_Now(const float &__Now)
{
    Now = __Now;
}

inline void Class_PID::Set_Integral_Error(const float &__Integral_Error)
{
    Integral_Error = __Integral_Error;
}

inline float Class_PID::Get_K_P() const { return K_P; }

inline float Class_PID::Get_K_I() const { return K_I; }

inline float Class_PID::Get_K_D() const { return K_D; }

inline float Class_PID::Get_K_F() const { return K_F; }

inline float Class_PID::Get_I_Out_Max() const { return I_Out_Max; }

inline float Class_PID::Get_Out_Max() const { return Out_Max; }

inline float Class_PID::Get_I_Variable_Speed_A() const { return I_Variable_Speed_A; }

inline float Class_PID::Get_I_Variable_Speed_B() const { return I_Variable_Speed_B; }

inline float Class_PID::Get_I_Separate_Threshold() const { return I_Separate_Threshold; }

inline float Class_PID::Get_D_T() const { return D_T; }

inline Enum_PID_D_First Class_PID::Get_D_First() const { return D_First; }

inline float Class_PID::Get_D_Filter_Cutoff() const { return D_Filter_Cutoff; }

#endif /* ALG_PID_H */

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
