/**
 * @file    bsp_bmi088_accel.h
 * @brief   BMI088 加速度计 —— 配置、数据读取、温度读取
 *
 * @note    来源：H7_BSP（USTC-RoboWalker / yssickjgd）。
 *
 * ── 相对 H7 原版的移植改动 ──────────────────────────────────────────
 *   ① SPI2 → SPI1，片选换成 C 板的 CS1_ACCEL（PA4）
 *   ② **温控方案不照 H7，照 basic_framework（C 板官方工程）**：
 *      C 板手册附表 TIM10_CH1 = PF6 就是给 IMU 恒温用的加热电阻（5V / 0.58W）。
 *      H7 版含"按电池电压补偿占空比"，C 板官方没有这一步，本模块也不做。
 *      参数按官方取值、再按本工程 PWM 的 ARR 等比换算（见下方 HEATER_*）。
 *   ③ 温度读取与有效性判定【保留】—— 温漂是零偏漂移的主因，看得到才能判。
 *   ④ 延时/时间戳接口对齐本工程。
 * ────────────────────────────────────────────────────────────────────
 */

#ifndef BSP_BMI088_ACCEL_H
#define BSP_BMI088_ACCEL_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088_accel_register.h"
#include "bsp_spi.h"
#include "alg_matrix.h"
#include "alg_pid.h"
#include "tim.h"        /* htim10 —— 加热 PWM */

/* Exported macros -----------------------------------------------------------*/

/* ── C 板引脚映射（手册附表）────────────────────────────────────────
 *   INT1_Accel = PC4 —— 加速度计 INT1，推挽输出，对应 MCU 上升沿 EXTI
 *   CS1_Accel  = PA4 —— CubeMX 已给标签（CS1_ACCEL_Pin / CS1_ACCEL_GPIO_Port）
 *   加热片     = TIM10_CH1（PF6），100Hz PWM，Compare 越大越热
 *   （C 板手册：高电平加热，功率 0.58W@5V，无电池电压补偿一说） */
#define BMI088_ACCEL_INT_GPIO_Port  GPIOC
#define BMI088_ACCEL_INT_Pin        GPIO_PIN_4
#define BMI088_HEAT_TIM             (&htim10)
#define BMI088_HEAT_CHANNEL         TIM_CHANNEL_1

/* Exported types ------------------------------------------------------------*/

/** 加速度计量程 */
enum Enum_BSP_BMI088_Accel_Range : uint8_t
{
    BMI088_ACCEL_RANGE_3G = 0x00,
    BMI088_ACCEL_RANGE_6G,
    BMI088_ACCEL_RANGE_12G,
    BMI088_ACCEL_RANGE_24G,
};

/** 温度的一次性快照（含"多久没更新了"和"还能不能信"） */
struct Struct_BMI088_Accel_Temperature_State
{
    float Temperature = 0.0f;
    uint64_t Now_Timestamp_Us = 0U;
    uint64_t Last_Valid_Timestamp_Us = 0U;
    uint32_t Age_Us = 0xffffffffU;
    bool Data_Valid = false;
};

/**
 * @brief 加速度计类
 */
class Class_BMI088_Accel
{
public:
    // 每个通信/配置步骤最多尝试 5 次，失败返回 false。
    bool Init();

    /**
     * @brief IMU 恒温控制（500Hz，由 IMU_Service_Task 分频调用）
     * @note  方案照搬 basic_framework（C 板官方工程）：
     *          纯 PID 直出 PWM 占空比 —— 温度低就加热多，到目标就少给。
     *          没有独立的"预热阶段"，全靠 PID 积分从 0 爬上来
     *          （积分限幅限制了爬升速度，等效于缓慢预热，不会过冲太多）。
     *        ★ 目标温度 40°C：C 板官方的取值。再高收益递减（温漂变小变慢），
     *          而夏天 40°C 起点近、加热负担小。
     */
    void Heater_Control();

    /** 恒温是否已使能（Init 里决定） */
    inline bool Get_Heater_Enable() const;

    /** 最近一次输出的 PWM Compare（调试观察用） */
    inline uint32_t Get_Heater_PWM_Compare() const;

    inline float Get_Now_Temperature() const;

    uint32_t Get_Temperature_Outlier_Counter() const;

    uint32_t Get_Accel_Invalid_Counter() const;

    Struct_BMI088_Accel_Temperature_State Get_Temperature_State() const;

    bool Get_Temperature_Valid_Flag() const;

    uint32_t Get_Temperature_Age_Us() const;

    inline bool Get_Valid_Flag() const;

    inline Class_Matrix_f32<3, 1> Get_Raw_Accel() const;

    /** SPI 收完一笔时由上层分发进来 */
    void SPI_RxCpltCallback();

    uint8_t SPI_Request_Accel();

    uint8_t SPI_Request_Temperature();

protected:
    /* ── 绑定 ── */
    Struct_SPI_Manage_Object *SPI_Manage_Object;
    GPIO_TypeDef *CS_GPIO_Port;
    uint16_t CS_Pin;
    GPIO_PinState Activate_Pin_State;

    /* ── 常量 ── */
    const uint8_t BMI088_ACCEL_READ_MASK = 0x80;
    // 加速度计读数据模式下, SPI 发完地址后还要再发 1 字节保留字节, 该字节会被接收但忽略
    const uint8_t BMI088_ACCEL_SPI_RX_RESERVED = 1;
    const uint8_t BMI088_ACCEL_INIT_INSTRUCTION_NUM = 6;
    // 量程 ±24g
    const Enum_BSP_BMI088_Accel_Range BMI088_ACCEL_RANGE = BMI088_ACCEL_RANGE_24G;

    /** 初始化要写的 6 个寄存器：{偏移, 值} */
    const uint8_t BMI088_ACCEL_REGISTER_CONFIG[6][2] = {
        // 开启加速度计电源
        {offsetof(Struct_BMI088_Accel_Register, ACC_PWR_CTRL_RW), 0x04},
        // 从默认挂起状态 0x03 切到工作状态 0x00
        {offsetof(Struct_BMI088_Accel_Register, ACC_PWR_CONF_RW), 0x00},
        // 无滤波器, 频率 1600Hz
        {offsetof(Struct_BMI088_Accel_Register, ACC_CONF_RW), (0x0a << 4) | 0x0c},
        // 量程
        {offsetof(Struct_BMI088_Accel_Register, ACC_RANGE_RW), BMI088_ACCEL_RANGE},
        // INT1 推挽输出
        {offsetof(Struct_BMI088_Accel_Register, INT1_IO_CTRL_RW), 0x01 << 3},
        // 数据就绪就拉 INT1
        {offsetof(Struct_BMI088_Accel_Register, INT_MAP_DATA_RW), 0x01 << 2},
    };

    /** 温度数据超过这么久没更新就判为过期 */
    uint32_t TEMPERATURE_STALE_TIMEOUT_US = 500000U;
    /** 温度跳变判野值后，要连续这么多帧一致才重新采信 */
    uint8_t TEMPERATURE_REBASE_SAMPLE_COUNT = 3U;

    /* ── 恒温（参数来自 basic_framework，按本工程 ARR=9999 等比换算）────
     *   原版（C 板官方）：Kp=1000, Ki=20, Kd=0，MaxOut=2000（他们的 ARR≈2000）
     *   本工程 ARR=9999，是原版的 ~5 倍 → Kp/Ki/MaxOut 等比 ×5：
     *     Kp 1000→5000, Ki 20→100, MaxOut 2000→9999, IntegralLimit 300→1500
     *   等比换算后 PID 的动态特性不变（同样的误差 → 同样的占空比比例）。 */
    static constexpr float HEATER_KP = 5000.0f;
    static constexpr float HEATER_KI = 100.0f;
    static constexpr float HEATER_TARGET_TEMPERATURE = 40.0f;
    static constexpr float HEATER_OUT_MAX = 9999.0f;
    static constexpr float HEATER_I_OUT_MAX = 1500.0f;
    /** 恒温名义计算周期：IMU_Service_Task 每 2 次周期服务调用一次。 */
    static constexpr float HEATER_D_T = 0.002f;

    /** 恒温 PID */
    Class_PID PID_Temperature;
    /** 是否使能恒温（Init 决定；PWM 启动失败自动关闭） */
    bool Heater_Enable = false;
    /** 最近一次写进 PWM 的 Compare */
    volatile uint32_t Heater_PWM_Compare = 0U;

    /* ── 状态 ── */

    Struct_BMI088_Accel_Register Register = {0};

    volatile float Now_Temperature = 0.0f;
    float Temperature_Rebase_Candidate = 0.0f;
    volatile uint64_t Temperature_Last_Valid_Timestamp = 0U;
    uint8_t Temperature_Rebase_Count = 0U;
    volatile bool Temperature_Valid_Flag = false;

    bool Valid_Flag = true;
    Class_Matrix_f32<3, 1> Vector_Raw_Accel;

    /* ── 内部函数 ── */

    void Read_Single_Register(const uint8_t &Register_Address) const;

    void Read_Multi_Register(const uint8_t &Register_Address, const uint32_t &Rx_Length) const;

    void Write_Single_Register(const uint8_t &Register_Address, const uint8_t *Tx_Data_Buffer) const;
};

/* Exported variables --------------------------------------------------------*/

/** 标准重力加速度（加速度换算要用） */
extern const float GRAVITY_ACCELERATION;

/* Exported function declarations --------------------------------------------*/

inline float Class_BMI088_Accel::Get_Now_Temperature() const
{
    return (Now_Temperature);
}

inline bool Class_BMI088_Accel::Get_Heater_Enable() const
{
    return (Heater_Enable);
}

inline uint32_t Class_BMI088_Accel::Get_Heater_PWM_Compare() const
{
    return (Heater_PWM_Compare);
}

inline bool Class_BMI088_Accel::Get_Valid_Flag() const
{
    return (Valid_Flag);
}

/** 读原始加速度：关中断取快照，避免读到写了一半的值 */
inline Class_Matrix_f32<3, 1> Class_BMI088_Accel::Get_Raw_Accel() const
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const Class_Matrix_f32<3, 1> accel = Vector_Raw_Accel;
    __set_PRIMASK(primask);
    return accel;
}

#endif /* BSP_BMI088_ACCEL_H */

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
