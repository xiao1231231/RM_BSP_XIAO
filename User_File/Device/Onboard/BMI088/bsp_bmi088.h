/**
 * @file    bsp_bmi088.h
 * @brief   BMI088 设备驱动 + VQF 姿态解算（整套 IMU 的顶层）
 *
 * @note    来源：H7_BSP（zzm / USTC-RoboWalker）。
 *
 *          它把三件事收在一个类里：
 *            ① 三路通道的状态机（加速度 / 陀螺 / 温度）——各自记录
 *               "就绪 → 传输中 → 完成/超时"，失败能定位到是哪一路卡了
 *            ② SPI 恢复：1ms 服务里检查每路是否超时、HAL 是否报错，
 *               需要时整体重启 SPI + DMA（不是只把某一路的状态清掉）
 *            ③ VQF 姿态解算：逐样本积分 + 有效样本才参与的重力修正
 *
 * ── 相对 H7 原版的移植改动 ──────────────────────────────────────────
 *   ① SPI2 → SPI1；片选/中断脚换成 C 板的 CS1_ACCEL(PA4)/CS1_GYRO(PB0)、
 *      INT1_Accel(PC4)/INT1_Gyro(PC5)
 *   ② 温控未移植：`BMI088_Accel.Init(true)` → `Init()`，
 *      128ms 回调里的加热 PID 调用已删除
 *   ③ 板级标定常量：**数据换成本板的（留空 = 不修正）**，理由见类内注释
 *   ④ 任务句柄对齐本工程 CubeMX 生成的名字 BMI088Handle
 *   ⑤ 时间戳接口对齐 Sys_Get_Micros()
 * ────────────────────────────────────────────────────────────────────
 */

#ifndef BSP_BMI088_H
#define BSP_BMI088_H

/* Includes ------------------------------------------------------------------*/

#include "Accel/bsp_bmi088_accel.h"
#include "Gyro/bsp_bmi088_gyro.h"
#include "alg_filter_vqf.h"
#include "alg_quaternion.h"

extern "C" {
#include "cmsis_os2.h"
}

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 一路通道的传输状态机
 *
 * @note  Ready_Flag 是"数据到了，可以发下一笔"；Transfering_Flag 是"这一笔正在飞"。
 *        Transfer_Start_Timestamp_Low32 + Transfer_Timeout_Armed 是超时判定用的：
 *        只有"转账真的启动成功"了才起算，避免把启动失败误判成超时。
 */
struct Struct_BMI088_Status
{
    bool Ready_Flag = false;
    bool Transfering_Flag = false;
    bool Update_Flag = false;
    uint64_t Ready_Timestamp = 0U;
    uint64_t Transfer_Ready_Timestamp = 0U;
    uint32_t Transfer_Start_Timestamp_Low32 = 0U;
    bool Transfer_Timeout_Armed = false;
    uint64_t Update_Timestamp = 0U;
    uint64_t Update_Ready_Timestamp = 0U;
};

/** 加速度更新被拒的原因（目前只有"数据无效"一种） */
enum Enum_BMI088_Accel_Reject_Reason : uint8_t
{
    BMI088_ACCEL_REJECT_NONE = 0U,
    BMI088_ACCEL_REJECT_INVALID = 1U << 0,
};

/** SPI 恢复的原因位掩码（可以同时命中多条，出问题看这个就知道卡在哪一路） */
enum Enum_BMI088_SPI_Recovery_Reason : uint8_t
{
    BMI088_SPI_RECOVERY_NONE = 0U,
    BMI088_SPI_RECOVERY_ACCEL_TIMEOUT = 1U << 0,
    BMI088_SPI_RECOVERY_GYRO_TIMEOUT = 1U << 1,
    BMI088_SPI_RECOVERY_TEMPERATURE_TIMEOUT = 1U << 2,
    BMI088_SPI_RECOVERY_HAL_ERROR = 1U << 3,
    BMI088_SPI_RECOVERY_ACCEL_START_FAILURE = 1U << 4,
    BMI088_SPI_RECOVERY_GYRO_START_FAILURE = 1U << 5,
    BMI088_SPI_RECOVERY_TEMPERATURE_START_FAILURE = 1U << 6,
};

/** VQF 的初始化配置（参数 + 两个名义周期） */
struct Struct_BMI088_VQF_Config
{
    ///< VQF 姿态修正、零偏估计和静止检测参数
    Struct_VQF_Parameter Parameter;
    ///< 陀螺仪名义更新周期，单位 s；陀螺按 2kHz 采样 → 0.0005 s
    float Gyro_D_T = 0.0005f;
    ///< 加速度计名义更新周期，单位 s；加速度按 250Hz 采样 → 0.004 s
    float Accel_D_T = 0.004f;
};

class Class_BMI088
{
public:
    Class_BMI088_Accel BMI088_Accel;
    Class_BMI088_Gyro BMI088_Gyro;

    /**
     * @brief 设置 BMI088 内部 VQF 的初始化配置
     * @note  ★ 必须在 Init() 之前调用；初始化之后调用会被忽略
     *        （运行中改采样周期或滤波参数会破坏滤波器内部状态）
     */
    void Set_VQF_Config(const Struct_BMI088_VQF_Config &__Config);

    /**
     * @brief 把外部标定出的零偏初值喂给 VQF（★ 必须在 Init() 之后调用）
     * @param __Bias 陀螺零偏，单位 rad/s
     * @note  为什么要这个接口：上游用的是"板级标定常量"，换板子就不准；
     *        本工程改成【开机静止采样自标定】，标出来的值从这里喂进去，
     *        在线估计就只需要跟温漂的残余 —— 上电零漂能立刻降一个量级。
     */
    void Set_VQF_Bias_Estimate(const Class_Matrix_f32<3, 1> &__Bias);

    bool Init();
    bool Is_Initialized() const { return Init_Finished_Flag; }

    /** 解算一帧（从陀螺样本队列取一个样本，跑一次 VQF）——由 BMI088_Task 调用 */
    void Calculate();

    void SPI_RxCpltCallback();
    void EXTI_Flag_Callback(uint16_t __GPIO_Pin);
    void TIM_128ms_Calculate_PeriodElapsedCallback();
    void TIM_1ms_Service_PeriodElapsedCallback();
    void BMI088_Service_Transfer(const bool &__Allow_Recovery = false);

    /* ── 姿态与原始数据 ── */
    inline Class_Matrix_f32<3, 1> Get_Original_Accel() const;
    inline Class_Matrix_f32<3, 1> Get_Original_Gyro() const;
    inline Class_Matrix_f32<3, 1> Get_Fixed_Corrected_Gyro() const;
    inline Class_Matrix_f32<3, 1> Get_Fixed_Gyro_Offset() const;
    inline Class_Matrix_f32<3, 1> Get_Euler_Angle() const;
    inline Class_Matrix_f32<3, 3> Get_Rotation_Matrix() const;
    inline Class_Matrix_f32<4, 1> Get_Axis_Angle() const;
    inline Class_Quaternion_f32 Get_Quaternion() const;
    inline Class_Matrix_f32<3, 1> Get_Accel_Body() const;
    inline Class_Matrix_f32<3, 1> Get_Gyro_Body() const;
    inline Class_Matrix_f32<3, 1> Get_Accel() const;
    inline Class_Matrix_f32<3, 1> Get_Gyro() const;

    /* ── 诊断量（本来是给波形/调试器看的）── */
    inline float Get_Accel_Norm() const;
    inline uint32_t Get_Accel_Update_Result() const;
    inline uint32_t Get_Accel_Update_Rejected_Counter() const;
    inline uint32_t Get_Accel_Update_Attempt_Counter() const;
    inline uint32_t Get_SPI_Recovery_Counter() const;
    inline uint32_t Get_SPI_Transfer_Timeout_Counter() const;
    inline uint32_t Get_SPI_Accel_Timeout_Counter() const;
    inline uint32_t Get_SPI_Gyro_Timeout_Counter() const;
    inline uint32_t Get_SPI_Temperature_Timeout_Counter() const;
    inline uint8_t Get_SPI_Recovery_Last_Reason() const;
    inline uint32_t Get_Sensor_Ready_Gap_Counter() const;
    inline uint32_t Get_Timestamp_Anomaly_Counter() const;
    inline float Get_D_T() const;
    inline uint64_t Get_Calculating_Time() const;
    inline uint64_t Get_Last_Sample_Timestamp_Us() const;

    /* ── VQF 相关 ── */
    inline uint32_t Get_VQF_Reset_Counter() const;
    inline Class_Matrix_f32<3, 1> Get_VQF_Gyro_Bias() const;
    inline float Get_VQF_Gyro_Bias_Sigma() const;
    inline bool Get_VQF_Rest_Detected() const;
    inline Class_Matrix_f32<2, 1> Get_VQF_Relative_Rest_Deviation() const;
    inline float Get_VQF_Accel_Correction_Rate() const;

protected:
    /** 关中断取一份结构体/矩阵的快照，避免读到"改了一半"的值 */
    template<typename Data_Type>
    inline Data_Type Get_Atomic_Copy(const Data_Type &__Data) const;

    Struct_SPI_Manage_Object *SPI_Manage_Object = nullptr;

    /** 一笔传输超过这么久还没完成，就判超时并触发整体恢复。
     *  ★ 本工程把上游的 1000µs 放宽到 5000µs：
     *    · 陀螺一次 FIFO 批量读最多 511 字节，10.5MHz 下光走线就要 490µs
     *    · 再叠加中断延迟（F407 @168MHz、无 D-Cache，比 H7 慢 2.9 倍）
     *    → 1ms 阈值只有 2 倍余量，会被正常的 CPU 抖动误判成"卡死"
     *  5ms 仍然能很快抓住真正的卡死（5ms = 10 个陀螺样本，FIFO 完全吸收得起） */
    static constexpr uint32_t TRANSFERING_TIMEOUT = 5000U;
    /** 两个样本间隔超过 0.1s，说明中间"断了"，VQF 必须重置而不是硬积分 */
    static constexpr float D_T_TIMEOUT_THRESHOLD = 0.1f;

    /* ── 板级标定常量 ────────────────────────────────────────────────
     * ⚠️ H7_BSP 原版里这三个是【他们那块板子】的实测标定值
     *    （加速度计仿射矩阵 + 零偏、陀螺零偏）。换一块板子这些数就不对了：
     *      · 照抄他们的陀螺零偏 = -0.009 rad/s（X 轴约 -0.52°/s），
     *        会让 Yaw 每分钟漂 ~30° —— 比不修正还糟
     *      · 所以这里先留成"单位矩阵 / 零"，等于【不修正】
     *    不修正没关系，因为：
     *      · 陀螺的慢零偏由 VQF 在线估计接管（参数里给了足够快的初始收敛）
     *      · 加速度计的标定要重新标一块板子，标出来把数填回来即可
     *    用法（在哪里乘、什么时候加）和上游完全一致，只是数据换成本板的。
     * ────────────────────────────────────────────────────────────── */
    const float ACCEL_AFFINE_DATA[9] = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f};
    const float ACCEL_BIAS_DATA[3] = {0.0f, 0.0f, 0.0f};
    const float GYRO_ZERO_OFFSET[3] = {0.0f, 0.0f, 0.0f};

    Struct_BMI088_VQF_Config VQF_Config;
    Class_Filter_VQF Filter_VQF;
    uint32_t VQF_Reset_Counter = 0U;
    uint64_t VQF_Pre_Timestamp = 0U;

    bool Init_Finished_Flag = false;
    Struct_BMI088_Status Accel_Status;
    Struct_BMI088_Status Gyro_Status;
    Struct_BMI088_Status Temperature_Status;
    uint8_t Transfer_Priority_Index = 0U;
    volatile bool Transfer_Service_Active = false;
    uint64_t Gyro_FIFO_Last_Fallback_Poll_Timestamp = 0U;

    Class_Matrix_f32<3, 1> Vector_Pending_Accel;
    uint64_t Pending_Accel_Timestamp = 0U;
    bool Accel_Observation_Pending = false;
    bool Pending_Accel_Valid = false;

    float D_T = 0.0005f;
    Class_Matrix_f32<3, 1> Vector_Original_Accel;
    Class_Matrix_f32<3, 1> Vector_Original_Gyro;
    Class_Matrix_f32<3, 1> Vector_Fixed_Corrected_Gyro;
    Class_Matrix_f32<3, 1> Vector_Euler_Angle;
    Class_Matrix_f32<3, 3> Matrix_Rotation;
    Class_Matrix_f32<4, 1> Vector_Axis_Angle;
    Class_Quaternion_f32 Quarternion;
    Class_Matrix_f32<3, 1> Vector_Accel_Body;
    Class_Matrix_f32<3, 1> Vector_Gyro_Body;
    Class_Matrix_f32<3, 1> Vector_Accel;
    Class_Matrix_f32<3, 1> Vector_Gyro;

    float Accel_Norm = 0.0f;
    uint32_t Accel_Update_Result = 0U;
    uint32_t Accel_Update_Attempt_Counter = 0U;
    uint32_t Accel_Update_Rejected_Counter = 0U;

    uint32_t SPI_Recovery_Counter = 0U;
    /** 连续软恢复次数：超过阈值才升级成重量级恢复（abort + DMA 复位） */
    uint32_t SPI_Soft_Recovery_Streak = 0U;
    /** 连续启动失败次数：同样只在超过阈值时才升级 */
    uint32_t SPI_Start_Failure_Streak = 0U;
    static constexpr uint32_t BMI088_SOFT_RECOVERY_ESCALATE_STREAK = 20U;
    static constexpr uint32_t BMI088_START_FAILURE_ESCALATE_STREAK = 20U;
    uint32_t SPI_Transfer_Timeout_Counter = 0U;
    uint32_t SPI_Accel_Timeout_Counter = 0U;
    uint32_t SPI_Gyro_Timeout_Counter = 0U;
    uint32_t SPI_Temperature_Timeout_Counter = 0U;
    volatile uint8_t SPI_Recovery_Pending_Reason = BMI088_SPI_RECOVERY_NONE;
    uint8_t SPI_Recovery_Last_Reason = BMI088_SPI_RECOVERY_NONE;
    uint32_t Sensor_Ready_Gap_Counter = 0U;
    uint32_t Timestamp_Anomaly_Counter = 0U;
    uint64_t Calculating_Time = 0U;

    void BMI088_Recover_SPI(uint8_t __Reason);
    /** 软恢复：只清 HAL 的错误码，不做 abort、不碰 DMA（理由见 .cpp 注释） */
    void BMI088_Soft_Recover_SPI();
    void BMI088_Service_Transfer_Locked(const bool &__Allow_Recovery);
    void Set_Accel_Update_Result(const bool &__Accepted,
                                 const uint8_t &__Reject_Reason);
};

/** 全局唯一实例 */
extern Class_BMI088 BSP_BMI088;

/* Exported functions --------------------------------------------------------*/

template<typename Data_Type>
inline Data_Type Class_BMI088::Get_Atomic_Copy(const Data_Type &__Data) const
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const Data_Type data = __Data;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return data;
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Original_Accel() const
{
    return Get_Atomic_Copy(Vector_Original_Accel);
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Original_Gyro() const
{
    return Get_Atomic_Copy(Vector_Original_Gyro);
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Fixed_Corrected_Gyro() const
{
    return Get_Atomic_Copy(Vector_Fixed_Corrected_Gyro);
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Fixed_Gyro_Offset() const
{
    return Class_Matrix_f32<3, 1>(GYRO_ZERO_OFFSET);
}

/** Euler 角顺序为 [Yaw, Pitch, Roll]（标准 ZYX），单位弧度 */
inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Euler_Angle() const
{
    return Get_Atomic_Copy(Vector_Euler_Angle);
}

inline Class_Matrix_f32<3, 3> Class_BMI088::Get_Rotation_Matrix() const
{
    return Get_Atomic_Copy(Matrix_Rotation);
}

inline Class_Matrix_f32<4, 1> Class_BMI088::Get_Axis_Angle() const
{
    return Get_Atomic_Copy(Vector_Axis_Angle);
}

inline Class_Quaternion_f32 Class_BMI088::Get_Quaternion() const
{
    return Get_Atomic_Copy(Quarternion);
}

/** 机体系加速度（已扣掉重力：纯运动加速度） */
inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Accel_Body() const
{
    return Get_Atomic_Copy(Vector_Accel_Body);
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Gyro_Body() const
{
    return Get_Atomic_Copy(Vector_Gyro_Body);
}

/** 地理系加速度 */
inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Accel() const
{
    return Get_Atomic_Copy(Vector_Accel);
}

/** 地理系角速度 */
inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Gyro() const
{
    return Get_Atomic_Copy(Vector_Gyro);
}

inline float Class_BMI088::Get_Accel_Norm() const
{
    return Accel_Norm;
}

inline uint32_t Class_BMI088::Get_Accel_Update_Result() const
{
    return Accel_Update_Result;
}

inline uint32_t Class_BMI088::Get_Accel_Update_Rejected_Counter() const
{
    return Accel_Update_Rejected_Counter;
}

inline uint32_t Class_BMI088::Get_Accel_Update_Attempt_Counter() const
{
    return Accel_Update_Attempt_Counter;
}

inline uint32_t Class_BMI088::Get_SPI_Recovery_Counter() const
{
    return SPI_Recovery_Counter;
}

inline uint32_t Class_BMI088::Get_SPI_Transfer_Timeout_Counter() const
{
    return SPI_Transfer_Timeout_Counter;
}

inline uint32_t Class_BMI088::Get_SPI_Accel_Timeout_Counter() const
{
    return SPI_Accel_Timeout_Counter;
}

inline uint32_t Class_BMI088::Get_SPI_Gyro_Timeout_Counter() const
{
    return SPI_Gyro_Timeout_Counter;
}

inline uint32_t Class_BMI088::Get_SPI_Temperature_Timeout_Counter() const
{
    return SPI_Temperature_Timeout_Counter;
}

inline uint8_t Class_BMI088::Get_SPI_Recovery_Last_Reason() const
{
    return SPI_Recovery_Last_Reason;
}

inline uint32_t Class_BMI088::Get_Sensor_Ready_Gap_Counter() const
{
    return Sensor_Ready_Gap_Counter;
}

inline uint32_t Class_BMI088::Get_Timestamp_Anomaly_Counter() const
{
    return Timestamp_Anomaly_Counter;
}

inline float Class_BMI088::Get_D_T() const
{
    return D_T;
}

inline uint64_t Class_BMI088::Get_Last_Sample_Timestamp_Us() const
{
    return Get_Atomic_Copy(VQF_Pre_Timestamp);
}

inline uint64_t Class_BMI088::Get_Calculating_Time() const
{
    return Get_Atomic_Copy(Calculating_Time);
}

inline uint32_t Class_BMI088::Get_VQF_Reset_Counter() const
{
    return VQF_Reset_Counter;
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_VQF_Gyro_Bias() const
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const Class_Matrix_f32<3, 1> bias = Filter_VQF.Get_Bias_Estimate();
    if (primask == 0U)
    {
        __enable_irq();
    }
    return bias;
}

inline float Class_BMI088::Get_VQF_Gyro_Bias_Sigma() const
{
    return Filter_VQF.Get_Bias_Sigma();
}

inline bool Class_BMI088::Get_VQF_Rest_Detected() const
{
    return Filter_VQF.Get_Rest_Detected();
}

inline Class_Matrix_f32<2, 1> Class_BMI088::Get_VQF_Relative_Rest_Deviation() const
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const Class_Matrix_f32<2, 1> deviation =
        Filter_VQF.Get_Relative_Rest_Deviation();
    if (primask == 0U)
    {
        __enable_irq();
    }
    return deviation;
}

inline float Class_BMI088::Get_VQF_Accel_Correction_Rate() const
{
    return Filter_VQF.Get_Last_Accel_Correction_Rate();
}

#ifdef __cplusplus
extern "C" {
#endif

/* 供 1ms 任务的周期回调表调用（名字和 H7_BSP 一致） */
void BMI088_TIM_128ms_Calculate_PeriodElapsedCallback();
void BMI088_TIM_1ms_Service_PeriodElapsedCallback();

#ifdef __cplusplus
}
#endif

#endif /* BSP_BMI088_H */

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
