/**
 * @file    bsp_bmi088_gyro.h
 * @brief   BMI088 陀螺仪 —— 配置、FIFO 读取、样本队列
 *
 * @note    来源：H7_BSP（USTC-RoboWalker / yssickjgd）。
 *
 *          这块是整套 IMU 里最复杂的一环，核心是【FIFO + 时间戳重建】：
 *            ① 陀螺按 2kHz（每 500µs）产出数据，每帧就绪时拉 INT3
 *            ② EXTI → 先读 FIFO_STATUS 问"有几帧"，再一次性把 FIFO_DATA 读回来
 *            ③ 读回来的第 N 帧不是"现在"的时刻，而是"锚点 - 回推 N 帧"——
 *               代码用实测样本周期把每帧的时间戳重建出来，再入队
 *            ④ 陀螺任务逐样本出队、喂给 VQF
 *          这样时间戳是【传感器自己的采样时刻】，而不是"CPU 什么时候读的"，
 *          抖动不会污染积分。
 *
 * ── 相对 H7 原版的移植改动 ──────────────────────────────────────────
 *   ① 去掉 stm32h7xx_hal.h（F4 的 HAL 随 main.h 进来）
 *   ② SPI2 → SPI1；片选标签换成 C 板的 CS1_GYRO / 陀螺中断脚 PC5
 *   ③ 时间戳延时接口对齐本工程（见 .cpp）
 * ────────────────────────────────────────────────────────────────────
 */

#ifndef BSP_BMI088_GYRO_H
#define BSP_BMI088_GYRO_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088_gyro_register.h"
#include "bsp_spi.h"
#include "alg_matrix.h"

/* Exported macros -----------------------------------------------------------*/

/* ── C 板引脚映射（手册附表）────────────────────────────────────────
 *   INT1_Gyro = PC5 —— 陀螺的 INT3，配成推挽 + 高有效，对应 MCU 上升沿 EXTI
 *   CS1_Gyro  = PB0 —— CubeMX 已给标签（CS1_GYRO_Pin / CS1_GYRO_GPIO_Port）
 * 中断脚在 CubeMX 里没有标签，所以在这里显式定义。 */
#define BMI088_GYRO_INT_GPIO_Port   GPIOC
#define BMI088_GYRO_INT_Pin         GPIO_PIN_5

/* Exported types ------------------------------------------------------------*/

/** 陀螺仪量程 */
enum Enum_BSP_BMI088_Gyro_Range : uint8_t
{
    BMI088_GYRO_RANGE_2000DPS = 0x00,
    BMI088_GYRO_RANGE_1000DPS,
    BMI088_GYRO_RANGE_500DPS,
    BMI088_GYRO_RANGE_250DPS,
    BMI088_GYRO_RANGE_125DPS,
};

/** SPI 接收回调的返回位：告诉上层"接下来还要不要继续发下一笔传输" */
enum Enum_BSP_BMI088_Gyro_SPI_Result : uint8_t
{
    BMI088_GYRO_SPI_RESULT_NONE = 0U,
    BMI088_GYRO_SPI_RESULT_SAMPLES_QUEUED = 1U << 0,
    BMI088_GYRO_SPI_RESULT_FOLLOWUP_REQUIRED = 1U << 1,
};

/** 一个陀螺样本（带"传感器自己的采样时刻"和有效性） */
struct Struct_BMI088_Gyro_Sample
{
    uint64_t Timestamp_Us;
    float Gyro_Rad_S[3];
    uint32_t Sequence;
    uint8_t Valid;
    uint8_t Reserved[3];
};

static_assert(sizeof(Struct_BMI088_Gyro_Sample) == 32U,
              "BMI088 gyro sample ABI changed");

/**
 * @brief 陀螺仪类
 */
class Class_BMI088_Gyro
{
public:
    // 每个通信/配置步骤最多尝试 5 次，失败返回 false。
    bool Init();

    /** 初始化收尾：重写 FIFO 配置，冲掉启动阶段积累的旧数据 */
    void Start_FIFO_Acquisition();

    inline bool Get_Valid_Flag() const;

    inline Class_Matrix_f32<3, 1> Get_Raw_Gyro() const;

    uint32_t Get_Outlier_Counter() const;

    /** EXTI 里调用：只记"中断来了"的时刻，不碰 SPI */
    void Notify_FIFO_Interrupt(const uint64_t &__Timestamp_Us);

    bool Pop_Sample(Struct_BMI088_Gyro_Sample &__Sample);

    inline uint16_t Get_Queue_Depth() const;

    inline uint16_t Get_Queue_High_Watermark() const;

    inline uint32_t Get_FIFO_Interrupt_Count() const;

    inline uint32_t Get_FIFO_Status_Read_Count() const;

    inline uint32_t Get_FIFO_Frame_Read_Count() const;

    inline uint32_t Get_FIFO_Overrun_Count() const;

    inline uint32_t Get_FIFO_Spurious_Interrupt_Count() const;

    inline uint32_t Get_FIFO_Followup_Request_Count() const;

    inline uint32_t Get_Queue_Enqueue_Count() const;

    inline uint32_t Get_Queue_Consume_Count() const;

    inline uint32_t Get_Queue_Drop_Count() const;

    inline float Get_FIFO_Sample_Period_Us() const;

    inline uint64_t Get_FIFO_Last_Interrupt_Timestamp_Us() const;

    inline Class_Matrix_f32<3, 1> Get_Callibrated_Gyro() const;

    /** SPI 收完一笔时由上层分发进来：解析 FIFO_STATUS / FIFO_DATA / 寄存器 */
    uint8_t SPI_RxCallback(const uint64_t &__Ready_Timestamp_Us);

    /** 发起下一笔读取（读 FIFO_STATUS 或 FIFO_DATA） */
    uint8_t SPI_Request_Gyro();

protected:
    /* ── 绑定 ── */
    Struct_SPI_Manage_Object *SPI_Manage_Object;
    GPIO_TypeDef *CS_GPIO_Port;
    uint16_t CS_Pin;
    GPIO_PinState Activate_Pin_State;

    /* ── 常量 ── */
    const uint8_t BMI088_GYRO_READ_MASK = 0x80;
    const uint8_t BMI088_GYRO_INIT_INSTRUCTION_NUM = 8;
    const Enum_BSP_BMI088_Gyro_Range BMI088_GYRO_RANGE = BMI088_GYRO_RANGE_2000DPS;

    /** 初始化要写的 8 个寄存器：{偏移, 值} */
    const uint8_t BMI088_GYRO_REGISTER_CONFIG[8][2] = {
        // 量程
        {offsetof(Struct_BMI088_Gyro_Register, GYRO_RANGE_RW), BMI088_GYRO_RANGE},
        // 反馈频率 2000Hz、带宽 230Hz（bit7 只读且恒为 1）
        {offsetof(Struct_BMI088_Gyro_Register, GYRO_BANDWODTH_RW), 0x01 | 0x80},
        // INT3 推挽 + 高有效 —— 必须和 MCU 的上升沿 EXTI 一致
        {offsetof(Struct_BMI088_Gyro_Register, INT3_INT4_IO_CONF_RW), 0x0d},
        // 数据就绪中断映射到 INT3：每帧就绪就驱动一次 FIFO 读取
        {offsetof(Struct_BMI088_Gyro_Register, INT3_INT4_IO_MAP_RW), 0x01},
        // FIFO 水位 = 1 帧
        {offsetof(Struct_BMI088_Gyro_Register, FIFO_CONFIG_0_RW), 0x00},
        // FIFO stream 模式（满时保留最新 99 帧，不会被填满卡死）
        {offsetof(Struct_BMI088_Gyro_Register, FIFO_CONFIG_1_RW), 0x80},
        // 不使用 FIFO 水位中断（用数据就绪中断）
        {offsetof(Struct_BMI088_Gyro_Register, FIFO_WM_EN_RW), 0x08},
        // 使能数据就绪中断
        {offsetof(Struct_BMI088_Gyro_Register, GYRO_INT_CTRL_RW), 0x80},
    };

    /** 下一笔该读什么：先问状态，再按帧数读数据 */
    enum Enum_BMI088_Gyro_FIFO_Request : uint8_t
    {
        BMI088_GYRO_FIFO_REQUEST_STATUS = 0U,
        BMI088_GYRO_FIFO_REQUEST_DATA,
    };

    static constexpr uint8_t BMI088_GYRO_FIFO_WATERMARK_FRAME_COUNT = 1U;
    static constexpr uint8_t BMI088_GYRO_FIFO_FRAME_SIZE = 6U;    // 3 轴 × int16
    static constexpr uint8_t BMI088_GYRO_FIFO_MAX_READ_FRAME_COUNT =
        (SPI_BUFFER_SIZE - 1U) / BMI088_GYRO_FIFO_FRAME_SIZE;
    static constexpr uint16_t BMI088_GYRO_SAMPLE_QUEUE_CAPACITY = 128U;
    static constexpr uint16_t BMI088_GYRO_SAMPLE_QUEUE_MASK =
        BMI088_GYRO_SAMPLE_QUEUE_CAPACITY - 1U;
    /** 2kHz 采样 → 标称 500µs/样本；实测值超出 ±10% 就认为不可信（不用于校正周期） */
    static constexpr float BMI088_GYRO_NOMINAL_SAMPLE_PERIOD_US = 500.0f;
    static constexpr float BMI088_GYRO_MIN_SAMPLE_PERIOD_US = 450.0f;
    static constexpr float BMI088_GYRO_MAX_SAMPLE_PERIOD_US = 550.0f;

    /* ── 状态 ── */

    Struct_BMI088_Gyro_Register Register;

    bool Valid_Flag = true;
    Class_Matrix_f32<3, 1> Vector_Raw_Gyro;

    /* 样本队列：中断里入队，任务里出队 */
    Struct_BMI088_Gyro_Sample Sample_Queue[BMI088_GYRO_SAMPLE_QUEUE_CAPACITY] = {};
    volatile uint16_t Sample_Queue_Head = 0U;
    volatile uint16_t Sample_Queue_Tail = 0U;
    uint16_t Sample_Queue_High_Watermark = 0U;

    /* FIFO 读取状态机 */
    Enum_BMI088_Gyro_FIFO_Request FIFO_Request = BMI088_GYRO_FIFO_REQUEST_STATUS;
    uint8_t FIFO_Pending_Frame_Count = 0U;
    uint8_t FIFO_Requested_Frame_Count = 0U;
    uint8_t FIFO_Batch_Total_Frame_Count = 0U;
    uint8_t FIFO_Batch_Processed_Frame_Count = 0U;
    uint8_t FIFO_Previous_Batch_Frame_Count = 0U;
    uint64_t FIFO_Batch_Anchor_Timestamp_Us = 0U;
    uint64_t FIFO_Previous_Anchor_Timestamp_Us = 0U;
    uint64_t FIFO_Last_Interrupt_Timestamp_Us = 0U;
    uint64_t FIFO_Last_Handled_Interrupt_Timestamp_Us = 0U;
    uint64_t FIFO_Last_Enqueued_Timestamp_Us = 0U;
    float FIFO_Sample_Period_Us = BMI088_GYRO_NOMINAL_SAMPLE_PERIOD_US;
    float FIFO_Batch_Sample_Period_Us = BMI088_GYRO_NOMINAL_SAMPLE_PERIOD_US;
    bool FIFO_Overrun_Latched = false;
    uint32_t FIFO_Sample_Sequence = 0U;

    /* ── 诊断计数（波形/调试器里看，出问题不用猜） ── */
    uint32_t FIFO_Interrupt_Count = 0U;
    uint32_t FIFO_Status_Read_Count = 0U;
    uint32_t FIFO_Frame_Read_Count = 0U;
    uint32_t FIFO_Overrun_Count = 0U;
    uint32_t FIFO_Spurious_Interrupt_Count = 0U;
    uint32_t FIFO_Followup_Request_Count = 0U;
    uint32_t Sample_Queue_Enqueue_Count = 0U;
    uint32_t Sample_Queue_Consume_Count = 0U;
    uint32_t Sample_Queue_Drop_Count = 0U;

    /* ── 内部函数 ── */

    void Read_Single_Register(const uint8_t &Register_Address) const;

    void Write_Single_Register(const uint8_t &Register_Address, const uint8_t *Tx_Data_Buffer) const;

    Class_Matrix_f32<3, 1> Decode_Gyro_Frame(const uint8_t *__Frame) const;

    bool Enqueue_Sample(const Class_Matrix_f32<3, 1> &__Gyro,
                        const uint64_t &__Timestamp_Us,
                        const bool &__Valid);
};

/* Exported function declarations --------------------------------------------*/

inline bool Class_BMI088_Gyro::Get_Valid_Flag() const
{
    return (Valid_Flag);
}

/** 读原始角速度：关中断取一份快照，避免读到"改了一半"的值 */
inline Class_Matrix_f32<3, 1> Class_BMI088_Gyro::Get_Raw_Gyro() const
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const Class_Matrix_f32<3, 1> gyro = Vector_Raw_Gyro;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return gyro;
}

inline uint16_t Class_BMI088_Gyro::Get_Queue_Depth() const
{
    return static_cast<uint16_t>((Sample_Queue_Head - Sample_Queue_Tail) &
                                 BMI088_GYRO_SAMPLE_QUEUE_MASK);
}

inline uint16_t Class_BMI088_Gyro::Get_Queue_High_Watermark() const
{
    return Sample_Queue_High_Watermark;
}

inline uint32_t Class_BMI088_Gyro::Get_FIFO_Interrupt_Count() const
{
    return FIFO_Interrupt_Count;
}

inline uint32_t Class_BMI088_Gyro::Get_FIFO_Status_Read_Count() const
{
    return FIFO_Status_Read_Count;
}

inline uint32_t Class_BMI088_Gyro::Get_FIFO_Frame_Read_Count() const
{
    return FIFO_Frame_Read_Count;
}

inline uint32_t Class_BMI088_Gyro::Get_FIFO_Overrun_Count() const
{
    return FIFO_Overrun_Count;
}

inline uint32_t Class_BMI088_Gyro::Get_FIFO_Spurious_Interrupt_Count() const
{
    return FIFO_Spurious_Interrupt_Count;
}

inline uint32_t Class_BMI088_Gyro::Get_FIFO_Followup_Request_Count() const
{
    return FIFO_Followup_Request_Count;
}

inline uint32_t Class_BMI088_Gyro::Get_Queue_Enqueue_Count() const
{
    return Sample_Queue_Enqueue_Count;
}

inline uint32_t Class_BMI088_Gyro::Get_Queue_Consume_Count() const
{
    return Sample_Queue_Consume_Count;
}

inline uint32_t Class_BMI088_Gyro::Get_Queue_Drop_Count() const
{
    return Sample_Queue_Drop_Count;
}

inline float Class_BMI088_Gyro::Get_FIFO_Sample_Period_Us() const
{
    return FIFO_Sample_Period_Us;
}

inline uint64_t Class_BMI088_Gyro::Get_FIFO_Last_Interrupt_Timestamp_Us() const
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint64_t timestamp_us = FIFO_Last_Interrupt_Timestamp_Us;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return timestamp_us;
}

#endif /* BSP_BMI088_GYRO_H */

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
