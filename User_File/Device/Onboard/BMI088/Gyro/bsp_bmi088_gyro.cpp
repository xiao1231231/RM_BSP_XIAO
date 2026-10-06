/**
 * @file    bsp_bmi088_gyro.cpp
 * @brief   BMI088 陀螺仪实现
 *
 * @note    来源：H7_BSP（USTC-RoboWalker / yssickjgd）。
 *
 * ── 相对 H7 原版的移植改动（只有这些）────────────────────────────────
 *   ① SPI2 → SPI1，片选换成 C 板的 CS1_GYRO（PB0），
 *      中断脚换成 C 板的 INT1_Gyro（PC5）
 *   ② 时间戳延时接口对齐本工程：
 *      Namespace_SYS_Timestamp::Delay_Millisecond(n) → Sys_Delay_S(n/1000)
 *   ③ 显式包含 <math.h>（fabsf）、<stddef.h>（offsetof）、alg_basic.h
 *      （Basic_Math_Is_Invalid_Float / BASIC_MATH_DEG_TO_RAD）
 *   ④ 去掉 stm32h7xx_hal.h
 * ────────────────────────────────────────────────────────────────────
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088_gyro.h"

#include "alg_basic.h"
#include <math.h>
#include <stddef.h>

/* Private variables ---------------------------------------------------------*/

/** 单轴绝对值超过它就判为野值：32 rad/s ≈ 1833°/s，接近 2000dps 量程上限 */
static constexpr float BMI088_GYRO_OUTLIER_ABSOLUTE_THRESHOLD = 32.0f;
static uint32_t BMI088_Gyro_Outlier_Counter = 0U;

/* Function prototypes -------------------------------------------------------*/

uint32_t Class_BMI088_Gyro::Get_Outlier_Counter() const
{
    return (BMI088_Gyro_Outlier_Counter);
}

/**
 * @brief EXTI 中断里调用
 *
 * @note  ★ 这里【只记录时刻、只加计数】，绝不发起 SPI 传输 ——
 *        中断里发 DMA 会和正在跑的传输抢总线。真正发起读取的是
 *        上层（EXTI 回调里调用 Service_Transfer → SPI_Request_Gyro）。
 */
void Class_BMI088_Gyro::Notify_FIFO_Interrupt(const uint64_t &__Timestamp_Us)
{
    FIFO_Last_Interrupt_Timestamp_Us = __Timestamp_Us;
    FIFO_Interrupt_Count++;
}

/**
 * @brief 从样本队列取一个样本（任务上下文调用）
 *
 * @return true = 取到了；false = 队列空
 */
bool Class_BMI088_Gyro::Pop_Sample(Struct_BMI088_Gyro_Sample &__Sample)
{
    const uint16_t tail = Sample_Queue_Tail;
    __DMB();
    if (tail == Sample_Queue_Head)
    {
        return false;
    }

    __Sample = Sample_Queue[tail];
    __DMB();
    Sample_Queue_Tail = static_cast<uint16_t>((tail + 1U) &
                                               BMI088_GYRO_SAMPLE_QUEUE_MASK);
    Sample_Queue_Consume_Count++;
    return true;
}

/**
 * @brief 初始化陀螺仪
 *
 * @note  每一步最多重试 5 次，任何一步失败都返回 false ——
 *        ★ 上层必须检查返回值：初始化失败还继续跑，"姿态不动"会变成
 *          一个没有任何提示的哑故障（旧驱动就是只打日志不检查）。
 */
bool Class_BMI088_Gyro::Init()
{
    const uint8_t max_attempts = 5;

    SPI_Manage_Object = &SPI1_Manage_Object;

    CS_GPIO_Port = CS1_GYRO_GPIO_Port;
    CS_Pin = CS1_GYRO_Pin;
    Activate_Pin_State = GPIO_PIN_RESET;

    Sample_Queue_Head = 0U;
    Sample_Queue_Tail = 0U;
    Sample_Queue_High_Watermark = 0U;
    FIFO_Request = BMI088_GYRO_FIFO_REQUEST_STATUS;
    FIFO_Pending_Frame_Count = 0U;
    FIFO_Requested_Frame_Count = 0U;
    FIFO_Batch_Total_Frame_Count = 0U;
    FIFO_Batch_Processed_Frame_Count = 0U;
    FIFO_Previous_Batch_Frame_Count = 0U;
    FIFO_Batch_Anchor_Timestamp_Us = 0U;
    FIFO_Previous_Anchor_Timestamp_Us = 0U;
    FIFO_Last_Interrupt_Timestamp_Us = 0U;
    FIFO_Last_Handled_Interrupt_Timestamp_Us = 0U;
    FIFO_Last_Enqueued_Timestamp_Us = 0U;
    FIFO_Sample_Period_Us = BMI088_GYRO_NOMINAL_SAMPLE_PERIOD_US;
    FIFO_Batch_Sample_Period_Us = BMI088_GYRO_NOMINAL_SAMPLE_PERIOD_US;
    FIFO_Overrun_Latched = false;
    FIFO_Sample_Sequence = 0U;
    FIFO_Interrupt_Count = 0U;
    FIFO_Status_Read_Count = 0U;
    FIFO_Frame_Read_Count = 0U;
    FIFO_Overrun_Count = 0U;
    FIFO_Spurious_Interrupt_Count = 0U;
    FIFO_Followup_Request_Count = 0U;
    Sample_Queue_Enqueue_Count = 0U;
    Sample_Queue_Consume_Count = 0U;
    Sample_Queue_Drop_Count = 0U;

    uint8_t res;

    // 检测通信是否正常（能读回芯片 ID 0x0F 才算通）
    Register.GYRO_CHIP_ID_RO = 0x00;
    for (uint8_t attempt = 0; attempt < max_attempts && Register.GYRO_CHIP_ID_RO != 0x0f; attempt++)
    {
        Read_Single_Register(offsetof(Struct_BMI088_Gyro_Register, GYRO_CHIP_ID_RO));
        Sys_Delay_S(0.1f);
    }
    if (Register.GYRO_CHIP_ID_RO != 0x0f)
    {
        return false;
    }

    // 软重启
    res = 0xb6;
    Write_Single_Register(offsetof(Struct_BMI088_Gyro_Register, GYRO_SOFTRESET_WO), &res);
    Sys_Delay_S(0.1f);

    // 重启后再确认一次通信
    Register.GYRO_CHIP_ID_RO = 0x00;
    for (uint8_t attempt = 0; attempt < max_attempts && Register.GYRO_CHIP_ID_RO != 0x0f; attempt++)
    {
        Read_Single_Register(offsetof(Struct_BMI088_Gyro_Register, GYRO_CHIP_ID_RO));
        Sys_Delay_S(0.1f);
    }
    if (Register.GYRO_CHIP_ID_RO != 0x0f)
    {
        return false;
    }

    // 逐条写配置，并且【读回校验】：写进去的值和读回来的不一致就重试，
    // 全部重试都失败说明这条 SPI 链路不可靠，直接返回 false。
    for (uint8_t i = 0; i < BMI088_GYRO_INIT_INSTRUCTION_NUM; i++)
    {
        uint8_t *readback = ((uint8_t *) (&Register)) + BMI088_GYRO_REGISTER_CONFIG[i][0];
        *readback = ~BMI088_GYRO_REGISTER_CONFIG[i][1];
        for (uint8_t attempt = 0; attempt < max_attempts && *readback != BMI088_GYRO_REGISTER_CONFIG[i][1]; attempt++)
        {
            Write_Single_Register(BMI088_GYRO_REGISTER_CONFIG[i][0], &BMI088_GYRO_REGISTER_CONFIG[i][1]);
            Sys_Delay_S(0.1f);

            Read_Single_Register(BMI088_GYRO_REGISTER_CONFIG[i][0]);
            Sys_Delay_S(0.1f);
        }
        if (*readback != BMI088_GYRO_REGISTER_CONFIG[i][1])
        {
            return false;
        }
    }

    // 中断脚：推挽、高有效、上升沿（和上面 INT3_INT4_IO_CONF 的配置必须一致）
    GPIO_InitTypeDef gpio_init = {};
    gpio_init.Pin = BMI088_GYRO_INT_Pin;
    gpio_init.Mode = GPIO_MODE_IT_RISING;
    gpio_init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BMI088_GYRO_INT_GPIO_Port, &gpio_init);
    return true;
}

/**
 * @brief 初始化收尾：重写一次 FIFO 配置
 *
 * @note  初始化过程本身也会往 FIFO 里灌数据（配置期间传感器已经在采样），
 *        这里重写配置等于"清空并重新开始"，避免把启动阶段的旧样本喂给滤波器。
 */
void Class_BMI088_Gyro::Start_FIFO_Acquisition()
{
    const uint8_t fifo_stream_mode = 0x80U;
    Write_Single_Register(
        offsetof(Struct_BMI088_Gyro_Register, FIFO_CONFIG_1_RW),
        &fifo_stream_mode);
    Sys_Delay_S(0.001f);
}

/**
 * @brief SPI 接收回调：解析刚收回来的一笔数据
 *
 * @return 位掩码：有没有入队样本 / 还要不要继续发下一笔
 *
 * @note  三件事分得很清楚：
 *          读 FIFO_STATUS → 知道攒了几帧，决定下一笔读多少
 *          读 FIFO_DATA   → 逐帧解码、重建时间戳、入队
 *          读普通寄存器   → 直接按偏移 memcpy 进 Register
 */
uint8_t Class_BMI088_Gyro::SPI_RxCallback(const uint64_t &__Ready_Timestamp_Us)
{
    const uint8_t spi_init_address =
        SPI_Manage_Object->Tx_Buffer[0] & ~BMI088_GYRO_READ_MASK;

    if (spi_init_address == offsetof(Struct_BMI088_Gyro_Register, FIFO_STATUS_RO))
    {
        if (SPI_Manage_Object->Rx_Buffer_Length != 1U)
        {
            return BMI088_GYRO_SPI_RESULT_NONE;
        }

        Register.FIFO_STATUS_RO = SPI_Manage_Object->Rx_Buffer[1];
        FIFO_Status_Read_Count++;
        const bool overrun = (Register.FIFO_STATUS_RO & 0x80U) != 0U;
        const uint8_t frame_count = Register.FIFO_STATUS_RO & 0x7fU;
        if (overrun && !FIFO_Overrun_Latched)
        {
            FIFO_Overrun_Latched = true;
            FIFO_Overrun_Count++;
        }

        /* 这一笔是"中断驱动"还是"1ms 兜底轮询"发起的：
         * 中断驱动时，就绪时刻 == 上次中断时刻。区分开是为了统计"假中断"。 */
        const bool from_interrupt =
            __Ready_Timestamp_Us != 0U &&
            __Ready_Timestamp_Us == FIFO_Last_Interrupt_Timestamp_Us &&
            __Ready_Timestamp_Us != FIFO_Last_Handled_Interrupt_Timestamp_Us;
        if (from_interrupt)
        {
            FIFO_Last_Handled_Interrupt_Timestamp_Us = __Ready_Timestamp_Us;
        }

        if (frame_count < BMI088_GYRO_FIFO_WATERMARK_FRAME_COUNT && !overrun)
        {
            if (from_interrupt)
            {
                FIFO_Spurious_Interrupt_Count++;
            }
            FIFO_Request = BMI088_GYRO_FIFO_REQUEST_STATUS;
            return BMI088_GYRO_SPI_RESULT_NONE;
        }
        if (frame_count == 0U)
        {
            FIFO_Request = BMI088_GYRO_FIFO_REQUEST_STATUS;
            return BMI088_GYRO_SPI_RESULT_NONE;
        }

        /* 用"两次锚点之间的帧数"反推实际样本周期（低通一下，抗单次抖动）——
         * 这是后面给每帧重建时间戳的尺子。 */
        if (FIFO_Previous_Anchor_Timestamp_Us != 0U &&
            FIFO_Previous_Batch_Frame_Count != 0U &&
            __Ready_Timestamp_Us > FIFO_Previous_Anchor_Timestamp_Us)
        {
            const float estimated_period_us =
                static_cast<float>(__Ready_Timestamp_Us -
                                   FIFO_Previous_Anchor_Timestamp_Us) /
                static_cast<float>(FIFO_Previous_Batch_Frame_Count);
            if (estimated_period_us >= BMI088_GYRO_MIN_SAMPLE_PERIOD_US &&
                estimated_period_us <= BMI088_GYRO_MAX_SAMPLE_PERIOD_US)
            {
                FIFO_Sample_Period_Us =
                    0.875f * FIFO_Sample_Period_Us +
                    0.125f * estimated_period_us;
            }
        }

        FIFO_Batch_Total_Frame_Count = frame_count;
        FIFO_Batch_Processed_Frame_Count = 0U;
        FIFO_Batch_Anchor_Timestamp_Us = __Ready_Timestamp_Us;
        FIFO_Batch_Sample_Period_Us = FIFO_Sample_Period_Us;
        /* 兜底轮询这一路的周期估算：中断没来（比如 EXTI 丢了）时也能量出
         * 一个合理周期，否则整批样本的时间戳会全按标称 500µs 算。 */
        if (!from_interrupt && FIFO_Last_Enqueued_Timestamp_Us != 0U &&
            __Ready_Timestamp_Us > FIFO_Last_Enqueued_Timestamp_Us)
        {
            const float batch_period_us =
                static_cast<float>(__Ready_Timestamp_Us -
                                   FIFO_Last_Enqueued_Timestamp_Us) /
                static_cast<float>(frame_count);
            if (batch_period_us >= 250.0f && batch_period_us <= 750.0f)
            {
                FIFO_Batch_Sample_Period_Us = batch_period_us;
            }
        }
        FIFO_Pending_Frame_Count =
            frame_count > BMI088_GYRO_FIFO_MAX_READ_FRAME_COUNT
                ? BMI088_GYRO_FIFO_MAX_READ_FRAME_COUNT
                : frame_count;
        FIFO_Request = BMI088_GYRO_FIFO_REQUEST_DATA;
        FIFO_Followup_Request_Count++;
        return BMI088_GYRO_SPI_RESULT_FOLLOWUP_REQUIRED;
    }

    if (spi_init_address == offsetof(Struct_BMI088_Gyro_Register, FIFO_DATA_RO))
    {
        const uint16_t expected_length =
            static_cast<uint16_t>(FIFO_Requested_Frame_Count) *
            BMI088_GYRO_FIFO_FRAME_SIZE;
        if (FIFO_Requested_Frame_Count == 0U ||
            SPI_Manage_Object->Rx_Buffer_Length != expected_length)
        {
            FIFO_Request = BMI088_GYRO_FIFO_REQUEST_STATUS;
            return BMI088_GYRO_SPI_RESULT_NONE;
        }

        uint8_t result = BMI088_GYRO_SPI_RESULT_NONE;
        for (uint8_t i = 0U; i < FIFO_Requested_Frame_Count; i++)
        {
            const uint8_t *frame =
                &SPI_Manage_Object->Rx_Buffer[1U +
                                              static_cast<uint16_t>(i) *
                                                  BMI088_GYRO_FIFO_FRAME_SIZE];
            const Class_Matrix_f32<3, 1> gyro = Decode_Gyro_Frame(frame);
            const bool invalid_float =
                Basic_Math_Is_Invalid_Float(gyro[0][0]) ||
                Basic_Math_Is_Invalid_Float(gyro[1][0]) ||
                Basic_Math_Is_Invalid_Float(gyro[2][0]);
            const bool outlier =
                fabsf(gyro[0][0]) >= BMI088_GYRO_OUTLIER_ABSOLUTE_THRESHOLD ||
                fabsf(gyro[1][0]) >= BMI088_GYRO_OUTLIER_ABSOLUTE_THRESHOLD ||
                fabsf(gyro[2][0]) >= BMI088_GYRO_OUTLIER_ABSOLUTE_THRESHOLD;
            const bool valid = !invalid_float && !outlier;
            if (outlier)
            {
                BMI088_Gyro_Outlier_Counter++;
            }

            Vector_Raw_Gyro = gyro;
            Valid_Flag = valid;

            /* 时间戳重建：这一批里【最后一帧】就是锚点时刻，
             * 第 i 帧的时刻 = 锚点 + (i - 最后一帧下标) × 实测周期。
             * 这样每个样本带的是"传感器采到它的时刻"，不是"CPU 读到它的时刻"。 */
            const uint8_t anchor_frame_index =
                FIFO_Batch_Total_Frame_Count - 1U;
            const int32_t frame_offset =
                static_cast<int32_t>(FIFO_Batch_Processed_Frame_Count) + i -
                static_cast<int32_t>(anchor_frame_index);
            const float timestamp_offset_f =
                static_cast<float>(frame_offset) *
                FIFO_Batch_Sample_Period_Us;
            const int64_t timestamp_offset_us = static_cast<int64_t>(
                timestamp_offset_f + (timestamp_offset_f >= 0.0f ? 0.5f : -0.5f));
            uint64_t sample_timestamp_us = FIFO_Batch_Anchor_Timestamp_Us;
            if (timestamp_offset_us >= 0)
            {
                sample_timestamp_us += static_cast<uint64_t>(timestamp_offset_us);
            }
            else
            {
                const uint64_t offset = static_cast<uint64_t>(-timestamp_offset_us);
                sample_timestamp_us = sample_timestamp_us > offset
                                          ? sample_timestamp_us - offset
                                          : 1U;
            }

            /* 保证时间戳严格递增：否则后面按时间积分会出现零步长或倒序 */
            if (FIFO_Last_Enqueued_Timestamp_Us != 0U &&
                sample_timestamp_us <= FIFO_Last_Enqueued_Timestamp_Us)
            {
                sample_timestamp_us = FIFO_Last_Enqueued_Timestamp_Us + 1U;
            }

            if (Enqueue_Sample(gyro, sample_timestamp_us, valid))
            {
                FIFO_Last_Enqueued_Timestamp_Us = sample_timestamp_us;
                result |= BMI088_GYRO_SPI_RESULT_SAMPLES_QUEUED;
            }
        }

        FIFO_Frame_Read_Count += FIFO_Requested_Frame_Count;
        FIFO_Batch_Processed_Frame_Count += FIFO_Requested_Frame_Count;
        const uint8_t remaining =
            FIFO_Batch_Total_Frame_Count - FIFO_Batch_Processed_Frame_Count;
        if (remaining != 0U)
        {
            /* 一帧 6 字节，一次最多读 (512-1)/6 = 85 帧；还剩就再读一笔 */
            FIFO_Pending_Frame_Count =
                remaining > BMI088_GYRO_FIFO_MAX_READ_FRAME_COUNT
                    ? BMI088_GYRO_FIFO_MAX_READ_FRAME_COUNT
                    : remaining;
            FIFO_Request = BMI088_GYRO_FIFO_REQUEST_DATA;
            FIFO_Followup_Request_Count++;
            result |= BMI088_GYRO_SPI_RESULT_FOLLOWUP_REQUIRED;
        }
        else
        {
            FIFO_Previous_Anchor_Timestamp_Us = FIFO_Batch_Anchor_Timestamp_Us;
            FIFO_Previous_Batch_Frame_Count = FIFO_Batch_Total_Frame_Count;
            FIFO_Batch_Total_Frame_Count = 0U;
            FIFO_Batch_Processed_Frame_Count = 0U;
            FIFO_Request = BMI088_GYRO_FIFO_REQUEST_STATUS;
        }
        return result;
    }

    /* 普通寄存器：收回来的字节按偏移填进 Register（Init 里读回校验用） */
    memcpy((uint8_t *) (&Register) + spi_init_address, &SPI_Manage_Object->Rx_Buffer[1], SPI_Manage_Object->Rx_Buffer_Length);

    return BMI088_GYRO_SPI_RESULT_NONE;
}

/**
 * @brief 发起下一笔读取：按状态机决定读 FIFO_STATUS 还是 FIFO_DATA
 *
 * @return HAL 状态（HAL_OK / HAL_BUSY / HAL_ERROR）
 */
uint8_t Class_BMI088_Gyro::SPI_Request_Gyro()
{
    uint8_t register_address =
        offsetof(Struct_BMI088_Gyro_Register, FIFO_STATUS_RO);
    uint16_t rx_length = 1U;
    if (FIFO_Request == BMI088_GYRO_FIFO_REQUEST_DATA)
    {
        register_address = offsetof(Struct_BMI088_Gyro_Register, FIFO_DATA_RO);
        FIFO_Requested_Frame_Count = FIFO_Pending_Frame_Count;
        rx_length = static_cast<uint16_t>(FIFO_Requested_Frame_Count) *
                    BMI088_GYRO_FIFO_FRAME_SIZE;
    }
    const uint8_t tx_data[1] = {
        static_cast<uint8_t>(register_address | BMI088_GYRO_READ_MASK)};

    const uint8_t status = SPI_Transmit_Receive_Data(
        SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin,
        Activate_Pin_State, tx_data, 1U, rx_length);
    /* 数据笔没发出去就把"已请求帧数"清零 —— 否则回调里会拿旧长度去校验，
     * 一直卡在"长度不符"，FIFO 再也读不出来 */
    if (status != HAL_OK && FIFO_Request == BMI088_GYRO_FIFO_REQUEST_DATA)
    {
        FIFO_Requested_Frame_Count = 0U;
    }
    return status;
}

/**
 * @brief 把 3 轴 int16 原始值转成 rad/s
 *
 * @note  量程 2000dps 时：1 LSB = 1<<4 = 16，即 16/32768 × 125°/s …
 *        公式照搬原版，别动。
 */
Class_Matrix_f32<3, 1> Class_BMI088_Gyro::Decode_Gyro_Frame(
    const uint8_t *__Frame) const
{
    Class_Matrix_f32<3, 1> gyro;
    if (__Frame == nullptr)
    {
        return gyro;
    }

    const int16_t raw_x = static_cast<int16_t>(
        static_cast<uint16_t>(__Frame[0]) |
        (static_cast<uint16_t>(__Frame[1]) << 8U));
    const int16_t raw_y = static_cast<int16_t>(
        static_cast<uint16_t>(__Frame[2]) |
        (static_cast<uint16_t>(__Frame[3]) << 8U));
    const int16_t raw_z = static_cast<int16_t>(
        static_cast<uint16_t>(__Frame[4]) |
        (static_cast<uint16_t>(__Frame[5]) << 8U));
    const float scale = static_cast<float>(1U << (4U - BMI088_GYRO_RANGE)) *
                        125.0f * BASIC_MATH_DEG_TO_RAD / 32768.0f;
    gyro[0][0] = static_cast<float>(raw_x) * scale;
    gyro[1][0] = static_cast<float>(raw_y) * scale;
    gyro[2][0] = static_cast<float>(raw_z) * scale;
    return gyro;
}

/**
 * @brief 入队一个样本（中断上下文调用）
 *
 * @return true = 入队成功；false = 队列满，丢弃（Drop 计数 +1）
 */
bool Class_BMI088_Gyro::Enqueue_Sample(
    const Class_Matrix_f32<3, 1> &__Gyro,
    const uint64_t &__Timestamp_Us,
    const bool &__Valid)
{
    const uint32_t sequence = ++FIFO_Sample_Sequence;
    const uint16_t head = Sample_Queue_Head;
    const uint16_t next_head = static_cast<uint16_t>(
        (head + 1U) & BMI088_GYRO_SAMPLE_QUEUE_MASK);
    if (next_head == Sample_Queue_Tail)
    {
        Sample_Queue_Drop_Count++;
        return false;
    }

    Struct_BMI088_Gyro_Sample &sample = Sample_Queue[head];
    sample.Timestamp_Us = __Timestamp_Us;
    sample.Gyro_Rad_S[0] = __Gyro[0][0];
    sample.Gyro_Rad_S[1] = __Gyro[1][0];
    sample.Gyro_Rad_S[2] = __Gyro[2][0];
    sample.Sequence = sequence;
    sample.Valid = __Valid ? 1U : 0U;
    sample.Reserved[0] = 0U;
    sample.Reserved[1] = 0U;
    sample.Reserved[2] = 0U;
    __DMB();
    Sample_Queue_Head = next_head;
    Sample_Queue_Enqueue_Count++;

    const uint16_t depth = Get_Queue_Depth();
    if (depth > Sample_Queue_High_Watermark)
    {
        Sample_Queue_High_Watermark = depth;
    }
    return true;
}

/**
 * @brief 读单个寄存器
 *
 * @note  ★ 异步接口：数据不会立刻返回，而是在 SPI 接收回调里填进 Register。
 *        所以"读 → 延时 → 用 Register 的值"这个顺序不能变。
 */
void Class_BMI088_Gyro::Read_Single_Register(const uint8_t &Register_Address) const
{
    const uint8_t tx_data[1] = {static_cast<uint8_t>(Register_Address | BMI088_GYRO_READ_MASK)};

    SPI_Transmit_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin,
                              Activate_Pin_State, tx_data, 1, 1);
}

/** @brief 写单个寄存器（地址 + 数据，两字节，写操作不需要掩码） */
void Class_BMI088_Gyro::Write_Single_Register(const uint8_t &Register_Address, const uint8_t *Tx_Data_Buffer) const
{
    const uint8_t tx_data[2] = {Register_Address, Tx_Data_Buffer[0]};

    SPI_Transmit_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin,
                      Activate_Pin_State, tx_data, sizeof(tx_data));
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
