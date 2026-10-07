/**
 * @file    bsp_bmi088.cpp
 * @brief   BMI088 顶层实现（状态机 + SPI 恢复 + VQF 解算）
 *
 * @note    来源：H7_BSP（zzm / USTC-RoboWalker）。
 *          移植改动清单见 bsp_bmi088.h 顶部注释。
 *
 * ── 数据流（看懂这个就看懂整个 IMU 了）──────────────────────────────
 *   陀螺：INT3(PC5) 每 500µs 就绪 → EXTI → Service_Transfer 发起读 FIFO
 *         → DMA 完成中断 → Gyro.SPI_RxCallback 解析 + 入队
 *         → 唤醒 BMI088_Task → Calculate() 逐样本跑 VQF
 *   加速度：INT1(PC4) 就绪 → EXTI → 发起读 → 回调里更新 Vector_Raw_Accel
 *         → Calculate() 用"新样本"做一次重力修正
 *   温度：128ms 周期读一次（慢变量，没必要更快）
 *   兜底：陀螺 INT 万一丢了，1ms 服务在 2ms 没收到中断时自己发起一次轮询
 * ────────────────────────────────────────────────────────────────────
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088.h"

#include "alg_basic.h"
#include <math.h>
#include <stddef.h>

/* 本工程 CubeMX 生成的任务句柄（名字跟着 CubeMX 里的 Task Name = BMI088 走）。
 * 由它来唤醒 BMI088_Task：0x0001 = 有样本入队，0x0002 = 还需要继续读 FIFO */
extern "C" { extern osThreadId_t BMI088Handle; }

/* Private variables ---------------------------------------------------------*/

/** 全局唯一实例 */
Class_BMI088 BSP_BMI088;

/* Private function declarations ---------------------------------------------*/

/**
 * @brief 判断某路通道的更新标志是否还对应同一帧（防止把新帧误清掉）
 */
static bool BMI088_Status_Update_Matches(const Struct_BMI088_Status &Status, const Struct_BMI088_Status &Shadow_Status)
{
    return Status.Update_Flag &&
           Shadow_Status.Update_Flag &&
           (Status.Update_Timestamp == Shadow_Status.Update_Timestamp) &&
           (Status.Update_Ready_Timestamp == Shadow_Status.Update_Ready_Timestamp);
}

static uint64_t BMI088_Status_Get_Update_Ready_Timestamp(const Struct_BMI088_Status &Status)
{
    if (Status.Update_Ready_Timestamp != 0)
    {
        return Status.Update_Ready_Timestamp;
    }
    return Status.Ready_Timestamp;
}

static void BMI088_Status_Clear_Update_If_Matches(Struct_BMI088_Status &Status, const Struct_BMI088_Status &Shadow_Status)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (BMI088_Status_Update_Matches(Status, Shadow_Status))
    {
        Status.Update_Flag = false;
    }
    __set_PRIMASK(primask);
}

/**
 * @brief 抢占一路通道：就绪且没有在飞的传输时，把它标成"传输中"
 *
 * @param Transfer_Ready_Timestamp 输出：这一笔对应的就绪时刻（回调里用它配对）
 */
static bool BMI088_Status_Begin_Transfer(Struct_BMI088_Status &Status,
                                         uint64_t &Transfer_Ready_Timestamp)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!Status.Ready_Flag || Status.Transfering_Flag)
    {
        __set_PRIMASK(primask);
        return false;
    }

    Transfer_Ready_Timestamp = Status.Ready_Timestamp;
    Status.Transfer_Ready_Timestamp = Transfer_Ready_Timestamp;
    Status.Transfer_Start_Timestamp_Low32 = 0U;
    Status.Transfer_Timeout_Armed = false;
    Status.Transfering_Flag = true;
    Status.Ready_Flag = false;
    __set_PRIMASK(primask);
    return true;
}

/**
 * @brief 传输真的启动成功后，才开始算超时
 *
 * @note  ★ 顺序很重要：先 Begin_Transfer（占坑）→ 调 HAL 启动 → 成功了才
 *        Arm_Transfer_Timeout。若在启动之前就起算，一次失败会被误判成超时。
 */
static void BMI088_Status_Arm_Transfer_Timeout(
    Struct_BMI088_Status &Status, const uint64_t &Transfer_Ready_Timestamp,
    const uint64_t &Transfer_Start_Timestamp)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (Status.Transfering_Flag &&
        Status.Transfer_Ready_Timestamp == Transfer_Ready_Timestamp)
    {
        Status.Transfer_Start_Timestamp_Low32 =
            static_cast<uint32_t>(Transfer_Start_Timestamp);
        __DMB();
        Status.Transfer_Timeout_Armed = true;
        __DMB();
    }
    __set_PRIMASK(primask);
}

static void BMI088_Status_Mark_Ready_If_Clear(
    Struct_BMI088_Status &Status, const uint64_t &Ready_Timestamp);

/** @brief 标记"这一路的数据到了"（EXTI / 128ms 周期里调用） */
static void BMI088_Status_Mark_Ready(Struct_BMI088_Status &Status,
                                     const uint64_t &Ready_Timestamp)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Status.Ready_Timestamp = Ready_Timestamp;
    __DMB();
    Status.Ready_Flag = true;
    __DMB();
    __set_PRIMASK(primask);
}

/**
 * @brief 启动传输失败后收尾：解除"传输中"，并把这笔的"就绪"还回去
 *
 * @note  必须还回去，否则这一路的数据就再也发不出去了（Ready 被吃掉了）。
 */
static void BMI088_Status_Restore_Ready_After_Start_Failure(
    Struct_BMI088_Status &Status, const uint64_t &Transfer_Ready_Timestamp)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Status.Transfering_Flag = false;
    Status.Transfer_Start_Timestamp_Low32 = 0U;
    Status.Transfer_Timeout_Armed = false;
    Status.Transfer_Ready_Timestamp = 0U;
    if (!Status.Ready_Flag)
    {
        Status.Ready_Timestamp = Transfer_Ready_Timestamp;
        Status.Ready_Flag = true;
    }
    __set_PRIMASK(primask);
}

/**
 * @brief 判超时：超了就解除"传输中"、并把就绪还回去
 *
 * @return true = 这一路刚判超时（调用者据此触发 SPI 恢复）
 *
 * @note  判超时顺手采一份"现场快照"（寄存器 + DMA 状态），事后可查。
 */
static bool BMI088_Status_Restore_Ready_On_Timeout(Struct_BMI088_Status &Status,
                                                   const uint32_t &Now_Timestamp_Low32,
                                                   const uint32_t &Timeout,
                                                   SPI_HandleTypeDef *SPI_Handler)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t elapsed_us = static_cast<uint32_t>(
        Now_Timestamp_Low32 - Status.Transfer_Start_Timestamp_Low32);
    const bool timed_out = Status.Transfering_Flag && Status.Transfer_Timeout_Armed &&
                           elapsed_us >= Timeout;
    if (timed_out)
    {
        SPI_Capture_Timeout_Snapshot(SPI_Handler, elapsed_us);
        const uint64_t transfer_ready_timestamp = Status.Transfer_Ready_Timestamp;
        Status.Transfering_Flag = false;
        Status.Transfer_Start_Timestamp_Low32 = 0U;
        Status.Transfer_Timeout_Armed = false;
        Status.Transfer_Ready_Timestamp = 0U;
        if (!Status.Ready_Flag)
        {
            Status.Ready_Timestamp = transfer_ready_timestamp;
            Status.Ready_Flag = true;
        }
    }
    __set_PRIMASK(primask);
    return timed_out;
}

/** @brief SPI 恢复时：把这一路从"传输中"拉回"就绪"，好让恢复后能重新发起 */
static void BMI088_Status_Restore_Ready_On_Recovery(Struct_BMI088_Status &Status)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (Status.Transfering_Flag && !Status.Ready_Flag)
    {
        Status.Ready_Timestamp = Status.Transfer_Ready_Timestamp;
        Status.Ready_Flag = true;
    }
    Status.Transfering_Flag = false;
    Status.Transfer_Start_Timestamp_Low32 = 0U;
    Status.Transfer_Timeout_Armed = false;
    Status.Transfer_Ready_Timestamp = 0U;
    __set_PRIMASK(primask);
}

/** @brief 把一条 DMA 流恢复到可用状态（HAL_SPI_Abort 之后必须做）
 *
 *  @return true = 这条流确实停了，可以再发起事务；false = 硬件还没停
 *
 *  @note  ★ 关键在【判据用硬件位，而不是 HAL 的状态书】：
 *         HAL_DMA_Abort 对"空闲流"和"真超时"都返回失败，这两者必须分开 ——
 *           · 空闲流：State != BUSY，Abort 直接返回失败（stm32f4xx_hal_dma.c:520），
 *             但这条流本来就停了，可以放行
 *           · 真超时：Abort 轮询 EN 超时后放弃（stm32f4xx_hal_dma.c:547），
 *             硬件可能还在跑；这时若强标 READY，下一笔事务就会盖掉
 *             这条流正在搬运的数据，而且错误现场被抹掉，事后无从查起
 *         所以只有 CR 的 EN 位清零了，才归还所有权。 */
static bool BMI088_Reset_DMA_Handle(DMA_HandleTypeDef *DMA_Handler)
{
    if (DMA_Handler == nullptr)
    {
        return true;        /* 没有这条流 = 不需要恢复，不是失败 */
    }

    (void)HAL_DMA_Abort(DMA_Handler);

    if ((DMA_Handler->Instance->CR & DMA_SxCR_EN) != 0U)
    {
        return false;       /* 硬件还没停：不改状态，让上层按"没恢复好"处理 */
    }

    DMA_Handler->ErrorCode = HAL_DMA_ERROR_NONE;
    DMA_Handler->State = HAL_DMA_STATE_READY;
    DMA_Handler->Lock = HAL_UNLOCKED;
    return true;
}

/* Function prototypes -------------------------------------------------------*/

void Class_BMI088::Set_VQF_Config(const Struct_BMI088_VQF_Config &__Config)
{
    if (Init_Finished_Flag)
    {
        return;
    }
    VQF_Config = __Config;
}

/**
 * @brief 把开机标定出的零偏喂给 VQF（必须在 Init() 之后）
 *
 * @note  第二参数（σ）不传，走默认的 -1 = 保持当前协方差：
 *        我们只提供"更好的初值"，不确定度仍由 VQF 自己的参数（Bias_Sigma_Init_Deg_S）
 *        决定 —— 标定值万一不准，在线估计照样能把它纠正回来。
 */
void Class_BMI088::Set_VQF_Bias_Estimate(const Class_Matrix_f32<3, 1> &__Bias)
{
    if (!Init_Finished_Flag)
    {
        return;
    }
    Filter_VQF.Set_Bias_Estimate(__Bias);
}

void Class_BMI088::Request_VQF_Bias_Estimate(const Class_Matrix_f32<3, 1> &__Bias)
{
    /* 关中断同时挡住任务切换（PendSV）：登记动作对解算任务是原子的 */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    VQF_Bias_Requested = __Bias;
    __DMB();
    VQF_Bias_Request_Pending = true;
    __set_PRIMASK(primask);
}

/**
 * @brief 初始化：加速度计 → 陀螺 → VQF
 *
 * @note  ★ 任何一步失败都返回 false，且【不打开 Init_Finished_Flag】——
 *        故障时整套 IMU 就不工作，而不是继续拿旧数据假装正常。
 */
bool Class_BMI088::Init()
{
    Init_Finished_Flag = false;
    SPI_Manage_Object = &SPI1_Manage_Object;

    const bool accel_ready = BMI088_Accel.Init();
    const bool gyro_ready = BMI088_Gyro.Init();
    if (!accel_ready || !gyro_ready)
    {
        return false;
    }
    Filter_VQF.Init(VQF_Config.Parameter,
                    VQF_Config.Gyro_D_T,
                    VQF_Config.Accel_D_T);

    // 第一次姿态输出默认 Yaw 为 0
    Vector_Euler_Angle[0][0] = 0.0f;

    Init_Finished_Flag = true;
    return true;
}

/**
 * @brief SPI 传输完成回调（从 bsp_spi 的分发函数进来）
 *
 * @note  ★ 这里【只解析、只置标志、只唤醒任务】，绝不发起下一笔传输 ——
 *        在 DMA 完成中断里再起一笔 DMA 会有竞态。下一笔由 EXTI / 1ms / 任务发起。
 */
void Class_BMI088::SPI_RxCpltCallback(GPIO_TypeDef *CS_Port, uint16_t CS_Pin)
{
    if (CS_Port == CS1_ACCEL_GPIO_Port && CS_Pin == CS1_ACCEL_Pin)
    {
        if (Init_Finished_Flag)
        {
            /* 按长度认这是哪一路：6 字节 = 加速度数据，2 字节 = 温度
             * （初始化阶段还没有这些状态机，所以上面用 Init_Finished_Flag 放行） */
            Struct_BMI088_Status *completed_status = nullptr;
            if (SPI_Manage_Object->Rx_Buffer_Length == 6U)
            {
                completed_status = &Accel_Status;
            }
            else if (SPI_Manage_Object->Rx_Buffer_Length == 2U)
            {
                completed_status = &Temperature_Status;
            }
            if (completed_status == nullptr || !completed_status->Transfering_Flag ||
                completed_status->Transfer_Ready_Timestamp == 0U)
            {
                SPI_Manage_Object->Callback_Anomaly_Count++;
                return;
            }
        }

        BMI088_Accel.SPI_RxCpltCallback();

        if (Init_Finished_Flag)
        {
            if (SPI_Manage_Object->Rx_Buffer_Length == 6)
            {
                Accel_Status.Transfering_Flag = false;
                Accel_Status.Update_Flag = true;
                Accel_Status.Update_Timestamp = Sys_Get_Micros();
                Accel_Status.Update_Ready_Timestamp = Accel_Status.Transfer_Ready_Timestamp;
                Accel_Status.Transfer_Ready_Timestamp = 0U;
                Accel_Status.Transfer_Start_Timestamp_Low32 = 0U;
                Accel_Status.Transfer_Timeout_Armed = false;
            }
            else if (SPI_Manage_Object->Rx_Buffer_Length == 2)
            {
                Temperature_Status.Transfering_Flag = false;
                Temperature_Status.Transfer_Ready_Timestamp = 0U;
                Temperature_Status.Transfer_Start_Timestamp_Low32 = 0U;
                Temperature_Status.Transfer_Timeout_Armed = false;
            }
        }
    }
    else if (CS_Port == CS1_GYRO_GPIO_Port && CS_Pin == CS1_GYRO_Pin)
    {
        if (Init_Finished_Flag &&
            (!Gyro_Status.Transfering_Flag ||
             Gyro_Status.Transfer_Ready_Timestamp == 0U))
        {
            SPI_Manage_Object->Callback_Anomaly_Count++;
            return;
        }

        const uint64_t gyro_ready_timestamp =
            Gyro_Status.Transfer_Ready_Timestamp;
        const uint8_t gyro_result =
            BMI088_Gyro.SPI_RxCallback(gyro_ready_timestamp);

        if (Init_Finished_Flag)
        {
            Gyro_Status.Transfering_Flag = false;
            Gyro_Status.Transfer_Ready_Timestamp = 0U;
            Gyro_Status.Transfer_Start_Timestamp_Low32 = 0U;
            Gyro_Status.Transfer_Timeout_Armed = false;

            /* 唤醒 BMI088 任务：
             *  0x0002 = FIFO 还没读完，任务里要接着发下一笔
             *  0x0001 = 有新样本入队，任务里要跑 Calculate() */
            if ((gyro_result &
                 BMI088_GYRO_SPI_RESULT_FOLLOWUP_REQUIRED) != 0U)
            {
                BMI088_Status_Mark_Ready_If_Clear(
                    Gyro_Status, Sys_Get_Micros());
                osThreadFlagsSet(BMI088Handle, 0x0002);
            }
            if ((gyro_result &
                 BMI088_GYRO_SPI_RESULT_SAMPLES_QUEUED) != 0U)
            {
                osThreadFlagsSet(BMI088Handle, 0x0001);
            }
        }
    }
}

/**
 * @brief EXTI 中断回调（PC4 = 加速度就绪，PC5 = 陀螺 FIFO 就绪）
 */
void Class_BMI088::EXTI_Flag_Callback(uint16_t GPIO_Pin)
{
    if (!Init_Finished_Flag) return;

    uint64_t now_timestamp = Sys_Get_Micros();

    if (GPIO_Pin == BMI088_ACCEL_INT_Pin)
    {
        BMI088_Status_Mark_Ready(Accel_Status, now_timestamp);
    }
    else if (GPIO_Pin == BMI088_GYRO_INT_Pin)
    {
        BMI088_Gyro.Notify_FIFO_Interrupt(now_timestamp);
        BMI088_Status_Mark_Ready(Gyro_Status, now_timestamp);
    }

    BMI088_Service_Transfer();
}

/**
 * @brief 128ms 周期回调：读一次温度 + 做一次带恢复的传输服务
 *
 * @note  温度是慢变量，128ms 足够；恢复动作放在这里（频率低，代价可以接受）。
 */
void Class_BMI088::TIM_128ms_Calculate_PeriodElapsedCallback()
{
    if (!Init_Finished_Flag)
    {
        return;
    }
    uint64_t now_timestamp = Sys_Get_Micros();

    BMI088_Status_Mark_Ready(Temperature_Status, now_timestamp);
    BMI088_Service_Transfer(true);
}

/**
 * @brief 1ms 周期回调：兜底轮询 + 服务传输
 *
 * @note  ★ 兜底的意义：陀螺的 INT3 万一丢了（干扰/接触不良），
 *        靠"2ms 没收到中断就自己发起一次"这条路径把数据接上，
 *        否则 FIFO 会一直攒着没人读。
 */
void Class_BMI088::TIM_1ms_Service_PeriodElapsedCallback()
{
    if (!Init_Finished_Flag)
    {
        return;
    }
    const uint64_t now_timestamp = Sys_Get_Micros();
    if ((now_timestamp - BMI088_Gyro.Get_FIFO_Last_Interrupt_Timestamp_Us()) >= 2000U &&
        (now_timestamp - Gyro_FIFO_Last_Fallback_Poll_Timestamp) >= 1000U)
    {
        Gyro_FIFO_Last_Fallback_Poll_Timestamp = now_timestamp;
        BMI088_Status_Mark_Ready_If_Clear(Gyro_Status, now_timestamp);
    }
    BMI088_Service_Transfer(true);
}

/**
 * @brief 软恢复：只清掉 HAL 的错误码。不做 abort、不碰 DMA、【不动】三路通道的状态标志
 *
 * @note  ★ 这是本工程相对上游 H7_BSP 的一处【策略改动】，依据是实测 + HAL 源码：
 *
 *          上游发现 HAL 报错时会立刻做重量级恢复（HAL_SPI_Abort + 复位两条 DMA）。
 *          但在 F4 的 HAL 里：
 *            · HAL_SPI_Abort() 内部会对每条 DMA 流调 HAL_DMA_Abort()
 *              （stm32f4xx_hal_spi.c:2368 附近）
 *            · 而 HAL_DMA_Abort() 对【空闲】的流直接返回 HAL_ERROR
 *              （stm32f4xx_hal_dma.c:520）
 *          → 恢复动作本身就会把 HAL_SPI_ERROR_DMA 记进 ErrorCode，
 *            于是"恢复"制造了"下一次恢复"的理由，形成每秒几十次的自持循环
 *            （实测：recover 每秒 +60、why 恒为 0x18 = 启动失败|HAL报错，
 *              而且每次恢复都要 1ms 任务买单 → overrun 跟着涨）。
 *
 *          软恢复先试一下通常就够了：所谓"HAL 报错"多数是上一次 abort 的残留，
 *          清掉即可；只有【连续 20 次】都好不了，才说明总线真的卡住 —— 那时才
 *          值得动 abort（由调用者按 Streak 升级）。
 */
void Class_BMI088::BMI088_Soft_Recover_SPI()
{
    SPI_Recovery_Pending_Reason = BMI088_SPI_RECOVERY_NONE;

    /* ★ 只清 HAL 的错误码，【绝不】动三路通道的状态标志。
     *
     *   最初我在这里顺手把三路都 Restore 成 Ready，结果更糟：
     *   那些标志是"有没有传输在飞"的唯一记录，硬清之后软件以为总线空了、
     *   去启动新传输，而 HAL 知道总线还忙着 → 直接拒绝 → 又记一次启动失败
     *   → 又触发软恢复…… 只是把自持循环换了个形式（实测 soft 跑到 800 次/秒）。
     *   所以：软件状态和硬件状态必须由【同一个事件】来改，不能从旁边硬掰。 */
    if (SPI_Manage_Object != nullptr && SPI_Manage_Object->SPI_Handler != nullptr)
    {
        SPI_Manage_Object->SPI_Handler->ErrorCode = HAL_SPI_ERROR_NONE;
    }
}

/**
 * @brief SPI 出问题时的整体恢复
 *
 * @note  不是"清掉某一路的状态"就完事，而是把整条链路复位：
 *        放开片选 → HAL_SPI_Abort → 复位两条 DMA 流 → 清 HAL 状态/锁 →
 *        释放 bsp_spi 层的事务占用。做完这些，SPI 就能重新发起了。
 *
 *        ⚠️ 这是【重量级】动作：HAL_SPI_Abort 里对空闲流的 HAL_DMA_Abort 会
 *        返回失败并污染 ErrorCode（见 BMI088_Soft_Recover_SPI 的注释），
 *        所以只在软恢复连续失败 20 次后才升级到这里。
 */
void Class_BMI088::BMI088_Recover_SPI(uint8_t __Reason)
{
    SPI_Recovery_Counter++;
    SPI_Recovery_Last_Reason = __Reason;
    if ((__Reason & (BMI088_SPI_RECOVERY_ACCEL_TIMEOUT |
                     BMI088_SPI_RECOVERY_GYRO_TIMEOUT |
                     BMI088_SPI_RECOVERY_TEMPERATURE_TIMEOUT)) != 0U)
    {
        SPI_Transfer_Timeout_Counter++;
    }
    if ((__Reason & BMI088_SPI_RECOVERY_ACCEL_TIMEOUT) != 0U)
    {
        SPI_Accel_Timeout_Counter++;
    }
    if ((__Reason & BMI088_SPI_RECOVERY_GYRO_TIMEOUT) != 0U)
    {
        SPI_Gyro_Timeout_Counter++;
    }
    if ((__Reason & BMI088_SPI_RECOVERY_TEMPERATURE_TIMEOUT) != 0U)
    {
        SPI_Temperature_Timeout_Counter++;
    }
    SPI_Recovery_Pending_Reason = BMI088_SPI_RECOVERY_NONE;

    BMI088_Status_Restore_Ready_On_Recovery(Accel_Status);
    BMI088_Status_Restore_Ready_On_Recovery(Gyro_Status);
    BMI088_Status_Restore_Ready_On_Recovery(Temperature_Status);

    if (SPI_Manage_Object == nullptr || SPI_Manage_Object->SPI_Handler == nullptr)
    {
        return;
    }

    SPI_HandleTypeDef *spi_handler = SPI_Manage_Object->SPI_Handler;

    if (SPI_Manage_Object->Activate_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(SPI_Manage_Object->Activate_GPIOx, SPI_Manage_Object->Activate_GPIO_Pin,
                          SPI_Manage_Object->Activate_Level == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    HAL_SPI_Abort(spi_handler);

    /* ★ 两条流都真停下来，才敢宣布"总线恢复好了"。
     *   有一条没停就返回：HAL 状态一律不改，下一次超时还会再进来重试 ——
     *   宁可让上层看见"没恢复好"，也不能给出一条半恢复的总线。 */
    if (!BMI088_Reset_DMA_Handle(spi_handler->hdmatx) ||
        !BMI088_Reset_DMA_Handle(spi_handler->hdmarx))
    {
        return;
    }

    spi_handler->ErrorCode = HAL_SPI_ERROR_NONE;
    spi_handler->State = HAL_SPI_STATE_READY;
    spi_handler->Lock = HAL_UNLOCKED;
    spi_handler->TxXferCount = 0;
    spi_handler->RxXferCount = 0;
    SPI_Manage_Object->Activate_GPIOx = nullptr;
    SPI_Manage_Object->Tx_Buffer_Length = 0;
    SPI_Manage_Object->Rx_Buffer_Length = 0;
    __DMB();
    SPI_Manage_Object->Transaction_Active = false;
    __DMB();
}

/**
 * @brief 传输入口（可重入保护）
 *
 * @note  ★ Transfer_Service_Active 是必须的：EXTI 回调里会调它，
 *        1ms/128ms 里也会调它。没有这道锁，两个上下文可能同时发起传输
 *        （DMA-in-DMA 竞态），症状是数据偶发错乱，极难查。
 */
void Class_BMI088::BMI088_Service_Transfer(const bool &Allow_Recovery)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (Transfer_Service_Active)
    {
        __set_PRIMASK(primask);
        return;
    }
    Transfer_Service_Active = true;
    __DMB();
    __set_PRIMASK(primask);

    BMI088_Service_Transfer_Locked(Allow_Recovery);

    const uint32_t unlock_primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    Transfer_Service_Active = false;
    __DMB();
    __set_PRIMASK(unlock_primask);
}

/**
 * @brief 实际的传输调度（同一时刻只允许一笔在飞）
 *
 * @note  三个要点：
 *        ① 先做恢复检查：任何一路超时 / HAL 报错 → 整体恢复后直接返回
 *        ② 已经有传输在飞 → 直接返回（等回调）
 *        ③ 按 Accel → Gyro → Temperature 轮转（Transfer_Priority_Index），
 *           避免某一路一直饿着
 */
void Class_BMI088::BMI088_Service_Transfer_Locked(const bool &Allow_Recovery)
{
    if (!Init_Finished_Flag)
    {
        return;
    }

    if (Allow_Recovery)
    {
        uint8_t recovery_reason = SPI_Recovery_Pending_Reason;
        const uint32_t now_timestamp_low32 =
            static_cast<uint32_t>(Sys_Get_Micros());
        SPI_HandleTypeDef *spi_handler = SPI_Manage_Object != nullptr
                                             ? SPI_Manage_Object->SPI_Handler
                                             : nullptr;
        if (BMI088_Status_Restore_Ready_On_Timeout(Accel_Status, now_timestamp_low32,
                                                   TRANSFERING_TIMEOUT,
                                                   spi_handler))
        {
            recovery_reason |= BMI088_SPI_RECOVERY_ACCEL_TIMEOUT;
        }
        if (BMI088_Status_Restore_Ready_On_Timeout(Gyro_Status, now_timestamp_low32,
                                                   TRANSFERING_TIMEOUT,
                                                   spi_handler))
        {
            recovery_reason |= BMI088_SPI_RECOVERY_GYRO_TIMEOUT;
        }
        if (BMI088_Status_Restore_Ready_On_Timeout(Temperature_Status, now_timestamp_low32,
                                                   TRANSFERING_TIMEOUT,
                                                   spi_handler))
        {
            recovery_reason |= BMI088_SPI_RECOVERY_TEMPERATURE_TIMEOUT;
        }

        const bool spi_error = (SPI_Manage_Object != nullptr) &&
                               (SPI_Manage_Object->SPI_Handler != nullptr) &&
                               (SPI_Manage_Object->SPI_Handler->ErrorCode != HAL_SPI_ERROR_NONE);
        if (spi_error)
        {
            recovery_reason |= BMI088_SPI_RECOVERY_HAL_ERROR;
        }
        if (recovery_reason != BMI088_SPI_RECOVERY_NONE)
        {
            /* 软恢复优先（理由见 BMI088_Soft_Recover_SPI 注释）；
             * 连续 20 拍都软恢复不掉，才升级成 abort + DMA 复位 */
            BMI088_Soft_Recover_SPI();
            if (++SPI_Soft_Recovery_Streak >= BMI088_SOFT_RECOVERY_ESCALATE_STREAK)
            {
                SPI_Soft_Recovery_Streak = 0U;
                BMI088_Recover_SPI(recovery_reason);
            }
            return;
        }
        /* 这一拍没有任何异常 → 连续计数归零 */
        SPI_Soft_Recovery_Streak = 0U;
    }
    else if (SPI_Recovery_Pending_Reason != BMI088_SPI_RECOVERY_NONE)
    {
        return;
    }

    if (Accel_Status.Transfering_Flag || Gyro_Status.Transfering_Flag || Temperature_Status.Transfering_Flag)
    {
        return;
    }

    for (uint8_t i = 0; i < 3; i++)
    {
        uint8_t index = (Transfer_Priority_Index + i) % 3;

        if (index == 0 && Accel_Status.Ready_Flag)
        {
            uint64_t transfer_ready_timestamp = 0U;
            if (!BMI088_Status_Begin_Transfer(Accel_Status, transfer_ready_timestamp))
            {
                continue;
            }
            uint8_t status = BMI088_Accel.SPI_Request_Accel();
            if (status == HAL_OK)
            {
                BMI088_Status_Arm_Transfer_Timeout(
                    Accel_Status, transfer_ready_timestamp,
                    Sys_Get_Micros());
                Transfer_Priority_Index = 1;
                SPI_Start_Failure_Streak = 0U;      /* 启动成功 → 连续失败计数归零 */
            }
            else
            {
                BMI088_Status_Restore_Ready_After_Start_Failure(
                    Accel_Status, transfer_ready_timestamp);
                /* ★ 启动失败 = "总线这一刻还被占着"：不记为待处理故障、不触发恢复，
                 *   数据仍标记 Ready，下一个节拍自然重试。连续 20 次才升级为
                 *   重量级恢复 —— 那才说明总线是真的卡死，而不是一次正常竞争。 */
                if (++SPI_Start_Failure_Streak >= BMI088_START_FAILURE_ESCALATE_STREAK &&
                    Allow_Recovery)
                {
                    SPI_Start_Failure_Streak = 0U;
                    BMI088_Recover_SPI(BMI088_SPI_RECOVERY_ACCEL_START_FAILURE);
                }
            }
            return;
        }
        else if (index == 1 && Gyro_Status.Ready_Flag)
        {
            uint64_t transfer_ready_timestamp = 0U;
            if (!BMI088_Status_Begin_Transfer(Gyro_Status, transfer_ready_timestamp))
            {
                continue;
            }
            uint8_t status = BMI088_Gyro.SPI_Request_Gyro();
            if (status == HAL_OK)
            {
                BMI088_Status_Arm_Transfer_Timeout(
                    Gyro_Status, transfer_ready_timestamp,
                    Sys_Get_Micros());
                Transfer_Priority_Index = 2;
                SPI_Start_Failure_Streak = 0U;
            }
            else
            {
                BMI088_Status_Restore_Ready_After_Start_Failure(
                    Gyro_Status, transfer_ready_timestamp);
                if (++SPI_Start_Failure_Streak >= BMI088_START_FAILURE_ESCALATE_STREAK &&
                    Allow_Recovery)
                {
                    SPI_Start_Failure_Streak = 0U;
                    BMI088_Recover_SPI(BMI088_SPI_RECOVERY_GYRO_START_FAILURE);
                }
            }
            return;
        }
        else if (index == 2 && Temperature_Status.Ready_Flag)
        {
            uint64_t transfer_ready_timestamp = 0U;
            if (!BMI088_Status_Begin_Transfer(Temperature_Status, transfer_ready_timestamp))
            {
                continue;
            }
            uint8_t status = BMI088_Accel.SPI_Request_Temperature();
            if (status == HAL_OK)
            {
                BMI088_Status_Arm_Transfer_Timeout(
                    Temperature_Status, transfer_ready_timestamp,
                    Sys_Get_Micros());
                Transfer_Priority_Index = 0;
                SPI_Start_Failure_Streak = 0U;
            }
            else
            {
                BMI088_Status_Restore_Ready_After_Start_Failure(
                    Temperature_Status, transfer_ready_timestamp);
                if (++SPI_Start_Failure_Streak >= BMI088_START_FAILURE_ESCALATE_STREAK &&
                    Allow_Recovery)
                {
                    SPI_Start_Failure_Streak = 0U;
                    BMI088_Recover_SPI(BMI088_SPI_RECOVERY_TEMPERATURE_START_FAILURE);
                }
            }
            return;
        }
    }
}

/**
 * @brief 用 VQF 处理一帧陀螺样本（BMI088_Task 里逐样本调用）
 *
 * @note  ★ 时间戳和积分步长的分工（两层，别混淆）：
 *          · 【积分步长】= VQF 初始化的名义周期（Gyro_D_T = 0.0005s），
 *            不随时间戳变 —— 硬件按 2kHz 匀速出样，逐帧等步长积分。
 *            队列丢样的代价是那一格的转角没被积分、时间上不会自动补齐，
 *            断流超过 0.1s 则整体 Reset（见下）。
 *          · 【逐帧算出的 D_T】（相邻样本时间戳之差）只用于配对与断流检测：
 *            加速度观测等到时间戳对上才用（Accel_Observation_Pending），
 *            D_T > 0.1s 说明中间断过（比如 SPI 恢复期间），必须 Reset
 *            而不是把大空白硬积分进去。
 *        时间戳是"陀螺样本自己的时刻"（FIFO 重建出来的），不是"现在"。
 */
void Class_BMI088::Calculate()
{
    if (!Init_Finished_Flag)
    {
        return;
    }
    const uint64_t calculate_start_timestamp = Sys_Get_Micros();
    Struct_BMI088_Gyro_Sample gyro_sample = {};
    if (!BMI088_Gyro.Pop_Sample(gyro_sample))
    {
        return;
    }

    /* 在短临界区取走完整请求；滤波器只读局部副本，后续请求留给下一帧。 */
    Class_Matrix_f32<3, 1> requested_bias;
    const uint32_t bias_primask = __get_PRIMASK();
    __disable_irq();
    const bool apply_bias = VQF_Bias_Request_Pending;
    if (apply_bias)
    {
        requested_bias = VQF_Bias_Requested;
        VQF_Bias_Request_Pending = false;
    }
    __set_PRIMASK(bias_primask);
    if (apply_bias)
    {
        Filter_VQF.Set_Bias_Estimate(requested_bias);
    }

    /* 加速度每 4ms 才更新一次，所以这里有"待用观测"的暂存：
     * 新的一帧加速度到了就记下来，等到时间戳对得上时再用。 */
    if (!Accel_Observation_Pending)
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        const Struct_BMI088_Status accel_status_snapshot = Accel_Status;
        const Class_Matrix_f32<3, 1> accel_snapshot = BMI088_Accel.Get_Raw_Accel();
        const bool accel_valid_snapshot = BMI088_Accel.Get_Valid_Flag();
        __set_PRIMASK(primask);

        if (accel_status_snapshot.Update_Flag)
        {
            const uint64_t accel_timestamp =
                BMI088_Status_Get_Update_Ready_Timestamp(accel_status_snapshot);
            const uint64_t accel_interval =
                static_cast<uint64_t>(VQF_Config.Accel_D_T * 1000000.0f);
            /* 同一个加速度周期内不重复取（accel_interval 取整后比较） */
            if (Pending_Accel_Timestamp == 0U ||
                accel_timestamp / accel_interval !=
                    Pending_Accel_Timestamp / accel_interval)
            {
                Vector_Pending_Accel = accel_snapshot;
                Pending_Accel_Timestamp = accel_timestamp;
                Pending_Accel_Valid = accel_valid_snapshot;
                Accel_Observation_Pending = true;
            }
            BMI088_Status_Clear_Update_If_Matches(Accel_Status, accel_status_snapshot);
        }
    }

    Vector_Original_Accel = Accel_Observation_Pending
                                ? Vector_Pending_Accel
                                : BMI088_Accel.Get_Raw_Accel();
    Vector_Original_Gyro[0][0] = gyro_sample.Gyro_Rad_S[0];
    Vector_Original_Gyro[1][0] = gyro_sample.Gyro_Rad_S[1];
    Vector_Original_Gyro[2][0] = gyro_sample.Gyro_Rad_S[2];
    Vector_Fixed_Corrected_Gyro = Vector_Original_Gyro;

    const bool gyro_valid = gyro_sample.Valid != 0U;

    /* ── 板级标定 ──
     * 原版把这段放在 `if (Heater_Enable)` 里（他们板子上加热常开）。
     * 本工程标定常量按本板留空（单位矩阵 / 零），无条件执行即可 ——
     * 留空时下面两个式子是恒等变换，不改变数据（恒温开不开与此无关）；
     * 将来标定出本板的数据，直接填进头文件里那三个数组即可。 */
    Vector_Original_Accel =
        (Class_Matrix_f32<3, 3>(ACCEL_AFFINE_DATA) *
             Vector_Original_Accel / GRAVITY_ACCELERATION +
         Class_Matrix_f32<3, 1>(ACCEL_BIAS_DATA)) *
        GRAVITY_ACCELERATION;
    if (gyro_valid)
    {
        Vector_Fixed_Corrected_Gyro +=
            Class_Matrix_f32<3, 1>(GYRO_ZERO_OFFSET);
    }
    Accel_Norm = Vector_Original_Accel.Get_Modulus();

    if (VQF_Pre_Timestamp == 0U)
    {
        D_T = VQF_Config.Gyro_D_T;
        VQF_Pre_Timestamp = gyro_sample.Timestamp_Us;
    }
    else if (gyro_sample.Timestamp_Us <= VQF_Pre_Timestamp)
    {
        // 时间戳不前进：收不到新样本或重建出错，记一笔并跳过这一帧
        Timestamp_Anomaly_Counter++;
        return;
    }
    else
    {
        D_T = static_cast<float>(gyro_sample.Timestamp_Us - VQF_Pre_Timestamp) /
              1000000.0f;
        VQF_Pre_Timestamp = gyro_sample.Timestamp_Us;
        if (D_T > D_T_TIMEOUT_THRESHOLD)
        {
            // 中间断了（>0.1s）：重置滤波器，别拿一段大空白去硬积分
            Sensor_Ready_Gap_Counter++;
            Filter_VQF.Reset();
            VQF_Reset_Counter++;
            D_T = VQF_Config.Gyro_D_T;
        }
    }

    if (gyro_valid)
    {
        Filter_VQF.Update_Gyro(Vector_Fixed_Corrected_Gyro);
    }

    if (Accel_Observation_Pending &&
        Pending_Accel_Timestamp <= gyro_sample.Timestamp_Us)
    {
        if (Pending_Accel_Valid &&
            !Basic_Math_Is_Invalid_Float(Accel_Norm) &&
            Accel_Norm > 1.0e-6f)
        {
            Filter_VQF.Update_Accel(Vector_Original_Accel);
            Set_Accel_Update_Result(true, BMI088_ACCEL_REJECT_NONE);
        }
        else
        {
            // 无效加速度宁可不修正，也不要把坏方向喂进去
            Set_Accel_Update_Result(false, BMI088_ACCEL_REJECT_INVALID);
        }
        Accel_Observation_Pending = false;
    }

    Quarternion = Filter_VQF.Get_Quaternion_6D();
    Vector_Euler_Angle = Quarternion.Get_Euler_Angle();
    Matrix_Rotation = Quarternion.Get_Rotation_Matrix();
    Vector_Axis_Angle = Quarternion.Get_Axis_Angle();

    /* 扣除重力得到"纯运动加速度"：
     * 机体系下重力方向 = R^T · (0,0,-g)，从原始加速度里减掉 */
    const Class_Matrix_f32<3, 1> vector_gravity_body =
        Matrix_Rotation.Get_Transpose() *
        (-Namespace_ALG_Matrix::Axis_Z_3d() * GRAVITY_ACCELERATION);
    Vector_Accel_Body = Vector_Original_Accel + vector_gravity_body;
    Vector_Accel = Matrix_Rotation * Vector_Accel_Body;
    Vector_Gyro_Body = Filter_VQF.Get_Last_Corrected_Gyro();
    Vector_Gyro = Matrix_Rotation * Vector_Gyro_Body;

    /* ── 整帧发布 ──
     * 用本轮算出的局部结果一次组帧、一次提交（详见结构体注释）：
     * 消费者拿到的一定是同一帧的四元数 + 欧拉角 + 原始量，
     * 不会出现"第 N 帧四元数配第 N-1 帧欧拉角"的半帧组合。 */
    Struct_BMI088_Attitude_Frame frame = {};
    frame.Quaternion     = Quarternion;
    frame.Euler_Angle    = Vector_Euler_Angle;
    frame.Gyro_Bias      = Filter_VQF.Get_Bias_Estimate();
    frame.Gyro           = Vector_Original_Gyro;
    frame.Accel          = Vector_Original_Accel;
    frame.Sample_Time_Us = gyro_sample.Timestamp_Us;
    frame.Rest_Detected  = Filter_VQF.Get_Rest_Detected() ? 1U : 0U;
    frame.Valid          = true;
    Publish_Attitude_Frame(frame);

    Calculating_Time = Sys_Get_Micros() - calculate_start_timestamp;
}

/**
 * @brief 整帧提交（生产者侧，只在 Calculate() 收尾调用）
 *
 * @note  关中断同时也挡住任务切换（PendSV），所以提交过程对任务上下文
 *        是原子的；读者（Get_Attitude_Frame）同样在关中断窗口里取，
 *        两边合起来保证"整帧"语义。
 */
void Class_BMI088::Publish_Attitude_Frame(const Struct_BMI088_Attitude_Frame &__Next)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Attitude_Frame = __Next;
    Attitude_Frame.Sequence = ++Attitude_Sequence;   /* 序号在提交时分配，保证单调 */
    __DMB();
    __set_PRIMASK(primask);
}

bool Class_BMI088::Get_Attitude_Frame(Struct_BMI088_Attitude_Frame &__Out) const
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __Out = Attitude_Frame;
    const bool valid = __Out.Valid;
    __DMB();
    __set_PRIMASK(primask);
    return valid;
}

/**
 * @brief 兜底轮询用：只有在"还没有就绪"时才标记，避免覆盖掉更新鲜的中断时刻
 */
static void BMI088_Status_Mark_Ready_If_Clear(
    Struct_BMI088_Status &Status, const uint64_t &Ready_Timestamp)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!Status.Ready_Flag)
    {
        Status.Ready_Timestamp = Ready_Timestamp;
        __DMB();
        Status.Ready_Flag = true;
        __DMB();
    }
    __set_PRIMASK(primask);
}

/** @brief 记录一次加速度更新结果（接受/拒绝 + 计数），供诊断 */
void Class_BMI088::Set_Accel_Update_Result(const bool &__Accepted, const uint8_t &__Reject_Reason)
{
    Accel_Update_Result = (__Accepted ? 1U : 0U) | (static_cast<uint32_t>(__Reject_Reason) << 8U);
    Accel_Update_Attempt_Counter++;
    if (!__Accepted)
    {
        Accel_Update_Rejected_Counter++;
    }
}

#ifdef __cplusplus
extern "C" {
#endif

/* ── 供 1ms 任务的周期回调表调用 ── */
void BMI088_TIM_128ms_Calculate_PeriodElapsedCallback()
{
    BSP_BMI088.TIM_128ms_Calculate_PeriodElapsedCallback();
}

void BMI088_TIM_1ms_Service_PeriodElapsedCallback()
{
    BSP_BMI088.TIM_1ms_Service_PeriodElapsedCallback();
}

#ifdef __cplusplus
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
