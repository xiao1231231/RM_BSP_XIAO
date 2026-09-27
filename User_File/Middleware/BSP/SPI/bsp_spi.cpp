/**
 * @file    bsp_spi.cpp
 * @brief   SPI 通信层实现（DMA 收发 + 事务占用 + 诊断计数）
 *
 * @note    来源：H7_BSP（USTC-RoboWalker / yssickjgd）的 Middleware/BSP/SPI。
 *
 * ── 相对 H7 原版的移植改动（只有这些，其余逐行照搬）──────────────────
 *   ① 去掉 `__attribute__((section(".dma_buffer"), aligned(32)))`：
 *      `.dma_buffer` 是 H7 链接脚本里的段（放非缓存内存，配合 D-Cache）。
 *      F407 没有 D-Cache，这个段在本工程的链接脚本里也不存在 —— 留着反而会让
 *      变量落到未定义的位置。
 *   ② 只保留 SPI1：本工程只有 BMI088 一路 SPI 器件，SPI2~SPI6 的对象、
 *      分支和 SPI6 的阻塞式特例全部删掉。
 *   ③ 时间戳接口对齐本工程：SYS_Timestamp.Get_Now_Microsecond() /
 *      Get_Current_Timestamp() → Sys_Get_Micros()（同为 64 位微秒）。
 *   ④ 超时快照换成 F4 的寄存器：删掉 H7 专有的 SPI_CFG1 / SPI_IER，
 *      DMA1->LISR → DMA2->LISR（F4 上 SPI1 用的是 DMA2 的流），
 *      NVIC 位改成 SPI1 / DMA2_Stream0 / DMA2_Stream3。
 *   ⑤ 删掉没有用到的 extern 声明 init_finished。
 * ────────────────────────────────────────────────────────────────────
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_spi.h"

/* Private variables ---------------------------------------------------------*/

/* 结构体自带 1KB 收发缓冲，静态分配即可。F407 的 DMA 能直接访问普通 SRAM */
Struct_SPI_Manage_Object SPI1_Manage_Object = {nullptr};

Struct_SPI_Timeout_Snapshot SPI1_Timeout_Snapshot = {};

/* Private function declarations ---------------------------------------------*/

/** 句柄 → 管理对象 的映射（加新 SPI 时在这里加一行） */
static Struct_SPI_Manage_Object *SPI_Get_Manage_Object(SPI_HandleTypeDef *hspi)
{
    if (hspi == nullptr)
    {
        return nullptr;
    }

    if (hspi->Instance == SPI1) return &SPI1_Manage_Object;
    return nullptr;
}

/**
 * @brief 采集一次"超时现场"
 *
 * @note  ★ 只在判定传输超时时调用：把 SPI/DMA/HAL 此刻的所有状态一次性冻结下来，
 *        之后慢慢在调试器里看。总线类故障往往转瞬即逝，没有快照就只能猜。
 */
void SPI_Capture_Timeout_Snapshot(SPI_HandleTypeDef *hspi, uint32_t Elapsed_Us)
{
    if (hspi == nullptr || hspi->Instance != SPI1)
    {
        return;
    }

    Struct_SPI_Manage_Object *manage_object = SPI_Get_Manage_Object(hspi);
    DMA_HandleTypeDef *rx_dma = hspi->hdmarx;
    DMA_HandleTypeDef *tx_dma = hspi->hdmatx;
    DMA_Stream_TypeDef *rx_stream = rx_dma != nullptr
                                         ? static_cast<DMA_Stream_TypeDef *>(rx_dma->Instance)
                                         : nullptr;
    DMA_Stream_TypeDef *tx_stream = tx_dma != nullptr
                                         ? static_cast<DMA_Stream_TypeDef *>(tx_dma->Instance)
                                         : nullptr;
    const uint32_t next_capture_count = SPI1_Timeout_Snapshot.Capture_Count + 1U;

    uint8_t pending_bits = 0U;
    pending_bits |= NVIC_GetPendingIRQ(SPI1_IRQn) != 0U ? 0x01U : 0U;
    pending_bits |= NVIC_GetPendingIRQ(DMA2_Stream0_IRQn) != 0U ? 0x02U : 0U;
    pending_bits |= NVIC_GetPendingIRQ(DMA2_Stream3_IRQn) != 0U ? 0x04U : 0U;

    uint8_t active_bits = 0U;
    active_bits |= NVIC_GetActive(SPI1_IRQn) != 0U ? 0x01U : 0U;
    active_bits |= NVIC_GetActive(DMA2_Stream0_IRQn) != 0U ? 0x02U : 0U;
    active_bits |= NVIC_GetActive(DMA2_Stream3_IRQn) != 0U ? 0x04U : 0U;

    SPI1_Timeout_Snapshot.Timestamp_Low32_Us =
        static_cast<uint32_t>(Sys_Get_Micros());
    SPI1_Timeout_Snapshot.Elapsed_Us = Elapsed_Us;
    SPI1_Timeout_Snapshot.SPI_CR1 = hspi->Instance->CR1;
    SPI1_Timeout_Snapshot.SPI_CR2 = hspi->Instance->CR2;
    SPI1_Timeout_Snapshot.SPI_SR = hspi->Instance->SR;
    SPI1_Timeout_Snapshot.DMA_LISR = DMA2->LISR;
    SPI1_Timeout_Snapshot.RX_DMA_CR =
        rx_stream != nullptr ? rx_stream->CR : 0U;
    SPI1_Timeout_Snapshot.RX_DMA_NDTR =
        rx_stream != nullptr ? rx_stream->NDTR : 0U;
    SPI1_Timeout_Snapshot.TX_DMA_CR =
        tx_stream != nullptr ? tx_stream->CR : 0U;
    SPI1_Timeout_Snapshot.TX_DMA_NDTR =
        tx_stream != nullptr ? tx_stream->NDTR : 0U;
    SPI1_Timeout_Snapshot.HAL_Error_Code = hspi->ErrorCode;
    SPI1_Timeout_Snapshot.HAL_Tx_Xfer_Count = hspi->TxXferCount;
    SPI1_Timeout_Snapshot.HAL_Rx_Xfer_Count = hspi->RxXferCount;
    SPI1_Timeout_Snapshot.Manager_Tx_Length =
        manage_object != nullptr ? manage_object->Tx_Buffer_Length : 0U;
    SPI1_Timeout_Snapshot.Manager_Rx_Length =
        manage_object != nullptr ? manage_object->Rx_Buffer_Length : 0U;
    SPI1_Timeout_Snapshot.HAL_State = static_cast<uint8_t>(hspi->State);
    SPI1_Timeout_Snapshot.HAL_Lock = static_cast<uint8_t>(hspi->Lock);
    SPI1_Timeout_Snapshot.RX_DMA_State =
        rx_dma != nullptr ? static_cast<uint8_t>(rx_dma->State) : 0U;
    SPI1_Timeout_Snapshot.TX_DMA_State =
        tx_dma != nullptr ? static_cast<uint8_t>(tx_dma->State) : 0U;
    SPI1_Timeout_Snapshot.Transaction_Active =
        manage_object != nullptr && manage_object->Transaction_Active ? 1U : 0U;
    SPI1_Timeout_Snapshot.NVIC_Pending_Bits = pending_bits;
    SPI1_Timeout_Snapshot.NVIC_Active_Bits = active_bits;
    SPI1_Timeout_Snapshot.Reserved = 0U;
    __DMB();
    SPI1_Timeout_Snapshot.Capture_Count = next_capture_count;
    __DMB();
}

/**
 * @brief 抢占事务（同一路 SPI 同时只允许一个传输）
 *
 * @note  用 PRIMASK 关中断而不是 RTOS 临界区：本层要在【中断回调里】也能用，
 *        而关中断是最短的、上下文无关的做法。原样保留 H7 的写法。
 */
static bool SPI_Try_Acquire_Transaction(Struct_SPI_Manage_Object *manage_object)
{
    if (manage_object == nullptr)
    {
        return false;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool acquired = !manage_object->Transaction_Active;
    if (acquired)
    {
        manage_object->Transaction_Active = true;
        __DMB();
    }
    else
    {
        manage_object->Transaction_Busy_Count++;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return acquired;
}

/** 把片选拉回"未选中"（电平与 Activate_Level 相反） */
static void SPI_Deactivate_Chip_Select(Struct_SPI_Manage_Object *manage_object)
{
    if (manage_object != nullptr && manage_object->Activate_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(manage_object->Activate_GPIOx, manage_object->Activate_GPIO_Pin,
                          manage_object->Activate_Level == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

/** 释放事务占用，清掉本次的记录 */
static void SPI_Release_Transaction(Struct_SPI_Manage_Object *manage_object)
{
    if (manage_object == nullptr)
    {
        return;
    }

    manage_object->Activate_GPIOx = nullptr;
    manage_object->Activate_GPIO_Pin = 0;
    manage_object->Tx_Buffer_Length = 0;
    manage_object->Rx_Buffer_Length = 0;
    __DMB();
    manage_object->Transaction_Active = false;
    __DMB();
}

/* Function prototypes -------------------------------------------------------*/

void SPI_Init(SPI_HandleTypeDef *hspi, SPI_Callback Callback_Function)
{
    Struct_SPI_Manage_Object *manage_object = SPI_Get_Manage_Object(hspi);
    if (manage_object == nullptr)
    {
        return;
    }

    memset(manage_object, 0, sizeof(Struct_SPI_Manage_Object));
    manage_object->SPI_Handler = hspi;
    manage_object->Callback_Function = Callback_Function;
}

uint8_t SPI_Transmit_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                          GPIO_PinState Activate_Level, const uint8_t *Tx_Data, uint16_t Tx_Length)
{
    Struct_SPI_Manage_Object *manage_object = SPI_Get_Manage_Object(hspi);
    if (manage_object == nullptr || Tx_Data == nullptr || Tx_Length == 0 || Tx_Length > SPI_BUFFER_SIZE)
    {
        return HAL_ERROR;
    }
    if (!SPI_Try_Acquire_Transaction(manage_object))
    {
        return HAL_BUSY;
    }

    memcpy(manage_object->Tx_Buffer, Tx_Data, Tx_Length);
    manage_object->Activate_GPIOx = GPIOx;
    manage_object->Activate_GPIO_Pin = GPIO_Pin;
    manage_object->Activate_Level = Activate_Level;
    manage_object->Tx_Buffer_Length = Tx_Length;
    manage_object->Rx_Buffer_Length = 0;

    if (GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, Activate_Level);
    }

    /* 启动失败（通常是 HAL 还是 BUSY）：
     * 必须当场把片选放开、把事务锁释放，否则这条 SPI 就永久卡死了 ——
     * 片选一直有效、事务一直"活跃"，之后每一次调用都返回 HAL_BUSY。 */
    const HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(hspi, manage_object->Tx_Buffer, Tx_Length);
    if (status != HAL_OK)
    {
        manage_object->Start_Failure_Count++;
        manage_object->Last_Start_Failure_Status = static_cast<uint8_t>(status);
        SPI_Deactivate_Chip_Select(manage_object);
        SPI_Release_Transaction(manage_object);
    }
    return static_cast<uint8_t>(status);
}

uint8_t SPI_Transmit_Receive_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                                  GPIO_PinState Activate_Level, const uint8_t *Tx_Data,
                                  uint16_t Tx_Length, uint16_t Rx_Length)
{
    Struct_SPI_Manage_Object *manage_object = SPI_Get_Manage_Object(hspi);
    const uint32_t transfer_length = static_cast<uint32_t>(Tx_Length) + Rx_Length;
    if (manage_object == nullptr || Tx_Data == nullptr || Tx_Length == 0 || Rx_Length == 0 ||
        transfer_length > SPI_BUFFER_SIZE)
    {
        return HAL_ERROR;
    }
    if (!SPI_Try_Acquire_Transaction(manage_object))
    {
        return HAL_BUSY;
    }

    /* 全双工：一次传输 transfer_length 个字节，Tx 不够的部分补 0
     * （BMI088 读寄存器时，地址字节之后要多发 1 个字节的"空数据"把数据移出来） */
    memset(manage_object->Tx_Buffer, 0, transfer_length);
    memset(manage_object->Rx_Buffer, 0, transfer_length);
    memcpy(manage_object->Tx_Buffer, Tx_Data, Tx_Length);
    manage_object->Activate_GPIOx = GPIOx;
    manage_object->Activate_GPIO_Pin = GPIO_Pin;
    manage_object->Activate_Level = Activate_Level;
    manage_object->Tx_Buffer_Length = Tx_Length;
    manage_object->Rx_Buffer_Length = Rx_Length;

    if (GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, Activate_Level);
    }

    /* 同发送函数：启动失败必须当场收尾，否则 SPI 永久卡死 */
    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
        hspi, manage_object->Tx_Buffer, manage_object->Rx_Buffer, static_cast<uint16_t>(transfer_length));
    if (status != HAL_OK)
    {
        manage_object->Start_Failure_Count++;
        manage_object->Last_Start_Failure_Status = static_cast<uint8_t>(status);
        SPI_Deactivate_Chip_Select(manage_object);
        SPI_Release_Transaction(manage_object);
    }
    return static_cast<uint8_t>(status);
}

/* ── HAL 回调：片选收尾 + 释放事务 + 交给上层 ─────────────────────────────
 * 这三个都在【DMA 中断上下文】执行，所以里面不能打日志、不能等。
 * 事务占用锁保证同一时刻只有一条传输，回调里不用再判重入。 */

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    Struct_SPI_Manage_Object *manage_object = SPI_Get_Manage_Object(hspi);
    if (manage_object == nullptr)
    {
        return;
    }
    if (!manage_object->Transaction_Active)
    {
        manage_object->Callback_Anomaly_Count++;
        return;
    }

    SPI_Deactivate_Chip_Select(manage_object);
    SPI_Release_Transaction(manage_object);
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    Struct_SPI_Manage_Object *manage_object = SPI_Get_Manage_Object(hspi);
    if (manage_object == nullptr)
    {
        return;
    }
    if (!manage_object->Transaction_Active)
    {
        manage_object->Callback_Anomaly_Count++;
        return;
    }

    SPI_Deactivate_Chip_Select(manage_object);
    manage_object->Rx_Timestamp = Sys_Get_Micros();

    if (manage_object->Callback_Function != nullptr)
    {
        manage_object->Callback_Function(manage_object->Tx_Buffer, manage_object->Rx_Buffer,
                                         manage_object->Tx_Buffer_Length, manage_object->Rx_Buffer_Length);
    }

    /* ★ 事务锁在【回调返回之后】才释放：本层的约定是"回调里只置标志/唤醒任务，
     *   不inline 发起下一笔传输"（BMI088 驱动就是这么写的 —— 从 DMA 回调里再起
     *   一笔 DMA 会产生竞态）。这个顺序正好把"回调里偷偷起传输"挡在外面：
     *   那时锁还没放开，会拿到 HAL_BUSY，同时 Transaction_Busy_Count 会 +1，
     *   不会被悄悄放过。 */
    SPI_Release_Transaction(manage_object);
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    Struct_SPI_Manage_Object *manage_object = SPI_Get_Manage_Object(hspi);
    if (manage_object == nullptr)
    {
        return;
    }
    if (!manage_object->Transaction_Active)
    {
        manage_object->Callback_Anomaly_Count++;
        return;
    }

    manage_object->Error_Count++;
    SPI_Deactivate_Chip_Select(manage_object);
    SPI_Release_Transaction(manage_object);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
