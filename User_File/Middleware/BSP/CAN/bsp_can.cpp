/**
 * @file    bsp_can.cpp
 * @brief   CAN 通信层实现
 *
 * @note    接收模型：硬件过滤器全放行 → 报文都进 FIFO0 → 中断里
 *          按 (总线, ID) 匹配已注册的设备 → 调它的回调。
 *          发送模型：直接找空闲邮箱塞帧，三个邮箱都满就立刻返回 false、
 *          不做软件排队 —— 丢帧和重发策略属于应用层（本工程调用方是
 *          1kHz 周期重发的控制帧，选择下一周期再发）。
 */

#include "bsp_can.h"

#include <string.h>

/* ── 设备注册表 ── */
static Struct_CAN_Device can_devices[CAN_DEVICE_MAX];

/* 数量是对中断的【发布点】：接收中断按它遍历注册表，
 * 所以它一变，前面的条目必须已经完整（见 CAN_Register_Device 的注释）。
 * volatile 保证中断里每次都真的去读内存。 */
static volatile uint8_t can_device_count = 0;

/* ── 总线健康 ── */
volatile uint32_t can_bus_off_count = 0;
volatile uint32_t can_last_lec = 0;
static bool can_in_bus_off = false;

bool CAN_Init(CAN_HandleTypeDef *hcan)
{
    if (hcan == nullptr) { return false; }

    /* ★ bxCAN 默认拒绝一切报文，不配过滤器 = 安静地什么都收不到。
     *   掩码全 0 = 每一位都不比对 = 全部放行（设备少，软件按 ID 分发最简单） */
    CAN_FilterTypeDef filter = {};
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank           = 0;      /* CAN1 用 bank0（CAN2 是从机，用 14 起） */
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;

    /* 三步都有返回值，且顺序不能改：
     *   ① 过滤器必须在 Start 之前配好（Start 之后改过滤器要先进初始化模式）
     *   ② 通知必须在 Start 之后开（没启动就没有报文可通知）
     * 任一步失败，"能收能发"这个前提就不成立，必须让调用方看得见 ——
     * 本工程在 System_Init 里蜂鸣报警，避免带着"看起来正常"的假象往下跑。 */
    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        return false;
    }
    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        return false;
    }
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        (void)HAL_CAN_Stop(hcan);
        return false;
    }
    return true;
}

bool CAN_Register_Device(CAN_HandleTypeDef *hcan, uint32_t Rx_ID, void *Id,
                         void (*Callback_Function)(void *Id, uint8_t *Data, uint16_t Length))
{
    if (hcan == nullptr || Callback_Function == nullptr) { return false; }
    if (can_device_count >= CAN_DEVICE_MAX)              { return false; }

    /* ★ 先填字段，最后才发布数量。
     *   CAN 接收中断随时可能进来（CAN_Init 里已经开着了），它按
     *   can_device_count 遍历 —— 数量一旦可见，条目就必须是完整的。
     *   写成 can_devices[can_device_count++] 的后果：中断读到半成品条目
     *   （CAN_Handler 还是 nullptr），那一帧静默不派发。
     *   不会崩，但丢帧不报错，属于最难查的那类问题。 */
    Struct_CAN_Device *device = &can_devices[can_device_count];
    device->CAN_Handler       = hcan;
    device->Rx_ID             = Rx_ID;
    device->Id                = Id;
    device->Callback_Function = Callback_Function;
    device->Rx_Count          = 0;

    /* 写屏障：编译器不许把上面几笔写挪到数量自增之后 */
    __DMB();
    can_device_count++;
    return true;
}

bool CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t Tx_ID,
                  const uint8_t *Data, uint8_t Length)
{
    if (hcan == nullptr || Data == nullptr || Length == 0 || Length > 8)
    {
        return false;
    }
    if (Tx_ID > 0x7FFU)                                /* 标准帧只有 11 位 ID */
    {
        return false;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)   /* 3 个邮箱全在飞 */
    {
        return false;
    }

    CAN_TxHeaderTypeDef header = {};
    header.StdId  = Tx_ID;
    header.IDE    = CAN_ID_STD;          /* 标准 ID（电调只用 11 位） */
    header.RTR    = CAN_RTR_DATA;        /* 数据帧 */
    header.DLC    = Length;
    header.TransmitGlobalTime = DISABLE;

    /* ★ 必须是 8 字节的实体，不能只给 Length 字节：
     *   HAL 无论 DLC 填多少，都会把 aData[0..7] 整个读一遍去填 TDLR/TDHR，
     *   传短缓冲就是从栈上越界读。（"避免 const 强转"不是理由 —— HAL 本来就收 const） */
    uint8_t buffer[8] = {};
    memcpy(buffer, Data, Length);

    /* ★ 最后一个参数是 HAL 的【输出】参数：它把选中的邮箱号写回来
     *   （stm32f4xx_hal_can.c:1285 的 *pTxMailbox = ...，在组帧之前就写）。
     *   传 nullptr 就是往地址 0 写 —— Flash 启动映射下这个写会被悄悄丢掉，
     *   所以以前"看起来能跑"，但它是未定义行为，换启动模式就变成改内存。 */
    uint32_t mailbox = 0U;
    return HAL_CAN_AddTxMessage(hcan, &header, buffer, &mailbox) == HAL_OK;
}

void BSP_CAN_RxFifo0Callback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    /* ★ 先查 fill level 再取。
     *   原来的写法是"取到失败为止"，而 FIFO 空时 HAL 会把
     *   HAL_CAN_ERROR_PARAM 记进 hcan->ErrorCode（stm32f4xx_hal_can.c:1524）——
     *   等于每次正常接收都在污染错误状态，以后真出错就分不出来了。
     *   （bxCAN 每个 FIFO 硬件只有 3 格，这个循环天然最多转 3 圈） */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            return;                     /* 真取失败：剩下的交给下一次中断 */
        }

        /* ★ 只认标准 ID 的数据帧：
         *   · 扩展帧：HAL 只填 ExtId，StdId 留着上一帧的残留值
         *     （stm32f4xx_hal_can.c:1546），拿它比对可能误命中别的设备
         *   · 远程帧：数据区不是真实数据，当电调反馈解析就是垃圾 */
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA)
        {
            continue;
        }

        for (uint8_t i = 0; i < can_device_count; i++)
        {
            Struct_CAN_Device *device = &can_devices[i];
            if (device->CAN_Handler->Instance == hcan->Instance &&
                device->Rx_ID == header.StdId)
            {
                device->Rx_Count++;
                device->Callback_Function(device->Id, data, header.DLC);
            }
        }
    }
}

void BSP_CAN_Service_PeriodElapsedCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == nullptr || hcan->Instance == nullptr) { return; }

    /* bus-off 计数用【边沿】判定，不能用"当前是否 bus-off"：
     * CubeMX 里 AutoBusOff 打开后硬件会自己恢复（约 1.4ms @1Mbps），
     * BOFF 位只反映此刻状态，只有 0→1 的那一下才算一次故障。
     * （若 AutoBusOff 是关的，BOFF 会一直是 1 —— 那是"卡死在 bus-off"，
     *   值应该是 1 不是几千，一眼能看出来区别） */
    const bool bus_off_now = (hcan->Instance->ESR & CAN_ESR_BOFF) != 0U;
    if (bus_off_now && !can_in_bus_off)
    {
        can_bus_off_count++;
    }
    can_in_bus_off = bus_off_now;

    /* LEC = 最近一次总线错误的原因：
     *   1=位填充 2=格式 3=ACK 4=隐性位 5=显性位 6=CRC
     * 最常见的是 3（ACK 错）：总线上没有第二个节点应答 ——
     * 电调没上电、线没接、终端电阻缺失，都会报这个。 */
    const uint32_t lec = (hcan->Instance->ESR & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos;
    if (lec != 0U)
    {
        can_last_lec = lec;
    }
}
