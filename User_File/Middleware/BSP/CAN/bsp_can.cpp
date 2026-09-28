/**
 * @file    bsp_can.cpp
 * @brief   CAN 通信层实现
 *
 * @note    接收模型：硬件过滤器全放行 → 报文都进 FIFO0 → 中断里
 *          按 (总线, ID) 匹配已注册的设备 → 调它的回调。
 *          发送模型：直接找空闲邮箱塞帧，忙就丢（调用方下一周期重试），
 *          电调控制帧本来就是周期性重发的，丢一帧无所谓。
 */

#include "bsp_can.h"

#include <string.h>

/* ── 设备注册表 ── */
static Struct_CAN_Device can_devices[CAN_DEVICE_MAX];
static uint8_t can_device_count = 0;

void CAN_Init(CAN_HandleTypeDef *hcan)
{
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
    HAL_CAN_ConfigFilter(hcan, &filter);

    /* 过滤器必须在 Start 之前配好；Start 后才有报文流动 */
    HAL_CAN_Start(hcan);

    /* FIFO0 有报文待取 → 触发中断（接收路径的唯一中断源） */
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

bool CAN_Register_Device(CAN_HandleTypeDef *hcan, uint32_t Rx_ID, void *Id,
                         void (*Callback_Function)(void *Id, uint8_t *Data, uint16_t Length))
{
    if (hcan == nullptr || Callback_Function == nullptr) { return false; }
    if (can_device_count >= CAN_DEVICE_MAX)              { return false; }

    Struct_CAN_Device *device = &can_devices[can_device_count++];
    device->CAN_Handler      = hcan;
    device->Rx_ID            = Rx_ID;
    device->Id               = Id;
    device->Callback_Function = Callback_Function;
    device->Rx_Count         = 0;
    return true;
}

bool CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t Tx_ID,
                  const uint8_t *Data, uint8_t Length)
{
    if (hcan == nullptr || Data == nullptr || Length == 0 || Length > 8)
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

    uint8_t buffer[8] = {};              /* 拷贝一份，避免 const 强转 */
    memcpy(buffer, Data, Length);
    return HAL_CAN_AddTxMessage(hcan, &header, buffer, nullptr) == HAL_OK;
}

void BSP_CAN_RxFifo0Callback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    /* 排空 FIFO：一次中断可能积了多帧，全取完（取空时 HAL 返回 ERROR 退出） */
    while (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) == HAL_OK)
    {
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