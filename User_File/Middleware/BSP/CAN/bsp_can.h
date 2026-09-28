/**
* @file    bsp_can.h
 * @brief   CAN 通信层 —— 过滤器/启动、按 ID 分发接收、非阻塞发送
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"       /* CAN_HandleTypeDef */

/** 最多注册的接收设备数（4 个电调 + 富余） */
#define CAN_DEVICE_MAX  8

/** 一个"接收者"：关心哪个 ID、收到后调谁 */
struct Struct_CAN_Device
{
    CAN_HandleTypeDef *CAN_Handler;
    uint32_t Rx_ID;                 /* 标准 ID，如 0x201 = 1 号 3508 的反馈 */
    void *Id;                       /* 注册者的上下文（如电机对象地址），回调时原样带回 */
    void (*Callback_Function)(void *Id, uint8_t *Data, uint16_t Length);
    volatile uint32_t Rx_Count;     /* 收帧计数，诊断用 */
};

/** 初始化一路 CAN：过滤器(全收) → 启动 → 打开 FIFO0 收报中断。调度器启动前调用 */
void CAN_Init(CAN_HandleTypeDef *hcan);

/** 注册一个接收者（Id 原样带回给回调，用于区分"这是哪个对象"）。满了返回 false */
bool CAN_Register_Device(CAN_HandleTypeDef *hcan, uint32_t Rx_ID, void *Id,
                         void (*Callback_Function)(void *Id, uint8_t *Data, uint16_t Length));

/** 发送一帧（标准 ID，数据帧，最多 8 字节）。没有空闲邮箱时返回 false，不排队 */
bool CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t Tx_ID,
                  const uint8_t *Data, uint8_t Length);

/* 以下由 System/callback 转发调用，业务代码不要直接调 */
void BSP_CAN_RxFifo0Callback(CAN_HandleTypeDef *hcan);

#endif /* BSP_CAN_H */