//
// Created by xiao on 2026/9/26.
//

#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>
#include <stdbool.h>

#include "main.h"        /* UART_HandleTypeDef 在这里 */

#ifdef __cplusplus
extern "C" {
#endif

/** 单个串口的接收缓冲区大小。不定长协议（上位机/视觉）建议给足 */
#define UART_BUFFER_SIZE 128

/**
 * @brief 接收完成回调
 *
 * @param Buffer 收满的帧数据（指向管理对象的 Ready 缓冲）
 * @param Length 本帧字节数
 *
 * @note  ★ 在【中断上下文】执行。只做拷贝和置位级的轻量解析。
 *        不在这里打日志、不做浮点运算、不调用任何可能阻塞的东西。
 *
 * @note  ★★ Buffer 指向管理对象内部！**回调返回后那块内存就可能被下一帧覆盖**。
 *        要么在回调里同步消费完，要么自己 memcpy 走。这是本接口最重要的契约。
 */
typedef void (*UART_Callback)(uint8_t *Buffer, uint16_t Length);

/**
 * @brief 一个串口的"管理对象"
 *
 * @note  全局静态存储：不 malloc、地址编译期确定，调试器里能按名字直接看。
 */
struct Struct_UART_Manage_Object
{
    UART_HandleTypeDef *UART_Handler;
    UART_Callback       Callback_Function;

    /* ── 双缓冲 ── */
    uint8_t  Rx_Buffer_0[UART_BUFFER_SIZE];
    uint8_t  Rx_Buffer_1[UART_BUFFER_SIZE];
    uint8_t *Rx_Buffer_Active;      /* DMA 正在往里写的 */
    uint8_t *Rx_Buffer_Ready;       /* 刚收满、已停止的，可以安全读 */

    /* ── 接收状态 ── */
    uint16_t Rx_Ready_Length;       /* Ready 缓冲里这一帧的长度 */
    uint64_t Rx_Timestamp;          /* 本帧时刻（Sys_Get_Micros）*/
    uint32_t Rx_Error_Count;
    uint32_t Rx_Restart_Count;
    volatile bool Rx_Restart_Pending;   /* 错误后待重启（中断置位，任务清理） */

    /* ── 发送 ── */
    uint8_t  Tx_Buffer[UART_BUFFER_SIZE];
    volatile bool Tx_Submitting;    /* 中断里清零，主循环里读 */
};

/* 串口管理对象全部收在 bsp_uart.cpp 内部的静态池里（UART_Init 时登记），
 * 外部只通过 UART_Init / UART_Transmit_Data / 接收回调使用，不直接碰对象。
 * 加一路串口 = 在 system_init 里多调一次 UART_Init，别处零改动。 */

/**
 * @brief 初始化一个串口：绑定回调、切到第一块缓冲、启动 DMA+IDLE 接收
 * @param huart    CubeMX 生成的句柄（如 &huart1）
 * @param Callback 接收完成回调，可为 NULL（只需发送时）
 */
void UART_Init(UART_HandleTypeDef *huart, UART_Callback Callback);

/**
 * @brief 发送一帧数据
 * @note  非阻塞：拷贝到内部 Tx_Buffer 后启动 DMA，函数返回后调用方即可复用源数据。
 *        忙时返回 HAL_BUSY，不排队。
 * @return HAL_OK / HAL_BUSY / HAL_ERROR
 */
uint8_t UART_Transmit_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length);

/* ────── 以下由 System/callback/callback.cpp 转发调用，业务代码不要直接调 ────── */

/** HAL_UARTEx_RxEventCallback 的落地实现 */
void BSP_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/** HAL_UART_TxCpltCallback 的落地实现 */
void BSP_UART_TxCpltCallback(UART_HandleTypeDef *huart);

/** HAL_UART_ErrorCallback 的落地实现 */
void BSP_UART_ErrorCallback(UART_HandleTypeDef *huart);

/**
 * @brief 接收看门狗：检查是否有串口"接收停了"，是则重启
 * @note  需要周期调用 —— 由 1ms 任务每拍调一次
 */
void BSP_UART_Recover_PeriodElapsedCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_UART_H */