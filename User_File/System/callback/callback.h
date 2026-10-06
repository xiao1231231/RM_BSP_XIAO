#ifndef SYSTEM_CALLBACK_H
#define SYSTEM_CALLBACK_H

#include <stdint.h>
#include "main.h"               /* GPIO_TypeDef —— SPI 回调的片选参数 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 集中分发 HAL 回调
 * @note  本文件【只做转发】，不含任何业务逻辑。
 *        UART / CAN / EXTI 的 HAL __weak 回调在这里覆盖，转给对应 BSP 层。
 *        另有两处历史分工：SPI 的完成/错误回调实现在 bsp_spi.cpp（它要操作
 *        本层的事务状态）；main.c 里的 TIM14 周期回调只是 HAL 时基
 *        （HAL_IncTick）—— 1ms 任务由 RTOS 的 SysTick 节拍驱动，与它无关。
 */
void System_Callback_Init(void);

/**
 * @brief SPI1 收发完成回调（由 bsp_spi 层通过函数指针调用）
 * @note  按参数里的片选分发给对应器件（目前只有 BMI088）。
 *        在 SPI_Init() 时作为回调注册进去，所以要能被 sys_attitude 取到地址。
 */
void SPI1_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer,
                   uint16_t Tx_Length, uint16_t Rx_Length,
                   GPIO_TypeDef *CS_Port, uint16_t CS_Pin);

/**
 * @brief USART6 收帧回调（由 bsp_uart 的接收链路调用）
 * @note  USART6 还没接外设，当前实现只做帧计数（usart6_frame_count）。
 *        接上位机/视觉时把解析逻辑写进这里。运行在中断上下文。
 */
void USART6_Frame_Callback(uint8_t *Buffer, uint16_t Length);

/** USART6 已收到的帧数（调试器可见；接了真实外设后可删） */
extern volatile uint32_t usart6_frame_count;

/**
 * @brief USB 虚拟串口收包回调（由 bsp_usb 的接收链路调用）
 * @note  当前只做包计数（usb_frame_count）。运行在 USB 中断上下文。
 */
void USB_Frame_Callback(uint8_t *Buffer, uint16_t Length);

/** USB 已收到的包数（调试器可见；一次 USB 传输 ≤64B，不是协议帧） */
extern volatile uint32_t usb_frame_count;

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_CALLBACK_H */