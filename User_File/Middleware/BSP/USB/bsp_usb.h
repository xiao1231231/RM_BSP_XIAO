/**
 * @file    bsp_usb.h
 * @brief   USB 虚拟串口（CDC / VCP）—— 对 CubeMX 生成的 usbd_cdc_if 的薄封装
 *
 * @note    分工：USB 协议栈（Middlewares/ST）、端点管理、包封装全在 CubeMX
 *          生成的代码里；本层只做三件事 —— 调初始化、把收到的包转发给
 *          注册的回调、提供一个非阻塞发送。脏活协议栈都干了，所以这层很薄。
 *
 *          与 UART 层的契约对齐：
 *            · 接收回调运行在【USB 中断上下文】，只做拷贝/置标志等轻活；
 *            · Buffer 只在回调期间有效（重新武装接收后会被下一包覆盖），
 *              要留存自己 memcpy；
 *            · 发送非阻塞，忙就返回 false，不排队。
 */

#ifndef BSP_USB_H
#define BSP_USB_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 接收回调类型（与 UART_Callback 同语义）。一"包"= 一次 USB 传输（≤64B@FS），
 *  不是"一帧协议"——帧边界要靠上层协议自己解析 */
typedef void (*USB_Rx_Callback)(uint8_t *Buffer, uint16_t Length);

/**
 * @brief 初始化 USB 虚拟串口（内部调 CubeMX 的 MX_USB_DEVICE_Init）
 * @param Callback 接收回调，可为 NULL（只发不收）
 * @note  调度器启动前调用。device-only 模式下初始化只配寄存器、注册 CDC 类，
 *        不创建 RTOS 对象，放 System_Init 里安全。
 */
void USB_Init(USB_Rx_Callback Callback);

/**
 * @brief 非阻塞发送
 * @return true = 已交给协议栈；false = 未枚举 / 端点忙（上一帧还没被主机取走）/ 参数错
 * @note  忙就丢，不排队。★ 本层自己持有发送缓冲并先拷贝再提交 ——
 *        不能把调用方缓冲直接交给协议栈：HAL 的 PCD 发送是异步的
 *        （数据在后续 TX FIFO 空中断里才被读走），栈上缓冲会提前失效。
 *        因为拷贝过，调用方函数返回后即可复用 Data。
 * @note  长度上限 128 字节（超过返回 false）。
 */
bool USB_Transmit(const uint8_t *Data, uint16_t Length);

/* 以下由 USB_DEVICE/App/usbd_cdc_if.c 的 CDC_Receive_FS 转发调用，业务代码不要直接调 */
void BSP_USB_RxCallback(uint8_t *Buffer, uint32_t Length);

#ifdef __cplusplus
}
#endif

#endif /* BSP_USB_H */
