/**
 * @file    bsp_usb.cpp
 * @brief   USB 虚拟串口实现 —— 桥接 CubeMX 的 usbd_cdc_if（见 .h 顶部说明）
 *
 * @note    桥接关系：
 *            发送：本层 USB_Transmit → CDC_Transmit_FS（usbd_cdc_if.c）
 *            接收：CDC_Receive_FS（usbd_cdc_if.c，USER CODE 区）
 *                  → 本层 BSP_USB_RxCallback → 注册的回调
 */

#include "bsp_usb.h"

#include "usb_device.h"     /* MX_USB_DEVICE_Init —— CubeMX 生成的入口 */
#include "usbd_cdc_if.h"    /* CDC_Transmit_FS / USBD_OK */

static USB_Rx_Callback usb_rx_callback = nullptr;

void USB_Init(USB_Rx_Callback Callback)
{
    usb_rx_callback = Callback;

    /* ★ 为什么必须在这里调：CubeMX 把 MX_USB_DEVICE_Init 放进了 __weak
     * TIM_1ms_Task 空壳（freertos.c），而本工程的任务是强实现覆盖那个壳
     * —— 那里的初始化永远不会执行，USB 将完全不动。 */
    MX_USB_DEVICE_Init();
}

bool USB_Transmit(const uint8_t *Data, uint16_t Length)
{
    if (Data == nullptr || Length == 0) { return false; }

    /* CDC_Transmit_FS 返回 USBD_OK=0 / USBD_BUSY=1 / USBD_FAIL=2。
     * 忙 = 上一包还没被主机取走（CDC 的 IN 端点只有一个包深）→ 丢弃本包 */
    return CDC_Transmit_FS((uint8_t *)Data, Length) == USBD_OK;
}

void BSP_USB_RxCallback(uint8_t *Buffer, uint32_t Length)
{
    if (usb_rx_callback != nullptr)
    {
        usb_rx_callback(Buffer, (uint16_t)Length);
    }
}
