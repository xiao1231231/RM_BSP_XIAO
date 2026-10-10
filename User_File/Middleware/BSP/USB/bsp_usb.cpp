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

#include "main.h"           /* __get_PRIMASK / __disable_irq —— CMSIS 内联 */
#include "usb_device.h"     /* MX_USB_DEVICE_Init —— CubeMX 生成的入口 */
#include "usbd_cdc_if.h"    /* CDC_Transmit_FS / USBD_OK / USBD_CDC_HandleTypeDef */

#include <string.h>         /* memcpy */

/* hUsbDeviceFS 定义在 USB_DEVICE/App/usb_device.c（C 链接）——
 * 这里只为读它的 pClassData（协议栈的 CDC 句柄，含 TxState 忙标志） */
extern "C" USBD_HandleTypeDef hUsbDeviceFS;

static USB_Rx_Callback usb_rx_callback = nullptr;

/* ── 发送缓冲：必须由 BSP 自己持有，不能用调用方的 ──
 *
 * ★ HAL 的 PCD 发送是【异步】的：
 *     HAL_PCD_EP_Transmit (stm32f4xx_hal_pcd.c:1939) 只保存指针
 *     → 非 DMA 路径（本工程 usbd_conf.c:338 里 dma_enable = DISABLE）
 *       不拷数据，只使能端点和"TX FIFO 空"中断
 *     → 数据是后续的 TX FIFO 空中断里由 PCD_WriteEmptyTxFifo
 *       (stm32f4xx_hal_pcd.c:2219) 从那个指针读走的
 *   所以把调用方的缓冲（例如 USB_Printf 的栈上 line[]）直接交出去，
 *   函数一返回栈就可能被复用 —— 发出去的会是被覆盖的数据。
 *   本层拷贝进下面的静态缓冲后，调用方返回即可复用 Data。
 *
 * 覆盖时机：只有协议栈空闲（TxState == 0，上一笔已全部搬进 FIFO）才覆盖。
 * 关中断窗口把"查空闲 → 拷贝 → 提交"做成一个原子事件：既挡住 TX 完成中断
 * 改 TxState，也挡住其它调用者并发进来（当前由 USB_Output_Task 调用）。
 */
#define USB_TX_BUFFER_SIZE  128U        /* ≥ 最大行（sys_debug 的 DEBUG_LINE_MAX） */
alignas(4) static uint8_t usb_tx_buffer[USB_TX_BUFFER_SIZE];

void USB_Init(USB_Rx_Callback Callback)
{
    usb_rx_callback = Callback;

    // System_Init 在调度器启动前显式初始化 USB。
    MX_USB_DEVICE_Init();
}

bool USB_Transmit(const uint8_t *Data, uint16_t Length)
{
    if (Data == nullptr || Length == 0 || Length > USB_TX_BUFFER_SIZE)
    {
        return false;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const USBD_CDC_HandleTypeDef *cdc =
        static_cast<const USBD_CDC_HandleTypeDef *>(hUsbDeviceFS.pClassData);

    bool accepted = false;
    /* 未枚举完成（pClassData 为空）或上一笔还在飞（TxState != 0）→ 丢弃本帧。
     * 两个条件都满足才敢动 usb_tx_buffer —— 否则会覆盖正在被中断读取的数据 */
    if (cdc != nullptr && cdc->TxState == 0U)
    {
        memcpy(usb_tx_buffer, Data, Length);
        accepted = (CDC_Transmit_FS(usb_tx_buffer, Length) == USBD_OK);
    }

    __set_PRIMASK(primask);
    return accepted;
}

void BSP_USB_RxCallback(uint8_t *Buffer, uint32_t Length)
{
    if (usb_rx_callback != nullptr)
    {
        usb_rx_callback(Buffer, (uint16_t)Length);
    }
}
