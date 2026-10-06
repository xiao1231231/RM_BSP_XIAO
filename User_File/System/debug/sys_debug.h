/**
 * @file    sys_debug.h
 * @brief   调试输出 —— 格式化一行文本，从【任意一路串口】发出去
 *
 * @note    发送接口有两个，对应两条物理链路，格式契约完全一致：
 *            UART_Printf(串口, 格式串, ...) —— 走 UART（USART1/6/3...）
 *            USB_Printf(格式串, ...)      —— 走 USB 虚拟串口（CDC/VCP）
 *          格式化、补换行、非阻塞发送全部是同一套代码。
 *
 *          发送规则（两条链路一致）：
 *            · 非阻塞：拷进缓冲就返回，绝不拖住调用者（1ms 任务/中断里都能调）
 *            · 忙时丢弃：上一帧还没发完就丢这一帧 —— 调用频率别超过通道带宽
 *              （UART 115200 ≈ 11.5KB/s；USB FS 实际可用带宽远高于此）
 *
 *          FireWater/VOFA+ 格式约定（见 UART_Printf 的参数说明）：
 *            一行就是一帧："前缀:数值,数值,..."，冒号只能有一个。
 */

#ifndef SYS_DEBUG_H
#define SYS_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"       /* UART_HandleTypeDef —— 句柄在这里定义 */

/** 单行最大长度（含 '\0'）。超出被截断，不会溢出 */
#define DEBUG_LINE_MAX      128

/**
 * @brief 格式化一行文本并发送到指定串口（★ 统一的对外发送接口）
 *
 * @param huart  目标串口句柄：&huart1（调试/波形）、&huart6（外设接口）……
 * @param Fmt    格式串（用法同 printf，末尾自动补 "\r\n"，不用自己写）。
 *               FireWater/VOFA+ 协议下：
 *               ★ 冒号左边是【前缀文本】（显示在数据区），右边是逗号分隔的纯数值。
 *                 不能写成 "roll:%.2f,pitch:%.2f" ——
 *                 FireWater 只认第一个冒号，后面的 "pitch:" 不是数值会被丢弃，
 *                 结果是只出来一个通道。正确写法："imu:%.2f,%.2f,%.2f"。
 * @param ...    和格式串对应的参数（浮点要转 double 再传，见调用处）
 *
 * @note  行缓冲 128 字节，超出截断不溢出。
 * @note  非阻塞 + 忙时丢弃（见文件顶部说明），可在任务和中断里调用。
 *
 * @example 波形：UART_Printf(&huart1, "imu:%.2f,%.2f,%.2f",
 *                            (double)roll, (double)pitch, (double)yaw);
 * @example 文本：UART_Printf(&huart6, "vbat=%.2fV", (double)voltage);
 */
void UART_Printf(UART_HandleTypeDef *huart, const char *Fmt, ...);

/**
 * @brief 格式化一行文本并通过 USB 虚拟串口发出（格式契约同 UART_Printf）
 * @note  VOFA+ 连接的是电脑上枚举出来的 COMx（"USB 串行设备"）。
 *        USB 没插电脑 / 未枚举时安全：底层返回忙、本帧丢弃，不阻塞不崩溃。
 */
void USB_Printf(const char *Fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* SYS_DEBUG_H */
