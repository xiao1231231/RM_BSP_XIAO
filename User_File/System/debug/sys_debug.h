/**
 * @file    sys_debug.h
 * @brief   调试输出 —— 格式化一行文本，从【任意一路串口】发出去
 *
 * @note    统一发送接口只有一个：UART_Printf(串口, 格式串, ...)。
 *          想发哪路就传哪路的句柄（&huart1 / &huart6 / 以后的 &huart3），
 *          格式化、补换行、非阻塞发送全部是同一套代码。
 *
 *          发送规则（所有串口一致）：
 *            · 非阻塞：拷进缓冲就返回，绝不拖住调用者（1ms 任务/中断里都能调）
 *            · 忙时丢弃：上一帧还没发完就丢这一帧 —— 调用频率别超过串口带宽
 *              （115200 波特 ≈ 11520 字节/秒，一行 30 字节 ≈ 2.6ms）
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

#ifdef __cplusplus
}
#endif

#endif /* SYS_DEBUG_H */
