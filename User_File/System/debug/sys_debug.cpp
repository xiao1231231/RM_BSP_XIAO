/**
 * @file    sys_debug.cpp
 * @brief   调试输出实现 —— 格式化 + 指定通道发送（统一接口）
 *
 * @note    本文件刻意【没有】日志函数：调试通道上只允许出现调用者明确要发的内容。
 *          两条发送链路共用同一套格式化（FireWater 一帧 = 一行"前缀:数值,..."）：
 *            UART_Printf(huart, ...) → UART_Transmit_Data（BSP：拷贝 + 启动 DMA）
 *            USB_Printf(...)         → USB_Transmit      （BSP：交协议栈，忙则丢）
 *          发什么、多久发一次由调用方（TIM_1ms_Task）决定。
 */

/* Includes ------------------------------------------------------------------*/

#include "sys_debug.h"

#include "bsp_uart.h"       /* UART_Transmit_Data */
#include "bsp_usb.h"        /* USB_Transmit */

#include <stdio.h>          /* vsnprintf */
#include <stdarg.h>         /* va_list */
#include <string.h>         /* strlen */

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 格式化一行 + 补 "\r\n"，返回实际写入的字节数
 * @note  ① 参数包必须经这层函数转给 vsnprintf —— __VA_ARGS__ 只能在宏里展开；
 *        ② ★ 必须【保证】行尾有 \r\n —— FireWater 靠换行分帧，
 *          缺了换行两帧粘连，上位机解析直接出错。空间不足先截断正文腾位。
 */
static uint16_t debug_format_line(char *Buf, size_t Size, const char *Fmt, va_list Ap)
{
  vsnprintf(Buf, Size, Fmt, Ap);

  size_t used = strlen(Buf);
  if (used > Size - 3U)
  {
    used = Size - 3U;
  }
  Buf[used++] = '\r';
  Buf[used++] = '\n';
  Buf[used]   = '\0';
  return (uint16_t)used;
}

/* Function prototypes -------------------------------------------------------*/

void UART_Printf(UART_HandleTypeDef *huart, const char *Fmt, ...)
{
  if (huart == NULL || Fmt == NULL)
  {
    return;
  }

  char line[DEBUG_LINE_MAX];

  va_list ap;
  va_start(ap, Fmt);
  const uint16_t used = debug_format_line(line, sizeof(line), Fmt, ap);
  va_end(ap);

  /* 非阻塞 + 忙时丢弃 —— 绝不拖住调用者 */
  (void)UART_Transmit_Data(huart, (uint8_t *)line, used);
}

void USB_Printf(const char *Fmt, ...)
{
  if (Fmt == NULL)
  {
    return;
  }

  char line[DEBUG_LINE_MAX];

  va_list ap;
  va_start(ap, Fmt);
  const uint16_t used = debug_format_line(line, sizeof(line), Fmt, ap);
  va_end(ap);

  /* USB 未连接电脑时底层按忙处理、本包丢弃 —— 不阻塞也不崩溃 */
  (void)USB_Transmit((const uint8_t *)line, used);
}
