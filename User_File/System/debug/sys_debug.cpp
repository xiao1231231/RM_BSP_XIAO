/**
 * @file    sys_debug.cpp
 * @brief   调试输出实现 —— 格式化 + 指定串口发送（统一接口）
 *
 * @note    本文件刻意【没有】日志函数：串口上只允许出现调用者明确要发的内容。
 *          发送链路的三层分工：
 *            TIM_1ms_Task（决定发什么、多久发一次）
 *              → UART_Printf（这里：格式化成一行）
 *                → UART_Transmit_Data（BSP：拷贝 + 启动 DMA，立刻返回）
 */

/* Includes ------------------------------------------------------------------*/

#include "sys_debug.h"

#include "bsp_uart.h"       /* UART_Transmit_Data */

#include <stdio.h>          /* vsnprintf */
#include <stdarg.h>         /* va_list */
#include <string.h>         /* strlen */

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 把参数包格式化到缓冲区
 * @note  ★ 必须单独抽一层：__VA_ARGS__ 只能在宏里用，
 *        不能出现在函数体里直接传给 vsnprintf。
 */
static void debug_vformat(char *Buf, size_t Size, const char *Fmt, va_list Ap)
{
  vsnprintf(Buf, Size, Fmt, Ap);
}

/* Function prototypes -------------------------------------------------------*/

void UART_Printf(UART_HandleTypeDef *huart, const char *Fmt, ...)
{
  if (huart == NULL || Fmt == NULL)
  {
    return;
  }

  char line[DEBUG_LINE_MAX];

  /* ① 正文 —— FireWater 的一帧就是一行 "前缀:数值,数值,..." */
  va_list ap;
  va_start(ap, Fmt);
  debug_vformat(line, sizeof(line), Fmt, ap);
  va_end(ap);

  /* ② 补换行：FireWater 靠换行分帧。
   *    ★ 必须【保证】有 \r\n —— 空间不足时先截断正文腾位。
   *      少了换行，两帧会粘在一起，上位机解析直接出错。 */
  size_t used = strlen(line);
  if (used > sizeof(line) - 3U)
  {
    used = sizeof(line) - 3U;
  }
  line[used++] = '\r';
  line[used++] = '\n';
  line[used]   = '\0';

  /* ③ 交给 BSP 发送（非阻塞 + 忙时丢弃 —— 绝不拖住调用者） */
  (void)UART_Transmit_Data(huart, (uint8_t *)line, (uint16_t)used);
}
