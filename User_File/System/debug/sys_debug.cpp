/**
 * @file    sys_debug.cpp
 * @brief   调试输出实现 —— 只有波形（FireWater 协议）
 *
 * @note    ★ 这里刻意【没有】日志函数：串口 1 上只允许出现三轴波形数据。
 *          要排查内部状态请看 sys_debug.h 顶部的说明。
 */

/* Includes ------------------------------------------------------------------*/

#include "sys_debug.h"

#include "main.h"
#include "usart.h"          /* huart1 */
#include "bsp_uart.h"       /* UART_Transmit_Data */

#include <stdio.h>          /* vsnprintf */
#include <stdarg.h>         /* va_list */
#include <string.h>         /* strlen */

/* Private macros ------------------------------------------------------------*/

/** 波形输出的目标串口 */
#define WAVE_UART_HANDLE   (&huart1)

/** 单行最大长度（含 '\0'）。超出被截断，不会溢出 */
#define WAVE_LINE_MAX      128

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 把参数包格式化到缓冲区
 * @note  ★ 必须单独抽一层：__VA_ARGS__ 只能在宏里用，
 *        不能出现在函数体里直接传给 vsnprintf。
 */
static void wave_vformat(char *Buf, size_t Size, const char *Fmt, va_list Ap)
{
  vsnprintf(Buf, Size, Fmt, Ap);
}

/* Function prototypes -------------------------------------------------------*/

void Wave_Output(const char *Fmt, ...)
{
  char line[WAVE_LINE_MAX];

  /* ① 正文 —— FireWater 的一帧就是一行 "前缀:数值,数值,..." */
  va_list ap;
  va_start(ap, Fmt);
  wave_vformat(line, sizeof(line), Fmt, ap);
  va_end(ap);

  /* ② 补换行：FireWater 靠换行分帧。
   *    ★ 必须【保证】有 \r\n —— 空间不足时先截断正文腾位。
   *      少了换行，两帧波形会粘在一起，VOFA+ 解析直接出错。 */
  size_t used = strlen(line);
  if (used > sizeof(line) - 3U)
  {
    used = sizeof(line) - 3U;
  }
  line[used++] = '\r';
  line[used++] = '\n';
  line[used]   = '\0';

  /* ③ 直接交给 BSP 发送。
   *    忙时丢弃（不阻塞）—— 波形丢一帧无所谓，绝不能拖住 1ms 任务。 */
  (void)UART_Transmit_Data(WAVE_UART_HANDLE, (uint8_t *)line, (uint16_t)used);
}
