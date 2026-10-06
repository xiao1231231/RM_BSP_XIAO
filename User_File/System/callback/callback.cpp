#include "callback.h"
#include "main.h"
#include "usart.h"
#include "bsp_uart.h"
#include "bsp_spi.h"
#include "bsp_bmi088.h"
#include "bsp_can.h"

/* ══════════════ USART6 收帧占位 ══════════════
 *
 * USART6（PG14/PG9，3-pin UART 接口）还没接外设，这里先只做帧计数，
 * 证明"接收链路活着"（已用 USB 转 TTL 验证过：收发回显全通）。
 * 接上位机/视觉时，把解析逻辑写在这里。
 *
 * ⚠️ 运行在【中断上下文】（DMA/IDLE 回调）—— 只做轻量的事：
 *    拷走数据 / 置标志 / 计数。UART_Transmit_Data 非阻塞、可在中断里调，
 *    但批量数据请先 memcpy 到自己的缓冲，别在回调里做重活。
 */
volatile uint32_t usart6_frame_count = 0;

void USART6_Frame_Callback(uint8_t *Buffer, uint16_t Length)
{
    (void)Buffer;
    (void)Length;
    usart6_frame_count++;       /* 收到一帧就 +1，供调试器观察 */
}

/* ══════════════ USB CDC 收包占位 ══════════════
 *
 * USB 虚拟串口当前只发不收（波形），收链路保持活着：只做包计数。
 * ⚠️ 运行在【USB 中断上下文】—— 一"包"是一次 USB 传输（FS 下 ≤64 字节）。
 */
volatile uint32_t usb_frame_count = 0;

void USB_Frame_Callback(uint8_t *Buffer, uint16_t Length)
{
    (void)Buffer;
    (void)Length;
    usb_frame_count++;
}

/* ══════════════ UART ══════════════
 *
 * ★★★ 每个 HAL 回调都必须加 extern "C"
 *
 *   HAL 里这些回调是用 C 编译的 __weak 空函数。C++ 会把函数名 mangling：
 *       你写的：      void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef*, uint16_t)
 *       生成的符号：  _Z25HAL_UARTEx_RxEventCallbackP18UART_HandleTypeDeft
 *       HAL 期待的：  HAL_UARTEx_RxEventCallback
 *                     ↑ 对不上 → 覆盖不了 → 中断来了调的还是空函数
 *
 *   症状：【编译链接全过，但回调永远不触发】—— 串口收不到数据，
 *         而且没有任何报错可查。
 *
 *   验证：arm-none-eabi-nm --defined-only build\Debug\RM_F407.elf | findstr HAL_UART
 *         看到 "T HAL_UARTEx_RxEventCallback" 就对；看到 "_Z25..." 就是漏了。
 *
 *   漏一个，那个回调就永远不触发。所以这个文件里【每一个】都要加。
 */

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    BSP_UART_RxEventCallback(huart, Size);
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_UART_TxCpltCallback(huart);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    BSP_UART_ErrorCallback(huart);
}

/* ══════════════ SPI：BMI088 ══════════════
 *
 * bsp_spi 层每收完一笔就回调这里 —— ★ 运行在 DMA 完成中断上下文。
 * 分发只看回调参数里带的片选（这一笔传输是谁的），不读任何全局对象 ——
 * 与 bsp_spi 内部"事务字段何时清空"的顺序彻底无关。
 * 之后由 bsp_bmi088 里的回调去解析数据、置状态、唤醒 BMI088_Task。
 */
extern "C" void SPI1_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer,
                              uint16_t Tx_Length, uint16_t Rx_Length,
                              GPIO_TypeDef *CS_Port, uint16_t CS_Pin)
{
    (void)Tx_Buffer;
    (void)Rx_Buffer;
    (void)Tx_Length;
    (void)Rx_Length;

    if ((CS_Port == CS1_ACCEL_GPIO_Port && CS_Pin == CS1_ACCEL_Pin) ||
        (CS_Port == CS1_GYRO_GPIO_Port  && CS_Pin == CS1_GYRO_Pin))
    {
        BSP_BMI088.SPI_RxCpltCallback(CS_Port, CS_Pin);
    }
}

/* ══════════════ EXTI：BMI088 数据就绪 ══════════════
 *
 * PC4 = INT1_Accel（加速度就绪）、PC5 = INT1_Gyro（陀螺 FIFO 就绪）。
 * 两个脚都配成上升沿（BMI088 的 INT 输出是推挽高有效，边沿要对上）。
 */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BMI088_ACCEL_INT_Pin || GPIO_Pin == BMI088_GYRO_INT_Pin)
    {
        BSP_BMI088.EXTI_Flag_Callback(GPIO_Pin);
    }
}

/* ══════════════ CAN：FIFO0 收报 ══════════════
 *
 * bsp_can 把所有过滤器都指向 FIFO0，所以接收回调只有这一个。
 * "MsgPending" = FIFO 里躺了报文待取；真正的取帧和按 ID 分发在 bsp_can 里。
 */
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    BSP_CAN_RxFifo0Callback(hcan);
}

/* ── 以后新增外设的回调加在这里，同样【每一个】都要 extern "C" ──
 * 已有的：UART ×3（上面）、CAN 收报、BMI088 的 SPI / EXTI。
 * 将来可能的：HAL_ADC_ConvCpltCallback()（ADC 采样完成）等。
 */

