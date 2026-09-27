#include "bsp_uart.h"

#include "sys_timestamp.h"      /* Sys_Get_Micros() —— 依赖第 02 章 */
#include <string.h>             /* memcpy */

/* ── 本工程用到的串口管理对象 ──
 * 全局静态存储：不 malloc、地址编译期确定、调试器里能按名字直接看。 */
struct Struct_UART_Manage_Object USART1_Manage_Object;

/**
 * @brief 句柄 → 管理对象 的映射
 * @note  加新串口时在这里加一行（第 04 章加 USART3）
 */
static struct Struct_UART_Manage_Object *uart_get_object(UART_HandleTypeDef *huart)
{
    if (huart == NULL)             { return NULL; }
    if (huart->Instance == USART1) { return &USART1_Manage_Object; }
    return NULL;
}

/**
 * @brief 关掉 RX DMA 的"半传输"中断
 *
 * @note  ★ 必须放在【每次启动接收之后】调用
 *        原因：HAL_UARTEx_ReceiveToIdle_DMA() 内部会设置
 *        hdmarx->XferHalfCpltCallback，而 HAL_DMA_Start_IT() 只要看到
 *        这个回调非空就会重新打开 HTIE（stm32f4xx_hal_dma.c 第 481 行）。
 *        在启动接收【之前】关，等于白关。
 *
 * @note  半传输事件不是帧边界，回调里本来也会过滤掉，所以这只是省掉
 *        多余的中断开销，不影响功能。
 */
static void uart_disable_rx_ht_irq(UART_HandleTypeDef *huart)
{
    if (huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

/**
 * @brief 启动一轮 RX DMA 接收 —— 三个调用点共用的唯一实现
 *
 * @note  "启动接收"是两个动作，缺一不可：
 *        ① HAL_UARTEx_ReceiveToIdle_DMA() —— 启动 DMA + IDLE 中断
 *        ② uart_disable_rx_ht_irq()       —— 关掉半传输中断（① 内部会把它重新打开）
 *        ★ 顺序不能反，先关再启动等于白关。
 *
 *        这三步原来在 UART_Init / RxEventCallback / 看门狗里各写了一遍，
 *        看门狗那条就漏了 ②。收敛成一个函数就是为了不再漏。
 *
 * @return true = 已在接收；false = 启动失败（已记错误并置重启标志，交给看门狗重试）
 */
static bool uart_start_receive(struct Struct_UART_Manage_Object *obj)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(obj->UART_Handler,
                                     obj->Rx_Buffer_Active,
                                     UART_BUFFER_SIZE) != HAL_OK)
    {
        obj->Rx_Error_Count++;
        obj->Rx_Restart_Pending = true;
        return false;
    }

    uart_disable_rx_ht_irq(obj->UART_Handler);
    obj->Rx_Restart_Pending = false;      /* 接收已经在跑，撤销重启请求 */
    return true;
}

void UART_Init(UART_HandleTypeDef *huart, UART_Callback Callback)
{
    struct Struct_UART_Manage_Object *obj = uart_get_object(huart);
    if (obj == NULL) { return; }

    obj->UART_Handler       = huart;
    obj->Callback_Function  = Callback;
    obj->Rx_Buffer_Active   = obj->Rx_Buffer_0;
    obj->Rx_Buffer_Ready    = obj->Rx_Buffer_1;
    obj->Rx_Ready_Length    = 0;
    obj->Rx_Timestamp       = 0;
    obj->Rx_Error_Count     = 0;
    obj->Rx_Restart_Count   = 0;
    obj->Rx_Restart_Pending = false;
    obj->Tx_Submitting      = false;

    /* 启动第一轮接收（写 Rx_Buffer_Active = Rx_Buffer_0）。
     * 启动 + 关 HT 中断两步统一在 uart_start_receive() 里，不再各写一遍。 */
    (void)uart_start_receive(obj);
}

uint8_t UART_Transmit_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length)
{
    struct Struct_UART_Manage_Object *obj = uart_get_object(huart);
    if (obj == NULL || Data == NULL || Length == 0) { return HAL_ERROR; }
    if (Length > UART_BUFFER_SIZE)                  { return HAL_ERROR; }

    if (huart->hdmatx != NULL)
    {
        /* 上一次 DMA 还没发完 → 返回忙，不覆盖正在发送的内容 */
        if (obj->Tx_Submitting) { return HAL_BUSY; }

        /* 拷贝到专用发送缓冲：函数返回后调用方即可复用源数据。
         * F407 没有 D-Cache，拷完直接启动 DMA 即可，不需要任何 Cache 维护。 */
        memcpy(obj->Tx_Buffer, Data, Length);
        obj->Tx_Submitting = true;

        if (HAL_UART_Transmit_DMA(huart, obj->Tx_Buffer, Length) != HAL_OK)
        {
            obj->Tx_Submitting = false;
            return HAL_ERROR;
        }
        return HAL_OK;
    }

    /* 没有配 TX DMA 的串口走阻塞发送（本工程 USART1 配了，走不到这里） */
    return (uint8_t)HAL_UART_Transmit(huart, Data, Length, 100);
}

void BSP_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    struct Struct_UART_Manage_Object *obj = uart_get_object(huart);
    if (obj == NULL) { return; }

    /* ① 半传输事件（万一 HT 中断没关干净）不是帧边界，直接丢弃 */
    if (huart->RxEventType == HAL_UART_RXEVENT_HT) { return; }
    if (Size == 0) { return; }

    /* ② 记录帧长与时刻 */
    obj->Rx_Ready_Length = Size;
    obj->Rx_Timestamp    = Sys_Get_Micros();

    /* ③ 交换缓冲：刚写满的变成 Ready，另一块拿去接收下一帧
     *    ★ 必须在解析之前做 —— 先让 DMA 继续跑，再慢慢处理数据 */
    uint8_t *just_filled  = obj->Rx_Buffer_Active;
    obj->Rx_Buffer_Ready  = just_filled;
    obj->Rx_Buffer_Active = (just_filled == obj->Rx_Buffer_0) ? obj->Rx_Buffer_1
                                                             : obj->Rx_Buffer_0;

    /* 立刻重新武装接收：先让 DMA 继续跑，再慢慢把刚收到的那帧交给设备层。
     * 启动 DMA + 关 HT 中断都在这个函数里（看门狗走的是同一个）。 */
    (void)uart_start_receive(obj);

    /* ④ 交给设备层解析 */
    if (obj->Callback_Function != NULL)
    {
        obj->Callback_Function(obj->Rx_Buffer_Ready, obj->Rx_Ready_Length);
    }
}

void BSP_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    struct Struct_UART_Manage_Object *obj = uart_get_object(huart);
    if (obj != NULL) { obj->Tx_Submitting = false; }
}

void BSP_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    struct Struct_UART_Manage_Object *obj = uart_get_object(huart);
    if (obj == NULL) { return; }

    obj->Rx_Error_Count++;

    /* ★ 只在【确认 TX 已经不在跑】的时候才放开发送标志。
     *
     *   这个回调是 RX / TX 共用的：接收错误（ORE / PE / FE / NE）时 HAL 只中止
     *   RX（UART_EndRxTransfer），TX DMA 很可能还在发。此时若放开标志，
     *   下一次 UART_Transmit_Data() 就会 memcpy 覆盖【正在被 DMA 读的 Tx_Buffer】，
     *   把线上那一帧改坏。
     *
     *   判据用 gState：真 TX 出错时 HAL 内部 UART_EndTxTransfer() 已经把它归位成
     *   READY，所以"TX 出错 → 标志永远为真 → 日志永久静默"那个场景照样被覆盖；
     *   发送正常结束时由 TxCpltCallback 放标志。 */
    if (huart->gState != HAL_UART_STATE_BUSY_TX)
    {
        obj->Tx_Submitting = false;
    }

    /* 只置标志，不在中断里重启。
     * HAL 的错误路径已经把 RxState 放回 READY 并停了 DMA，理论上可以立刻重启，
     * 但统一交给看门狗重试，这样错误处理路径是【幂等】的 —— 重试多少次都安全。 */
    obj->Rx_Restart_Pending = true;
}

void BSP_UART_Recover_PeriodElapsedCallback(void)
{
    /* 加新串口时往这个数组加一项 */
    struct Struct_UART_Manage_Object *objs[] = { &USART1_Manage_Object };

    for (uint32_t i = 0; i < sizeof(objs) / sizeof(objs[0]); i++)
    {
        struct Struct_UART_Manage_Object *obj = objs[i];
        if (obj->UART_Handler == NULL) { continue; }
        if (!obj->Rx_Restart_Pending)  { continue; }

        /* 只有 HAL 把状态放开（READY）才能重启，否则返回 HAL_BUSY */
        if (obj->UART_Handler->RxState != HAL_UART_STATE_READY) { continue; }

        /* ★ 走和初始化/正常收帧同一个函数 —— 这里以前漏了"关 HT 中断"那一步 */
        if (uart_start_receive(obj))
        {
            obj->Rx_Restart_Count++;
        }
    }
}