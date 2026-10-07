#include "bsp_uart.h"

#include "sys_timestamp.h"      /* Sys_Get_Micros() —— 给收到的一帧打时间戳 */
#include <string.h>             /* memcpy */

/* ── 串口管理对象池 ──
 * 所有串口对象收在这里，UART_Init 时登记；实例映射和接收看门狗都遍历这个池。
 * 以前"对象定义 / 映射 if / 看门狗数组 / extern"要登记 4 处，漏任何一处都是
 * 编译照过、运行静默失效 —— 现在收敛成"只在 UART_Init 登记"这一个入口。 */
#define UART_PORT_MAX 4                 /* USART1 / USART6 + 将来接 DBUS 的 USART3，留富余 */
static struct Struct_UART_Manage_Object uart_pool[UART_PORT_MAX];
static uint8_t uart_registered = 0;     /* 只在调度器启动前写（System_Init），之后只读 */

/**
 * @brief 句柄 → 管理对象：遍历登记表比对 Instance
 */
static struct Struct_UART_Manage_Object *uart_get_object(UART_HandleTypeDef *huart)
{
    if (huart == NULL) { return NULL; }
    for (uint8_t i = 0; i < uart_registered; i++)
    {
        if (uart_pool[i].UART_Handler->Instance == huart->Instance)
        {
            return &uart_pool[i];
        }
    }
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
 *        这套动作原来在 UART_Init / RxEventCallback / 看门狗【三个调用点】
 *        各写了一遍，看门狗那条就漏了 ②。收敛成一个函数就是为了不再漏。
 *
 * @return true = 已在接收；false = 启动失败（已记错误并置重启标志，交给看门狗重试）
 */
static bool uart_start_receive(struct Struct_UART_Manage_Object *obj)
{
    const uint32_t errors_before = obj->Rx_Error_Count;

    if (HAL_UARTEx_ReceiveToIdle_DMA(obj->UART_Handler,
                                     obj->Rx_Buffer_Active,
                                     UART_BUFFER_SIZE) != HAL_OK)
    {
        obj->Rx_Error_Count++;
        obj->Rx_Restart_Pending = true;
        return false;
    }

    uart_disable_rx_ht_irq(obj->UART_Handler);

    /* ★ 只有"启动过程中没有新错误发生"才撤销重启请求。
     *   反例（不加这层保护的竞态）：启动刚成功、还没清标志时来了一个接收错误，
     *   错误回调把 Rx_Restart_Pending 置真并停掉接收 —— 这里若无条件清零，
     *   那次恢复请求就被吞掉，串口永久停收（看门狗也不会再重试）。
     *   比较与清标志必须在同一临界区，避免错误中断插在两者之间。 */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (obj->Rx_Error_Count == errors_before)
    {
        obj->Rx_Restart_Pending = false;
    }
    __set_PRIMASK(primask);
    return true;
}

/**
 * @brief 初始化一个串口：登记进对象池、绑定回调、启动 DMA+IDLE 接收
 * @param huart    CubeMX 生成的句柄（如 &huart1）
 * @param Callback 接收完成回调，可为 NULL（只需发送时）
 * @note  重复初始化同一路 / 池满都会被忽略（调度器启动前调用，无并发问题）
 */
void UART_Init(UART_HandleTypeDef *huart, UART_Callback Callback)
{
    if (huart == NULL || uart_registered >= UART_PORT_MAX) { return; }
    if (uart_get_object(huart) != NULL) { return; }     /* 同一路只登记一次 */

    struct Struct_UART_Manage_Object *obj = &uart_pool[uart_registered++];
    obj->UART_Handler       = huart;
    obj->Callback_Function  = Callback;
    obj->Rx_Buffer_Active   = obj->Rx_Buffer_0;
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
        /* ★ 查忙 → 拷贝 → 置位 必须在同一个关中断窗口里完成。
         *   契约承诺"任务和中断里都能调"（sys_debug.h），两个上下文可能并发进来：
         *   A 查完忙、还没拷完就被 B（比如中断里的发送）抢占 —— B 整套跑完、
         *   DMA 开始读 Tx_Buffer；A 恢复后 memcpy 覆盖正在被读的缓冲，
         *   线上出现撕裂帧，且无报错无计数。
         *   窗口内只有 ≤128 字节的 memcpy 和 DMA 寄存器配置（~1µs 量级），
         *   关中断代价可忽略；DMA 启动期间的中断只是挂起，恢复后照常触发。 */
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();

        if (obj->Tx_Submitting)
        {
            __set_PRIMASK(primask);
            return HAL_BUSY;
        }

        /* 拷贝到专用发送缓冲：函数返回后调用方即可复用源数据。
         * F407 没有 D-Cache，拷完直接启动 DMA 即可，不需要任何 Cache 维护。 */
        memcpy(obj->Tx_Buffer, Data, Length);
        obj->Tx_Submitting = true;

        /* HAL 调用也放进窗口：置位/启动/失败回滚成为原子事件，
         * 不存在"标志已置、DMA 却没启动"的中间态 */
        const HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(huart, obj->Tx_Buffer, Length);
        if (status != HAL_OK)
        {
            obj->Tx_Submitting = false;
        }

        __set_PRIMASK(primask);
        return (uint8_t)status;
    }

    /* 没配 TX DMA 的串口：本工程 USART1 / USART6 都配了 TX DMA，走不到这里。
     * 以前这里退化成阻塞发送（最长 100ms），与"非阻塞、忙就丢"的接口契约矛盾 ——
     * 一个叫 UART_Transmit_Data 的函数不该在某些配置下悄悄变成阻塞。
     * 真需要阻塞发送，应该另起一个名字明确的接口。 */
    return (uint8_t)HAL_ERROR;
}

void BSP_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    struct Struct_UART_Manage_Object *obj = uart_get_object(huart);
    if (obj == NULL) { return; }

    /* ① 半传输事件（万一 HT 中断没关干净）不是帧边界，直接丢弃 */
    if (huart->RxEventType == HAL_UART_RXEVENT_HT) { return; }
    if (Size == 0) { return; }

    /* ② 记录接收时刻 */
    obj->Rx_Timestamp = Sys_Get_Micros();

    /* ③ 交换缓冲：刚写满的变成 Ready，另一块拿去接收下一帧
     *    ★ 必须在解析之前做 —— 先让 DMA 继续跑，再慢慢处理数据 */
    uint8_t *just_filled  = obj->Rx_Buffer_Active;
    obj->Rx_Buffer_Active = (just_filled == obj->Rx_Buffer_0) ? obj->Rx_Buffer_1
                                                             : obj->Rx_Buffer_0;

    /* 立刻重新武装接收：先让 DMA 继续跑，再慢慢把刚收到的那帧交给设备层。
     * 启动 DMA + 关 HT 中断都在这个函数里（看门狗走的是同一个）。 */
    (void)uart_start_receive(obj);

    /* ④ 交给设备层解析 */
    if (obj->Callback_Function != NULL)
    {
        obj->Callback_Function(just_filled, Size);
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

    /* ★ 只在【确认 TX DMA 真的没在跑】的时候才放开发送标志。
     *
     *   不能用 gState 判断（这里以前就是那么写的，是错的）：
     *   F4 HAL 的 UART_DMAError()（stm32f4xx_hal_uart.c:3177）只看 CR3.DMAT
     *   开着没有，【不看是哪个流出错】—— RX 流报错时它照样会调
     *   UART_EndTxTransfer() 把 gState 改成 READY，而那个函数
     *   （stm32f4xx_hal_uart.c:3356）只清中断位，根本不碰 DMA 流。
     *   照 gState 放标志 = 允许下一次 memcpy 覆盖正在被 DMA 读的 Tx_Buffer，
     *   把线上那一帧改坏。
     *
     *   判据直接看流本身，满足任一条就认为还在跑、不放：
     *     · State == BUSY     —— HAL 认为它在传
     *     · CR 的 EN 位还立着 —— 硬件真的还开着（不信 HAL 那本状态书）
     *   正常发送结束由 TxCpltCallback 放标志，这里只是兜底。 */
    if (huart->hdmatx == NULL ||
        (huart->hdmatx->State != HAL_DMA_STATE_BUSY &&
         (huart->hdmatx->Instance->CR & DMA_SxCR_EN) == 0U))
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
    /* 遍历登记池：UART_Init 过的串口自动纳入看门狗覆盖，无需再手动登记 */
    for (uint8_t i = 0; i < uart_registered; i++)
    {
        struct Struct_UART_Manage_Object *obj = &uart_pool[i];
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
