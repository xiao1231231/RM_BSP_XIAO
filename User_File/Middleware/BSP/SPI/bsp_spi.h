/**
 * @file    bsp_spi.h
 * @brief   SPI 通信层 —— DMA 收发 + 事务占用 + 诊断计数
 *
 * @note    来源：H7_BSP（USTC-RoboWalker / yssickjgd）的 Middleware/BSP/SPI。
 *          为 STM32F407 做了适配，改动清单见 bsp_spi.cpp 顶部。
 *
 *          它比裸 HAL 多做了三件事，这也是它值得移植的原因：
 *            ① 事务占用锁：同一路 SPI 同时只允许一个传输在飞，冲突返回 HAL_BUSY。
 *               SPI 是共享总线，两个调用者交叉发起会串数据，而且症状很隐蔽。
 *            ② 片选自动管理：调用者只说"哪个脚、什么电平算选中"，
 *               收发完成（或出错）后由中断回调自动拉回，不用调用者记着收尾。
 *            ③ 诊断计数：启动失败次数、回调异常次数、总线忙冲突次数，
 *               以及一份"超时现场快照"（寄存器 + DMA 状态 + HAL 状态），
 *               出问题时有据可查，而不是只能猜。
 */

#ifndef BSP_SPI_H
#define BSP_SPI_H

/* Includes ------------------------------------------------------------------*/

#include "sys_timestamp.h"
#include "spi.h"                /* hspi1 + main.h（F4 的 HAL 随 main.h 进来） */
#include <string.h>

#include <stdint.h>
#include <stdbool.h>

/* Exported macros -----------------------------------------------------------*/

/** 收发缓冲区字节长度（单次传输的 Tx+Tx 总长上限） */
#define SPI_BUFFER_SIZE 512

/* Exported types ------------------------------------------------------------*/

/**
 * 一次收发完成后的回调
 * @note  Tx/Rx 指针指向本层内部缓冲区，回调返回后即可能被下一笔覆盖（要留存自己拷贝）。
 *        CS_Port / CS_Pin 是【这一笔传输】的片选（值拷贝）—— 收方靠它认"数据是谁的"，
 *        不必再读管理对象里"最后一次传输的片选"字段，因此分发与
 *        "对象字段何时清空"彻底解耦。
 */
typedef void (*SPI_Callback)(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer,
                             uint16_t Tx_Length, uint16_t Rx_Length,
                             GPIO_TypeDef *CS_Port, uint16_t CS_Pin);

/** 一路 SPI 的管理对象 */
struct Struct_SPI_Manage_Object
{
    SPI_HandleTypeDef *SPI_Handler;
    SPI_Callback       Callback_Function;

    /** 当前是否有传输在飞（起传输前抢占，回调里释放）—— 这就是"事务占用锁" */
    volatile bool Transaction_Active;

    /* ── 片选信号：由本层在回调里自动拉回 ── */
    GPIO_TypeDef  *Activate_GPIOx;
    uint16_t       Activate_GPIO_Pin;
    GPIO_PinState  Activate_Level;

    /* ── 收发缓冲（调用方的数据先 memcpy 进来，函数返回后即可复用） ── */
    uint8_t  Tx_Buffer[SPI_BUFFER_SIZE];
    uint8_t  Rx_Buffer[SPI_BUFFER_SIZE];
    uint16_t Tx_Buffer_Length;
    uint16_t Rx_Buffer_Length;

    /** 本次收发完成的时刻（μs，来自 Sys_Get_Micros） */
    volatile uint64_t Rx_Timestamp;

    /* ── 诊断计数 ── */
    volatile uint32_t Error_Count;             /**< HAL 错误回调次数 */
    volatile uint32_t Transaction_Busy_Count;  /**< 抢占事务锁失败次数（上一次还没发完） */
    volatile uint32_t Start_Failure_Count;     /**< HAL 启动传输就失败了 */
    volatile uint32_t Callback_Anomaly_Count;  /**< 回调来了但没有活跃事务（状态错乱） */
    volatile uint8_t  Last_Start_Failure_Status;  /**< 最后一次启动失败的 HAL 返回值 */
};

/**
 * @brief 超时现场快照
 *
 * @note  Capture_Count 最后写、且前后加内存屏障 —— 调试器里先看它有没有变，
 *        变了说明这份快照是新鲜的，再去读其余字段。
 */
struct Struct_SPI_Timeout_Snapshot
{
    volatile uint32_t Capture_Count;
    volatile uint32_t Timestamp_Low32_Us;
    volatile uint32_t Elapsed_Us;
    volatile uint32_t SPI_CR1;
    volatile uint32_t SPI_CR2;
    volatile uint32_t SPI_SR;
    volatile uint32_t DMA_LISR;
    volatile uint32_t RX_DMA_CR;
    volatile uint32_t RX_DMA_NDTR;
    volatile uint32_t TX_DMA_CR;
    volatile uint32_t TX_DMA_NDTR;
    volatile uint32_t HAL_Error_Code;
    volatile uint16_t HAL_Tx_Xfer_Count;
    volatile uint16_t HAL_Rx_Xfer_Count;
    volatile uint16_t Manager_Tx_Length;
    volatile uint16_t Manager_Rx_Length;
    volatile uint8_t  HAL_State;
    volatile uint8_t  HAL_Lock;
    volatile uint8_t  RX_DMA_State;
    volatile uint8_t  TX_DMA_State;
    volatile uint8_t  Transaction_Active;
    volatile uint8_t  NVIC_Pending_Bits;
    volatile uint8_t  NVIC_Active_Bits;
    volatile uint8_t  Reserved;
};

/* Exported variables ---------------------------------------------------------*/

/** 本工程只用 SPI1（BMI088）。加新 SPI 器件时在这里加一行 */
extern struct Struct_SPI_Manage_Object SPI1_Manage_Object;

/** SPI1 的超时现场快照（调试器里直接看） */
extern struct Struct_SPI_Timeout_Snapshot SPI1_Timeout_Snapshot;

/* Exported function declarations ---------------------------------------------*/

/** @brief 绑定句柄与完成回调。在调度器启动前调用（BMI088 初始化流程里） */
void SPI_Init(SPI_HandleTypeDef *hspi, SPI_Callback Callback_Function);

/**
 * @brief 只发不收（写寄存器用）
 * @return HAL_OK / HAL_BUSY（上一次还没发完）/ HAL_ERROR
 */
uint8_t SPI_Transmit_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                          GPIO_PinState Activate_Level, const uint8_t *Tx_Data, uint16_t Tx_Length);

/**
 * @brief 全双工收发（读寄存器用）
 * @note  实际长度 = Tx_Length + Rx_Length：Tx 部分不够的位置补 0。
 *        BMI088 读寄存器就是"1 字节地址 + 1 字节空数据"这个形状。
 */
uint8_t SPI_Transmit_Receive_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                                  GPIO_PinState Activate_Level, const uint8_t *Tx_Data,
                                  uint16_t Tx_Length, uint16_t Rx_Length);

/** @brief 采集一次超时现场（由上层在判定传输超时时调用） */
void SPI_Capture_Timeout_Snapshot(SPI_HandleTypeDef *hspi, uint32_t Elapsed_Us);

#endif /* BSP_SPI_H */

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
