#ifndef SYS_TIMESTAMP_H
#define SYS_TIMESTAMP_H

#include <stdint.h>
#include <stdbool.h>

/* ★ 必须包含 main.h：下面 Sys_Get_Cycle() 直接引用 DWT，
 *   而 DWT 是由 CMSIS 的 stm32f4xx.h（经 main.h 间接引入）声明的。
 *   头文件要自给自足，不能指望使用者先 include 了 main.h。 */
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/** CPU 主频，与 CubeMX 时钟树里的 HCLK 一致（本工程 168MHz） */
#define SYS_CPU_FREQ_MHZ 168U

/** @brief 初始化 DWT 计数器。在 System_Init() 最开头调用 */
void Sys_Timestamp_Init(void);

/**
 * @brief 读取原始 CYCCNT 值
 * @note  32 位，@168MHz 每 25.57 秒回绕一次
 */
static inline uint32_t Sys_Get_Cycle(void)
{
    return DWT->CYCCNT;
}

/**
 * @brief 计算自上次调用以来经过的秒数，并更新 *last_cycle
 *
 * @param last_cycle 调用者自己持有的"上次时间戳"变量地址
 * @return float 经过的秒数
 *
 * @note  ★ 推荐在所有 ISR 和周期任务里用这个接口：
 *        无全局状态、可重入，而且天然处理回绕。
 */
float Sys_Get_DeltaTime(uint32_t *last_cycle);

/**
 * @brief 自启动以来经过的微秒数（64 位累计）
 * @note  累加状态在临界区里更新，所以任务和 ISR 里都能安全调用。
 *        ⚠️ 但两次调用的间隔必须 < 25.57s（32 位 CYCCNT 的一个回绕周期），
 *        超了会丢掉整圈的周期数，累加值直接算错。
 *        → 保活由 1ms 任务显式完成：循环里每拍调一次 Sys_Get_Micros()
 *          （不依赖"有没有在收串口 / IMU 是否初始化成功"这些隐式路径）。
 *        只在需要"绝对时间"时用它（如日志时间戳）；
 *        纯测时间间隔用 Sys_Get_DeltaTime()，没有临界区，更便宜。
 */
uint64_t Sys_Get_Micros(void);

/** @brief 阻塞式微秒延时（忙等，基于 DWT） */
void Sys_Delay_US(float us);

/** @brief 阻塞式秒延时（忙等） */
void Sys_Delay_S(float s);

#ifdef __cplusplus
}
#endif

#endif
