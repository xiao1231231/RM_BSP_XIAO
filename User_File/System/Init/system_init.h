#ifndef SYSTEM_INIT_H_
#define SYSTEM_INIT_H_

#ifdef  __cplusplus
extern "C" {
#endif
/**
 * @brief 系统初始化统一入口
 *
 * @note  调用位置：Core/Src/main.c 的 USER CODE BEGIN 2 区
 *        （在所有 MX_XXX_Init() 之后、osKernelInitialize() 之前）。
 *
 *        做的事：时间戳 → 灯/蜂鸣器 → USART1/USART6 串口 →
 *        CAN + 4 路 3508 电机 → BMI088 + VQF 姿态初始化。
 *
 *        ⚠️ 里面有【阻塞】操作：开机零偏标定 1 秒 + 配置失败重试等待，
 *           最坏要几秒。所以必须在调度器启动前调用，不能塞进任务里。
 *           期间板子必须静止放好（零偏标定要求）。
 */
void System_Init(void);

#ifdef __cplusplus
}
#endif

#endif
