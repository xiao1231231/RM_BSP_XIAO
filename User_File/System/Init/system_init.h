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
 *        做的事：记录复位原因 → 时间戳/调试/灯/蜂鸣器/串口 →
 *        BMI088 + VQF 姿态初始化 → 打印启动信息 → 切到波形模式。
 *
 *        ⚠️ 里面有【阻塞】操作：BMI088 的上电稳定延时和开机零偏标定
 *           最长约 10 秒。所以它必须在调度器启动前调用，不能塞进任务里。
 */
void System_Init(void);

#ifdef __cplusplus
}
#endif

#endif
