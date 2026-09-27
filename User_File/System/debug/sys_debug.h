/**
 * @file    sys_debug.h
 * @brief   调试输出 —— ★ 串口上【只发三轴姿态波形】，不打印任何日志
 *
 * @note    ★ 本工程刻意只保留这一条输出：
 *            串口 1 上唯一的内容就是 50Hz 的 "imu:roll,pitch,yaw"，
 *            没有任何日志、任何时间戳前缀、任何别的东西掺进来。
 *            好处：VOFA+ 的解析不可能被干扰；也没有每帧浮点转文本的开销。
 *
 *          需要看内部状态（零偏估计、各种计数器、初始化结果）时：
 *            · 最省事：在 TIM_1ms_Task 的 Wave_Output 格式串里加一两个通道
 *              （驱动里那些诊断 getter 都还在，见 Device/Onboard/BMI088/）
 *            · 或者把带日志的版本恢复回来 —— 备份在 RM_F407_backup_20260927
 */

#ifndef SYS_DEBUG_H
#define SYS_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief VOFA+ 波形输出（FireWater 协议）
 * @param Fmt 形如 "imu:%.2f,%.2f,%.2f" ——
 *            ★ 冒号左边是【前缀文本】（显示在数据区），右边是逗号分隔的纯数值。
 *              不能写成 "roll:%.2f,pitch:%.2f" ——
 *              FireWater 只认第一个冒号，后面的 "pitch:" 不是数值会被丢弃，
 *              结果是只出来一个通道。
 * @note  函数内部自动补 "\r\n" 分帧。通道名在 VOFA+ 界面里配。
 * @note  受串口"忙时丢弃"规则约束 —— 调用频率别超过串口带宽。
 */
void Wave_Output(const char *Fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* SYS_DEBUG_H */
