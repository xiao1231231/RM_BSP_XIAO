/**
 * @file    bsp_flash.h
 * @brief   片上 Flash 数据区 —— 用最后一个扇区存掉电不丢的标定数据
 *
 * @note    C 板没有片外 Flash（W25Q64 那类），但 STM32F407 自己的 1MB Flash
 *          支持运行时擦写。标准做法是拿【最后一个扇区】（sector 11，
 *          0x080E0000，128KB）当数据区 —— 程序区（~135KB）离它十万八千里。
 *
 *          ⚠️ F407 只有一个 Flash bank：擦/写期间 CPU 取指被硬件阻塞，
 *          【整个系统冻结 1~2 秒】（所有中断都排队等着）。因此：
 *            · Save 只允许在【任务上下文、台架场景】调用，绝不能在
 *              控制运行中发生；
 *            · Load 就是普通内存读，任意上下文（含中断）都安全。
 */

#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 标定数据扇区：F407IGH6 的 sector 11（128KB，0x080E0000 ~ 0x080FFFFF） */
#define BSP_FLASH_CAL_SECTOR    FLASH_SECTOR_11
#define BSP_FLASH_CAL_ADDR      0x080E0000U

/**
 * @brief 保存零偏标定到片上 Flash（整扇区擦除 → 按 word 写入 → 写后读校验）
 * @param Gyro_Bias    三轴陀螺零偏（rad/s，语义 = "要从原始值里减掉的量"，即均值原值）
 * @param Temperature  标定时的 IMU 温度（°C），仅作记录
 * @return true = 写入并读回校验一致
 * @note  ⚠️ 全机冻结 1~2 秒（见文件头）；擦写寿命 ~1 万次，标定频率下无压力
 */
bool BSP_Flash_Calibration_Save(const float Gyro_Bias[3], float Temperature);

/**
 * @brief 读取零偏标定（魔数 + CRC32 双重校验，任一不过按"无标定"处理）
 * @return true = 读到了有效标定
 * @note  普通内存读，任意上下文（含中断、调度器启动前）都安全
 */
bool BSP_Flash_Calibration_Load(float Gyro_Bias_out[3], float *Temperature_out);

#ifdef __cplusplus
}
#endif

#endif /* BSP_FLASH_H */
