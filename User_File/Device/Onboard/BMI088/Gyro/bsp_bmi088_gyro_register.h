/**
 * @file    bsp_bmi088_gyro_register.h
 * @brief   BMI088 陀螺仪寄存器映射
 *
 * @note    来源：H7_BSP（USTC-RoboWalker / yssickjgd）。
 *
 *          这是"用结构体描述寄存器地址空间"的写法：只用来算偏移
 *          （offsetof(...)），不是真的把它当内存映射去访问 —— 真正的读写
 *          都走 SPI，一字节一字节发。
 *          ★ 因此 Reserved_x[n] 里那些数组长度不是随便写的，
 *            它们把每个寄存器顶到数据手册上的正确地址，改一个就全错。
 */

#ifndef BSP_BMI088_GYRO_REGISTER_H
#define BSP_BMI088_GYRO_REGISTER_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_spi.h"

/* Exported types ------------------------------------------------------------*/

/** 陀螺仪寄存器映射（地址 → 字段，靠 offsetof 取地址） */
struct Struct_BMI088_Gyro_Register
{
    uint8_t GYRO_CHIP_ID_RO;
    uint8_t Reserved_0[1];

    // 陀螺仪数据
    int16_t RATE_X_RO;
    int16_t RATE_Y_RO;
    int16_t RATE_Z_RO;

    uint8_t Reserved_1[0x09 - 0x07];

    uint8_t GYRO_INT_STAT_1_RO;
    uint8_t Reserved_2[0x0d - 0x0a];

    uint8_t FIFO_STATUS_RO;

    uint8_t GYRO_RANGE_RW;
    uint8_t GYRO_BANDWODTH_RW;
    uint8_t GYRO_LPM1_RW;
    uint8_t Reserved_3[0x13 - 0x11];

    uint8_t GYRO_SOFTRESET_WO;

    uint8_t GYRO_INT_CTRL_RW;
    uint8_t INT3_INT4_IO_CONF_RW;
    uint8_t Reserved_4[0x17 - 0x16];

    uint8_t INT3_INT4_IO_MAP_RW;
    uint8_t Reserved_5[0x1d - 0x18];

    uint8_t FIFO_WM_EN_RW;
    uint8_t Reserved_6[0x33 - 0x1e];

    uint8_t FIFO_EXT_INT_S_RW;
    uint8_t Reserved_7[0x3b - 0x34];

    uint8_t GURO_SELF_TEST_RWX;

    uint8_t FIFO_CONFIG_0_RW;
    uint8_t FIFO_CONFIG_1_RW;

    uint8_t FIFO_DATA_RO;
} __attribute__((packed));

#endif /* BSP_BMI088_GYRO_REGISTER_H */

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
