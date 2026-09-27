/**
 * @file    bsp_bmi088_accel_register.h
 * @brief   BMI088 加速度计寄存器映射
 *
 * @note    来源：H7_BSP（USTC-RoboWalker / yssickjgd）。
 *          和陀螺那份一样：结构体只用来算偏移（offsetof），不是内存映射访问。
 *          ★ Reserved_x[n] 的长度决定了后面每个寄存器的地址，别动。
 */

#ifndef BSP_BMI088_ACCEL_REGISTER_H
#define BSP_BMI088_ACCEL_REGISTER_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_spi.h"

/* Exported types ------------------------------------------------------------*/

/** 加速度计寄存器映射 */
struct Struct_BMI088_Accel_Register
{
    uint8_t ACC_CHIP_ID_RO;
    uint8_t Reserved_0[1];

    uint8_t ACC_ERR_REG_RO;

    uint8_t ACC_STATUS_RO;
    uint8_t Reserved_1[0x11 - 0x03];

    // 加速度数据
    int16_t ACC_X_RO;
    int16_t ACC_Y_RO;
    int16_t ACC_Z_RO;

    uint8_t SENSORTIME_0_RO;
    uint8_t SENSORTIME_1_RO;
    uint8_t SENSORTIME_2_RO;
    uint8_t Reserved_2[0x1c - 0x1a];

    uint8_t ACC_INT_STAT_1_RO;
    uint8_t Reserved_3[0x21 - 0x1d];

    // 温度数据
    uint8_t TEMP_MSB_RO;
    uint8_t TEMP_LSB_RO;

    uint8_t FIFO_LENGTH_0_RO;
    uint8_t FIFO_LENGTH_1_RO;
    uint8_t FIFO_DATA_RO;
    uint8_t Reserved_4[0x3f - 0x26];

    uint8_t ACC_CONF_RW;
    uint8_t ACC_RANGE_RW;
    uint8_t Reserved_5[0x44 - 0x41];

    uint8_t FIFO_DOWNS_RW;
    uint8_t FIFO_WTM_0_RW;
    uint8_t FIFO_WTM_1_RW;
    uint8_t FIFO_CONFIG_0_RW;
    uint8_t FIFO_CONFIG_1_RW;
    uint8_t Reserved_6[0x52 - 0x49];

    uint8_t INT1_IO_CTRL_RW;
    uint8_t INT2_IO_CTRL_RW;
    uint8_t Reserved_7[0x57 - 0x54];

    uint8_t INT_MAP_DATA_RW;
    uint8_t Reserved_8[0x6c - 0x58];

    uint8_t ACC_SELF_TEST_RW;
    uint8_t Reserved_9[0x7b - 0x6d];

    uint8_t ACC_PWR_CONF_RW;
    uint8_t ACC_PWR_CTRL_RW;
    uint8_t ACC_SOFTRESET_WO;
} __attribute__((packed));

#endif /* BSP_BMI088_ACCEL_REGISTER_H */

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
