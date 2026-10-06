/**
 * @file    bsp_flash.cpp
 * @brief   片上 Flash 数据区实现（见 .h 顶部说明）
 *
 * @note    存储帧布局（28 字节，全 32 位对齐，按 word 编程）：
 *            Magic(4) + Version(4) + Gyro_Bias[3](12) + Temperature(4) + Crc32(4)
 *          CRC32 覆盖前 24 字节；float 按【位】原样存取（memcpy，不做数值转换）。
 */

#include "bsp_flash.h"

#include "main.h"       /* HAL Flash API */

#include <string.h>
#include <stddef.h>

/* Private defines -----------------------------------------------------------*/

/** 帧头识别码，ASCII "CAL1"（小端）。对不上 = 空片 / 从没存过 */
static const uint32_t CAL_MAGIC = 0x314C4143U;
static const uint32_t CAL_VERSION = 1U;

/* Private types -------------------------------------------------------------*/

struct Struct_Flash_Cal_Frame
{
    uint32_t Magic;
    uint32_t Version;
    uint32_t Gyro_Bias[3];
    uint32_t Temperature;       /* float 按位存 */
    uint32_t Crc32;             /* 覆盖本字段之前的 24 字节 */
};

/* Private functions ---------------------------------------------------------*/

/** CRC-32（反射多项式 0xEDB88320，无查表版 —— 28 字节不值得占 256 项表） */
static uint32_t Calc_Crc32(const uint8_t *Data, uint32_t Length)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0U; i < Length; i++)
    {
        crc ^= Data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            crc = ((crc & 1U) != 0U) ? ((crc >> 1) ^ 0xEDB88320U) : (crc >> 1);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

/* Exported functions --------------------------------------------------------*/

bool BSP_Flash_Calibration_Save(const float Gyro_Bias[3], float Temperature)
{
    Struct_Flash_Cal_Frame frame = {};
    frame.Magic     = CAL_MAGIC;
    frame.Version   = CAL_VERSION;
    memcpy(frame.Gyro_Bias, Gyro_Bias, sizeof(frame.Gyro_Bias));
    memcpy(&frame.Temperature, &Temperature, sizeof(frame.Temperature));
    frame.Crc32 = Calc_Crc32((const uint8_t *)&frame,
                             offsetof(Struct_Flash_Cal_Frame, Crc32));

    bool ok = (HAL_FLASH_Unlock() == HAL_OK);
    if (ok)
    {
        /* ⚠️ 从这里开始全机冻结 ~1-2 秒（128KB 扇区擦除 + 7 个 word 编程） */
        FLASH_EraseInitTypeDef erase = {};
        erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
        erase.Sector       = BSP_FLASH_CAL_SECTOR;
        erase.NbSectors    = 1U;
        erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;     /* 3.3V 供电 */
        uint32_t sector_error = 0U;
        ok = (HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK);
    }

    if (ok)
    {
        const uint32_t *words = (const uint32_t *)&frame;
        for (uint32_t i = 0U; i < sizeof(frame) / 4U; i++)
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                  BSP_FLASH_CAL_ADDR + i * 4U,
                                  words[i]) != HAL_OK)
            {
                ok = false;
                break;
            }
        }
    }
    (void)HAL_FLASH_Lock();

    /* 写后读校验：不信任 API 返回值，信内存里的实际内容 */
    if (ok)
    {
        Struct_Flash_Cal_Frame verify = {};
        memcpy(&verify, (const void *)BSP_FLASH_CAL_ADDR, sizeof(verify));
        ok = (memcmp(&verify, &frame, sizeof(frame)) == 0);
    }
    return ok;
}

bool BSP_Flash_Calibration_Load(float Gyro_Bias_out[3], float *Temperature_out)
{
    Struct_Flash_Cal_Frame frame;
    memcpy(&frame, (const void *)BSP_FLASH_CAL_ADDR, sizeof(frame));

    if (frame.Magic != CAL_MAGIC)
    {
        return false;       /* 空片 / 从没存过 */
    }

    const uint32_t crc = Calc_Crc32((const uint8_t *)&frame,
                                    offsetof(Struct_Flash_Cal_Frame, Crc32));
    if (crc != frame.Crc32)
    {
        return false;       /* 内容损坏（如擦写中途断电），按无标定处理 */
    }

    memcpy(Gyro_Bias_out, frame.Gyro_Bias, sizeof(frame.Gyro_Bias));
    memcpy(Temperature_out, &frame.Temperature, sizeof(frame.Temperature));

    /* 量纲合理性：|零偏| < 0.1 rad/s（≈5.7°/s）—— CRC 只保证"没坏"，
     * 这一步保证"合理"，坏数据（如写入时板子根本没静止）按无标定处理 */
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (Gyro_Bias_out[i] > 0.1f || Gyro_Bias_out[i] < -0.1f)
        {
            return false;
        }
    }
    return true;
}
