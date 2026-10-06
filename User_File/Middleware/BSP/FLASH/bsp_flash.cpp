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

#include <math.h>       /* isfinite —— NaN/Inf 检查 */
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
    if (Gyro_Bias == nullptr)
    {
        return false;
    }

    /* 擦除前先验证：旧数据被擦掉、新数据却是垃圾 —— 那是最坏的结果。
     * 零偏范围与读取端同一判据（±0.1 rad/s ≈ ±5.7°/s）；
     * 温度只要求有限（异常时记 0，不让它拦下一次合格的标定） */
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (!isfinite(Gyro_Bias[i]) ||
            Gyro_Bias[i] > 0.1f || Gyro_Bias[i] < -0.1f)
        {
            return false;
        }
    }
    if (!isfinite(Temperature))
    {
        Temperature = 0.0f;
    }

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
    if (Gyro_Bias_out == nullptr || Temperature_out == nullptr)
    {
        return false;
    }

    /* 帧布局是 Flash 里的物理约定，改了必须同步改这段与 Save —— 编译期拦住 */
    static_assert(sizeof(Struct_Flash_Cal_Frame) == 28U, "Flash 标定帧布局变了");

    Struct_Flash_Cal_Frame frame;
    memcpy(&frame, (const void *)BSP_FLASH_CAL_ADDR, sizeof(frame));

    /* 三重身份检查：魔数（是不是我们的数据）、版本（格式对不对）、
     * CRC（内容有没有坏）—— 任一不过都按"无标定"处理 */
    if (frame.Magic != CAL_MAGIC ||
        frame.Version != CAL_VERSION ||
        frame.Crc32 != Calc_Crc32((const uint8_t *)&frame,
                                  offsetof(Struct_Flash_Cal_Frame, Crc32)))
    {
        return false;
    }

    /* 先在局部变量里全部验证通过，最后才写调用方输出 ——
     * 失败时输出保持原样，调用方不会被"改了一半"的值误导 */
    float bias[3];
    float temperature;
    memcpy(bias, frame.Gyro_Bias, sizeof(bias));
    memcpy(&temperature, &frame.Temperature, sizeof(temperature));

    if (!isfinite(temperature))
    {
        return false;
    }
    for (uint8_t i = 0U; i < 3U; i++)
    {
        /* ★ 必须先查 isfinite：NaN 与任何常数比较都是 false，
         *   光靠范围判断会让 NaN 溜过去 */
        if (!isfinite(bias[i]) || bias[i] > 0.1f || bias[i] < -0.1f)
        {
            return false;
        }
    }

    memcpy(Gyro_Bias_out, bias, sizeof(bias));
    *Temperature_out = temperature;
    return true;
}
