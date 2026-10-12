#include "dt7.h"
#include "bsp_uart.h"
#include "sys_timestamp.h"

// 保存最近一帧有效数据，仅在本文件内使用
static Struct_DT7_Data s_dt7_data = {};

static bool DT7_Decode_Frame(const uint8_t *buffer,
                             uint16_t length,
                             Struct_DT7_Data *out)
{
    if (buffer == nullptr || out == nullptr ||
        length != DT7_FRAME_LENGTH)
    {
        return false;
    }

    Struct_DT7_Data data = {};

    // 四个摇杆通道，每个占 11 位
    data.Channel[0] = (buffer[0] | (buffer[1] << 8)) & 0x07FFU;
    data.Channel[1] = ((buffer[1] >> 3) | (buffer[2] << 5)) & 0x07FFU;
    data.Channel[2] = ((buffer[2] >> 6) | (buffer[3] << 2) |
                       (buffer[4] << 10)) & 0x07FFU;
    data.Channel[3] = ((buffer[4] >> 1) | (buffer[5] << 7)) & 0x07FFU;

    for (uint8_t i = 0; i < 4U; ++i)
    {
        // 将摇杆中心值从 1024 转换为 0
        data.Channel[i] -= DT7_CHANNEL_CENTER;

        if (data.Channel[i] < -DT7_CHANNEL_LIMIT ||
            data.Channel[i] > DT7_CHANNEL_LIMIT)
        {
            return false;
        }
    }

    // 左开关在 byte[5] 的高两位，右开关在 bit[5:4]
    data.Switch_Left  = (buffer[5] >> 6) & 0x03U;
    data.Switch_Right = (buffer[5] >> 4) & 0x03U;

    // 两位只能表示 0～3，其中 0 是非法开关位置
    if (data.Switch_Left == 0U || data.Switch_Right == 0U)
    {
        return false;
    }

    data.Mouse_X = static_cast<int16_t>(buffer[6] | (buffer[7] << 8));
    data.Mouse_Y = static_cast<int16_t>(buffer[8] | (buffer[9] << 8));
    data.Mouse_Z = static_cast<int16_t>(buffer[10] | (buffer[11] << 8));

    data.Mouse_Left_Button  = buffer[12];
    data.Mouse_Right_Button = buffer[13];

    if (data.Mouse_Left_Button > 1U || data.Mouse_Right_Button > 1U)
    {
        return false;
    }

    data.Keyboard = static_cast<uint16_t>(buffer[14] | (buffer[15] << 8));
    data.Aux_Raw  = static_cast<uint16_t>(buffer[16] | (buffer[17] << 8));

    // 校验全部通过后，才提交解析结果
    *out = data;
    return true;
}

static void DT7_Rx_Callback(uint8_t *buffer, uint16_t length)
{
    Struct_DT7_Data data = {};

    if (!DT7_Decode_Frame(buffer, length, &data))
    {
        return;
    }

    data.Last_Rx_Time_Us = Sys_Get_Micros();
    data.Valid = 1U;

    // 整帧发布，避免读取到一半新数据、一半旧数据
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    data.Sequence = s_dt7_data.Sequence + 1U;
    s_dt7_data = data;

    __DMB();
    __set_PRIMASK(primask);
}

void DT7_Init(UART_HandleTypeDef *huart)
{
    if (huart == nullptr)
    {
        return;
    }

    // 只在调度器启动前初始化一次
    s_dt7_data = {};
    UART_Init(huart, DT7_Rx_Callback);
}

bool DT7_Get_Snapshot(Struct_DT7_Data *out)
{
    if (out == nullptr)
    {
        return false;
    }

    // 整帧复制，保证各字段来自同一帧
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    *out = s_dt7_data;

    __DMB();
    __set_PRIMASK(primask);

    // 没收到有效帧，或已有 100ms 没更新，快照就不可用
    if (out->Valid == 0U ||
        (Sys_Get_Micros() - out->Last_Rx_Time_Us) >= DT7_TIMEOUT_US)
    {
        out->Valid = 0U;
        return false;
    }

    return true;
}