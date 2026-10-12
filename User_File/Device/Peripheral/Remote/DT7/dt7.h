#ifndef DT7_H
#define DT7_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DT7_FRAME_LENGTH     18U
#define DT7_CHANNEL_CENTER   1024
#define DT7_CHANNEL_LIMIT    700
#define DT7_TIMEOUT_US       100000U

#define DT7_SWITCH_UP        1U
#define DT7_SWITCH_DOWN      2U
#define DT7_SWITCH_MIDDLE    3U

typedef struct
{
    // 四个摇杆通道：中心为 0，正常范围约 -660～660
    int16_t Channel[4];

    // 开关位置：1=上，2=下，3=中
    uint8_t Switch_Left;
    uint8_t Switch_Right;

    // 鼠标移动量和按键状态
    int16_t Mouse_X;
    int16_t Mouse_Y;
    int16_t Mouse_Z;
    uint8_t Mouse_Left_Button;
    uint8_t Mouse_Right_Button;

    // 键盘按键位图，完整保留 16 位
    uint16_t Keyboard;

    // 帧的最后两个字节，先保留原始值
    uint16_t Aux_Raw;

    // 最近一帧有效数据的接收时间、发布序号
    uint64_t Last_Rx_Time_Us;
    uint32_t Sequence;

    // 快照是否可用：未收到有效帧或超时为 0
    uint8_t Valid;
} Struct_DT7_Data;

// 绑定串口并启动接收，在 System_Init 中调用
void DT7_Init(UART_HandleTypeDef *huart);

// 整帧获取数据；返回 true 表示当前快照有效
bool DT7_Get_Snapshot(Struct_DT7_Data *out);

#ifdef __cplusplus
}
#endif

#endif // DT7_H