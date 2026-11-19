#ifndef GIMBAL_REMOTE_H
#define GIMBAL_REMOTE_H

#include "dt7.h"

// 左开关中档时把左摇杆水平通道映射为偏航目标角速度，单位 rad/s。
// 数据无效或不在中档时返回零。
float Gimbal_Remote_Map_Yaw_Rate_Radps(const Struct_DT7_Data &remote);

#endif // GIMBAL_REMOTE_H
