#ifndef CHASSIS_H
#define CHASSIS_H

// 轮位对应的 3508 ID：暂定，后续按实际安装修改。
// 四个 ID 必须是 1～4，且互不重复。
#define CHASSIS_FL_ID  1U   // 左前
#define CHASSIS_FR_ID  2U   // 右前
#define CHASSIS_RL_ID  3U   // 左后
#define CHASSIS_RR_ID  4U   // 右后

// 电机方向：正转能让对应轮子推动车辆向前时填 +1，否则填 -1。
// 当前为占位值，后续逐轮确认。
#define CHASSIS_FL_DIRECTION  (+1.0f)
#define CHASSIS_FR_DIRECTION  (+1.0f)
#define CHASSIS_RL_DIRECTION  (+1.0f)
#define CHASSIS_RR_DIRECTION  (+1.0f)

// 机械尺寸，单位 mm；0 表示尚未填写。
#define CHASSIS_WHEEL_DIAMETER_MM  0.0f  // 轮子直径
#define CHASSIS_WHEEL_BASE_MM      0.0f  // 前后轮心距离
#define CHASSIS_WHEEL_TRACK_MM     0.0f  // 左右轮心距离

// 调试起步值，后续根据实际底盘调整。
#define CHASSIS_MAX_VX_MPS        0.3f    // 最大前后速度，m/s
#define CHASSIS_MAX_VY_MPS        0.3f    // 最大横移速度，m/s
#define CHASSIS_MAX_WZ_RADPS      0.5f    // 最大转向速度，rad/s
#define CHASSIS_MAX_WHEEL_RPM     100.0f  // 轮轴转速上限

// 麦轮逆运动学：vx 向前、vy 向左、wz 逆时针为正。
// 输入单位：m/s、m/s、rad/s。
// 输出单位：输出轴 rpm；数组顺序：左前、右前、左后、右后。
void Chassis_Mecanum_Inverse(float vx, float vy, float wz,
                            float wheel_rpm[4]);

// 更新底盘控制，输出数组下标 0～3 对应 3508 ID 1～4。
void Chassis_Update(float target_rpm[4]);

#endif // CHASSIS_H