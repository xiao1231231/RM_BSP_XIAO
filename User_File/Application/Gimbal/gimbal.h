#ifndef GIMBAL_H
#define GIMBAL_H

// 初始化、目标设置和 Update 统一由 Motor_Task 调用；Update 每 1 ms 执行一次。
// 新目标来源在该任务中选择，避免被每周期的遥控目标写入覆盖。
void Gimbal_Init(void);
void Gimbal_Update(void);

// 单圈绝对目标，单位度；有限数值折回 [0, 360)，控制时选择最短路径。
// 每次调用均先清零目标角速度。
// NaN/Inf 会使后续 Update 保持零电流，直到再次设置有效目标。
void Gimbal_Set_Target_Angle_Deg(float angle_deg);

// 设置目标偏航角速度，单位 rad/s；在 Gimbal_Update 中积分成角度目标。
// 零值或 NaN/Inf 按零角速度处理，保持已有角度目标，不返回启动目标。
// 同时设置两者时先设角度、再设角速度；后续 Update 会继续积分推进角度目标。
void Gimbal_Set_Target_Angular_Velocity_Radps(float rate_rad_s);

// 同一控制周期的数据；电流均使用云台正方向，单位为 CAN 指令，不是 A。
struct Struct_Gimbal_Debug
{
    float Angle_Deg;         // 单圈绝对角度 [0, 360)，不随主控复位归零。
    float Integral_Current;  // 积分补偿。
    float Control_Current;   // SMC 加积分并限幅后的总输出。
};

// 可跨任务读取；返回 true 才能使用 data，未完成计算或反馈超时返回 false。
bool Gimbal_Get_Debug_Snapshot(Struct_Gimbal_Debug *data);

#endif // GIMBAL_H
