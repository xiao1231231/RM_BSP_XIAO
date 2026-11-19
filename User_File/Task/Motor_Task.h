#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

// 控制任务发布的软件空闲状态，不证明机构物理静止。
// 标定任务读取此状态，不跨任务读取各电机的目标。
bool Motor_Control_Is_Idle();

// 以下握手仅由标定任务调用：每次操作请求一次，后续周期轮询 Ready。
// 请求由 Motor_Task 下一周期处理；等待超时由调用方处理。
// 成功、失败或取消退出时均须 Release，释放后恢复正常控制流程。
// 前提：运行期电机 CAN 帧统一由 Motor_Task 发送；新增发送者须重新检查批次确认。
void Motor_Maintenance_Request();

// Ready 表示停机批次获得 TXOK，并通过现有 DM 失能/反馈检查。
// 不证明每台执行器已执行命令或机构已停止运动。
bool Motor_Maintenance_Is_Ready();

void Motor_Maintenance_Release();

#endif /* MOTOR_TASK_H */
