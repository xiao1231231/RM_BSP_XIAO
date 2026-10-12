# DJI 电机自检与 GM6020 上板检查

主机自检直接编译工程中的 `dji_motor.cpp`、PID 和数学代码，仅替换 CAN 收发、时间读取与 CMSIS 中断接口。它不验证硬件接线或真实中断时序，也不接入 STM32 固件构建。

在工程根目录运行：

```powershell
python tests/run_dji_motor_selftest.py --cxx D:\mingw64\bin\g++.exe
```

覆盖反馈正负值、转速换算、角度过零、非法帧、型号/编号校验、零目标速度闭环、创建失败、四组报文、限幅、超时清 PID、5 ms 超时及恢复。生成的程序放在已忽略的 `build/host_tests`。

## 当前上板默认值

- CAN1 创建四个 3508（ID 1～4）和两个 GM6020（拨码 ID 1、2）。
- GM6020 的 PID 在 `System_Init()` 中，首次 Kp/Ki/Kd 都是零。
- `Motor_Task()` 中 `gm6020_target_rpm[0/1]` 对应 GM6020 ID 1/2，初始均为零。
- 电机通过目标转速控制。零目标仍运行速度环，非零 PID 增益下可能输出制动电流；当前 GM6020 增益全零，因此先只观察反馈。
- `Get_Out_Current()` 和反馈 `Torque_Current` 都是原始数值。GM6020 控制指令 ±16384 对应 ±3 A；此控制比例不能直接当成反馈电流换算比例。

## 由用户执行的实机验证

1. 确认 GM6020 固件 >= 1.0.11.2，在 RoboMaster Assistant >= 2.7 打开电流环开关，并核对 CAN1 接线和拨码 ID。
2. 保持两台 PID 增益及目标转速为零，调用 `Get_Feedback_Snapshot()` 观察 ID 1：`Received` 应为真，`Sequence` 持续增加，手动转动时角度与 rpm 更新。ID 1/2 的反馈地址应为 `0x205/0x206`。
3. 在 `CAN_Transmit()` 断点查看 `0x1FE` 帧：ID 1 用字节 0/1，ID 2 用字节 2/3，其余四字节为零。初始八字节应全零；长时间断点暂停会影响 1 ms 调度，恢复后的掉拍计数增加属于预期。
4. 速度验证时只调整 ID 1 的 PID，给 `gm6020_target_rpm[0]` 小幅正/负目标并调参；ID 2 的增益与目标均保持零。仅改目标、但 PID 仍全零时不会产生电流输出。
5. 将 ID 1 目标设为零，验证速度环将转速控制到零；暂停其反馈，验证超过 5 ms 后零输出。恢复反馈应重新运行速度环，PID 历史已清零。
6. 验证原有 3508 控制，并观察 `motor_task_period_s`、`motor_task_overrun_count`、`can_tx_drop_count` 和 `can_bus_off_count`。所有六台都接入时需复核 1 Mbps 总线实际负载。

本次自动验证未烧录、驱动电机或完成上述实机检查。
