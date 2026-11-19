# DJI 电机自检与 GM6020 上板检查

主机自检直接编译工程中的 `dji_motor.cpp`、PID 和数学代码，仅替换 CAN 收发、时间读取与 CMSIS 中断接口。它不验证硬件接线或真实中断时序，也不接入 STM32 固件构建。

在工程根目录运行：

```powershell
python tests/run_dji_motor_selftest.py --cxx D:\mingw64\bin\g++.exe
```

覆盖反馈正负值、转速换算、角度过零、非法帧、型号/编号校验、零目标速度闭环、创建失败、四组报文、限幅、超时清 PID、5 ms 超时及恢复。生成的程序放在已忽略的 `build/host_tests`。

## SMC 与云台控制自检

```powershell
python tests/run_gimbal_selftest.py --cxx D:\mingw64\bin\g++.exe
```

直接编译真实 SMC、云台、电机驱动、滤波和数学代码，仅替换 CAN、时间与 CMSIS 边界。
覆盖 SMC 解析值、正负制动、幂次/线性段拼接、限幅、无效参数、目标归一化、
角度跨零最短路径、积分累积、5 ms 超时边界与恢复、无效目标和控制掉拍。
可加 `--trace build/host_tests/gimbal_trace.tsv` 保存每拍时间、角度、积分、总输出和驱动电流，
用于重构前后的逐拍对比；浮点值使用十六进制格式保存，避免小数打印精度掩盖差异。

控制代码分工：

- `User_File/Application/Gimbal/gimbal_config.h`：集中设置电机 ID、方向、启动目标、滤波、积分及 SMC 参数。
- `alg_smc.h/.cpp`：只做 SMC 计算；调用 `Init(config)` 后，使用 `Calculate(error, error_rate)` 的返回值。没有上次输出缓存或积分状态。
- `gimbal.h/.cpp`：处理反馈、单位和方向换算、速度滤波、积分抗饱和、最终限幅及电流目标；运行状态收在文件内部。
- `Motor_Task` 调用 `Gimbal_Init()`、`Gimbal_Update()`；修改目标用 `Gimbal_Set_Target_Angle_Deg()`，同样由控制任务调用。
- USB 等其他任务调用 `Gimbal_Get_Debug_Snapshot()`；仅在返回 `true` 时使用数据。

## 当前上板默认值

- CAN1 创建四个 3508（ID 1～4）和两个 GM6020（拨码 ID 5、6）。主机自检中的 ID 是协议测试样例，与此上板配置独立。
- GM6020 的速度 PID 在 `System_Init()` 中：ID 5 为 15/0/0，ID 6 为 0/0/0。被云台接管的电机使用直接电流控制，不经过速度 PID。
- `Gimbal_Config::Motor_Id` 当前为 5，方向为 +1，启动目标为 0°。单圈绝对角度由 `Ecd` 换算，0° 对应编码器零位，主控复位不会改变这个对应关系；它尚不是安装后的机械零点。
- 当前 SMC：`C=4`、`K=8`、`Epsilon=0`、`Phi=0.2`、`J=12.5`、`P=21`、`Q=27`、线性段 ±1°；SMC 与最终总输出共用限幅 ±2000。
- 积分增益为 50、积分限幅 ±500；速度滤波截止频率为 0（旁路）。首拍、反馈超时或控制掉拍超过 5 ms 时重新计时、清积分并对齐滤波。
- `Motor_Task()` 中 `gm6020_target_rpm[0/1]` 对应 GM6020 ID 5/6，初始均为零；仅给未被 Gimbal 接管的电机设置速度目标。
- USB 当前通过 `Attitude_Get_Snapshot()` 输出同一帧的六项 IMU 数据：`Roll,Pitch,Yaw,GyroX,GyroY,GyroZ`。前三项为 VQF 解算的姿态角（度），后三项为安装机体系角速度（未扣零偏，度/秒）；当前 `Face_Down=false`，正常摆放，三轴与传感器轴一致。保留三位小数，格式为 `data:...`，每 5 ms 尝试发送一次，姿态无效或样本过期时不输出。
- `Get_Out_Current()` 和反馈 `Torque_Current` 都是原始数值。GM6020 控制指令 ±16384 对应 ±3 A；此控制比例不能直接当成反馈电流换算比例。

## 由用户执行的实机验证

1. 确认 GM6020 固件 >= 1.0.11.2，在 RoboMaster Assistant >= 2.7 打开电流环开关，并核对 CAN1 接线和拨码 ID。
2. 调用 `Get_Feedback_Snapshot()` 观察 ID 5、6：`Received` 应为真，`Sequence` 持续增加，角度与 rpm 更新。ID 5/6 的反馈地址应为 `0x209/0x20A`。
3. 核对 `0x2FE` 控制帧：ID 5 用字节 0/1，ID 6 用字节 2/3，后四字节为零。被云台接管的电机电流随误差和转速变化，另一台当前保持零输出；长时间断点暂停会影响 1 ms 调度。
4. 核对绝对角度、正方向和目标位置。跨过 8191/0 时显示角度在 360°/0° 之间回绕，控制误差应选择短路径；主控复位后，同一物理位置仍应对应相同读数。
5. 验证目标角度保持、正反转、反馈超过 5 ms 未更新时零输出，以及反馈恢复后积分重新从零开始。修改被接管电机的速度 PID 或 `gm6020_target_rpm` 不会改变其直接电流控制律。
6. 验证原有 3508 控制，并观察 `motor_task_period_s`、`motor_task_overrun_count`、`can_tx_drop_count` 和 `can_bus_off_count`。所有六台都接入时需复核 1 Mbps 总线实际负载。

本次自动验证未烧录、驱动电机或完成上述实机检查。

## BMI088 正常摆放坐标自检

```powershell
python tests/run_imu_mounting_selftest.py --cxx D:\mingw64\bin\g++.exe
```

直接编译生产使用的安装坐标变换和 VQF，检查正常摆放重力下带小水平扰动的多组启动姿态、滤波器复位、传感器/安装坐标零偏往返及正反向偏航。此主机自检不模拟真实 BMI088 噪声或板上安装方向；需上板确认 USB 的角速度轴向和云台控制方向。

## DT7 底盘控制主机自检

```powershell
python tests/run_chassis_selftest.py --cxx D:\mingw64\bin\g++.exe
```

直接编译真实 `chassis.cpp` 和 `dt7.cpp`，仅替换 UART 接收入口、时间和 CMSIS 边界。
覆盖遥控未连接、前后/横移/旋转映射、半杆、中心死区、输入限幅、轮位和方向映射、
拨杆停车、100 ms 超时边界及恢复、组合运动的四轮转速上限。
主机自检不能确认实物摇杆通道、拨杆名称、电机方向、UART/DMA 或真实调度时序。
