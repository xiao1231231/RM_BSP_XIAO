/**
 * @file    dji_motor.cpp
 * @brief   DJI 3508/C620 与 GM6020 电流模式实现
 *
 * @note    分工：CAN 收到反馈只做【解析】（快进快出）；
 *          控制在 1kHz 任务里统一跑 —— PID 的 D_T 必须和调用周期一致。
 */

#include "dji_motor.h"

#include "sys_timestamp.h"      /* Sys_Get_Micros() —— 反馈新鲜度判定 */

static CAN_HandleTypeDef *motor_can_handle = nullptr;
static constexpr uint8_t MOTOR_TYPE_COUNT = 2U;
static Class_DJI_Motor *motors[MOTOR_TYPE_COUNT][DJI_MOTOR_CNT] = {}; /* [型号][编号-1] */

static bool Motor_Selection_Valid(Enum_DJI_Motor_Num num, Enum_DJI_Motor_Type type)
{
    if (type != DJI_MOTOR_TYPE_M3508 && type != DJI_MOTOR_TYPE_GM6020) { return false; }
    const uint8_t max_id = (type == DJI_MOTOR_TYPE_GM6020) ? 7U : DJI_MOTOR_CNT;
    return num >= DJI_MOTOR_1 && num <= max_id;
}

/* bsp_can 的回调是 C 函数指针，不能指向成员函数 —— 用它带回的 Id 还原对象 */
static void Motor_Feedback_Callback(void *Id, uint8_t *Data, uint16_t Length)
{
    static_cast<Class_DJI_Motor *>(Id)->Feedback_Parse(Data, Length);
}

void DJI_Motor_Init(CAN_HandleTypeDef *hcan)
{
    motor_can_handle = hcan;
}

Class_DJI_Motor *DJI_Motor_Create(Enum_DJI_Motor_Num Motor_Num,
                                  float K_P, float K_I, float K_D,
                                  Enum_DJI_Motor_Type Motor_Type)
{
    if (motor_can_handle == nullptr || !Motor_Selection_Valid(Motor_Num, Motor_Type))
    {
        return nullptr;
    }
    const uint8_t motor_id = static_cast<uint8_t>(Motor_Num);
    const uint8_t type_index = static_cast<uint8_t>(Motor_Type);
    if (motors[type_index][motor_id - 1] != nullptr) { return nullptr; }

    /* 同一个硬件编号在两种型号中分别持有对象；不 malloc。 */
    static Class_DJI_Motor pool[MOTOR_TYPE_COUNT][DJI_MOTOR_CNT];
    Class_DJI_Motor *motor = &pool[type_index][motor_id - 1];
    motor->Init(motor_can_handle, motor_id, K_P, K_I, K_D, Motor_Type);

    /* 对象完成初始化后注册接收；成功才发布到对象表。
     * 注册失败不会留下可取出、可发送的半初始化对象。 */
    const uint32_t rx_base = (Motor_Type == DJI_MOTOR_TYPE_GM6020) ? 0x204U : 0x200U;
    if (!CAN_Register_Device(motor_can_handle, rx_base + motor_id, motor,
                             Motor_Feedback_Callback))
    {
        return nullptr;
    }
    motors[type_index][motor_id - 1] = motor;
    return motor;
}

Class_DJI_Motor *DJI_Motor_Get(Enum_DJI_Motor_Num Motor_Num, Enum_DJI_Motor_Type Motor_Type)
{
    if (!Motor_Selection_Valid(Motor_Num, Motor_Type)) { return nullptr; }
    return motors[static_cast<uint8_t>(Motor_Type)][static_cast<uint8_t>(Motor_Num) - 1];
}

bool DJI_Motor_Any_Target_Active(void)
{
    for (uint8_t type = 0U; type < MOTOR_TYPE_COUNT; type++)
    {
        for (uint8_t i = 0U; i < DJI_MOTOR_CNT; i++)
        {
            const Class_DJI_Motor *motor = motors[type][i];
            if (motor != nullptr && motor->Get_Target_Speed_Rpm() != 0.0f)
            {
                return true;
            }
        }
    }
    return false;
}

void Class_DJI_Motor::Init(CAN_HandleTypeDef *hcan, uint8_t Motor_Id,
                           float K_P, float K_I, float K_D, Enum_DJI_Motor_Type Motor_Type)
{
    CAN_Handler = hcan;
    this->Motor_Id = Motor_Id;
    this->Motor_Type = Motor_Type;
    Gear_Ratio = (Motor_Type == DJI_MOTOR_TYPE_GM6020) ? 1.0f : DJI_GEAR_RATIO;
    Feedback = {};
    Last_Ecd = 0U;
    Target_Speed_Rpm = 0.0f;
    Out_Current = 0;

    /* 速度环 1kHz：输出原始电流指令，两个型号均限幅 ±16384。 */
    Speed_PID.Init(K_P, K_I, K_D, 0.0f, DJI_OUT_CURRENT_MAX, DJI_OUT_CURRENT_MAX, 0.001f);
    Speed_PID.Reset();
}

void Class_DJI_Motor::Control()
{
    /* ── 反馈可用性检查：拿不到新鲜反馈就不闭环 ──
     * 两种坏状态在这里统一拦截：
     *   · 从没收到反馈（电调没上电 / ID 不对 / 线没接）→ 测量值全零
     *   · 反馈停了（掉线 / 接触不良）→ 旧值冻结在最后一帧
     * 两者都输出零电流（电机不带力，现象肉眼可见），同时清 PID 历史 ——
     * 恢复后从零起步，不带陈旧积分。整车级联动停机是应用层的事。
     * ★ 没有这段的话：速度环会拿冻结的转速当真，电流一路顶到限幅。 */
    Struct_DJI_Motor_Feedback feedback;
    const bool received = Get_Feedback_Snapshot(feedback);
    if (!received ||
        (Sys_Get_Micros() - feedback.Last_Rx_Time_Us) > DJI_FEEDBACK_TIMEOUT_US ||
        Basic_Math_Is_Invalid_Float(Target_Speed_Rpm))
    {
        Out_Current = 0;
        Speed_PID.Reset();      /* 每拍清，幂等；恢复瞬间无历史负担 */
        return;
    }

    Speed_PID.Set_Target(Target_Speed_Rpm);
    Speed_PID.Set_Now(feedback.Measure.Speed_Rpm);
    Speed_PID.TIM_Calculate_PeriodElapsedCallback();
    const float current = Speed_PID.Get_Out();
    if (Basic_Math_Is_Invalid_Float(current))
    {
        Out_Current = 0;
        Speed_PID.Reset();
        return;
    }
    Out_Current = static_cast<int16_t>(current);
}

void Class_DJI_Motor::Feedback_Parse(uint8_t *Data, uint16_t Length)
{
    if (Data == nullptr || Length != 8U) { return; }
    const uint16_t ecd = static_cast<uint16_t>(Data[0] << 8 | Data[1]);
    if (ecd > 8191U) { return; }

    /* 以已提交的帧为底，在栈上把这一帧凑齐再整体提交 ——
     * 读者拿到的 Measure 各字段保证来自同一次 CAN 反馈 */
    Struct_DJI_Motor_Feedback frame = Feedback;
    const int16_t rotor_rpm = (int16_t)(Data[2] << 8 | Data[3]);
    frame.Measure.Ecd            = ecd;
    frame.Measure.Torque_Current = (int16_t)(Data[4] << 8 | Data[5]);
    frame.Measure.Temperature    = Data[6];
    frame.Measure.Speed_Rpm      = (float)rotor_rpm / Gear_Ratio;

    /* 多圈累计：8192 过零判向（350→10 是正转过零，10→350 是反转过零）。
     * 首帧只记基准，不算圈数增量 */
    if (frame.Received)
    {
        int32_t delta = (int32_t)ecd - (int32_t)Last_Ecd;
        if (delta > 4096)       { delta -= 8192; }
        else if (delta < -4096) { delta += 8192; }
        frame.Measure.Total_Angle += (float)delta * DJI_ECD_ANGLE_COEF;
    }
    Last_Ecd = ecd;

    frame.Last_Rx_Time_Us = Sys_Get_Micros();
    frame.Received        = true;

    /* 短临界区整帧提交（本函数在 CAN 中断里跑；primask 保存恢复支持嵌套） */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    frame.Sequence = Feedback.Sequence + 1U;    /* 序号在提交时分配，保证单调 */
    Feedback = frame;
    __DMB();
    __set_PRIMASK(primask);
}

bool Class_DJI_Motor::Get_Feedback_Snapshot(Struct_DJI_Motor_Feedback &__Out) const
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __Out = Feedback;
    const bool received = __Out.Received;
    __DMB();
    __set_PRIMASK(primask);
    return received;
}

void DJI_Motor_Control_Task()
{
    if (motor_can_handle == nullptr) { return; }

    /* 型号决定控制帧 ID；GM6020 只发送电流控制报文。 */
    static constexpr uint32_t tx_ids[MOTOR_TYPE_COUNT][2] = {
        {0x200U, 0x1FFU},
        {0x1FEU, 0x2FEU},
    };
    for (uint8_t type = 0; type < MOTOR_TYPE_COUNT; type++)
    {
        for (uint8_t i = 0; i < DJI_MOTOR_CNT; i++)
        {
            if (motors[type][i] != nullptr) { motors[type][i]->Control(); }
        }

        /* 每四个硬件 ID 为一组；未创建的槽位补零，整组无对象则不发送。
         * GM6020 的 8 号创建被拒绝，所以 0x2FE 的最后两字节恒为零。 */
        for (uint8_t group = 0; group < 2; group++)
        {
            bool group_has_motor = false;
            uint8_t frame[8] = {};
            for (uint8_t j = 0; j < 4; j++)
            {
                Class_DJI_Motor *motor = motors[type][group * 4 + j];
                if (motor == nullptr) { continue; }
                group_has_motor = true;
                const uint16_t current = static_cast<uint16_t>(motor->Get_Out_Current());
                frame[j * 2]     = static_cast<uint8_t>(current >> 8);
                frame[j * 2 + 1] = static_cast<uint8_t>(current);
            }
            if (group_has_motor)
            {
                CAN_Transmit(motor_can_handle, tx_ids[type][group], frame, 8);
            }
        }
    }
}
