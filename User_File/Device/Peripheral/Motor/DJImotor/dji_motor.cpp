/**
 * @file    dji_motor.cpp
 * @brief   DJI 3508 电机实现
 *
 * @note    分工：CAN 收到反馈只做【解析】（快进快出）；
 *          控制在 1kHz 任务里统一跑 —— PID 的 D_T 必须和调用周期一致。
 */

#include "dji_motor.h"

#include "sys_timestamp.h"      /* Sys_Get_Micros() —— 反馈新鲜度判定 */

static CAN_HandleTypeDef *motor_can_handle = nullptr;
static Class_DJI_Motor *motors[DJI_MOTOR_CNT] = {};   /* 下标 = 编号-1 */

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
                                  float K_P, float K_I, float K_D)
{
    const uint8_t motor_id = (uint8_t)Motor_Num;
    if (motor_can_handle == nullptr ||
        motor_id < 1 || motor_id > DJI_MOTOR_CNT ||
        motors[motor_id - 1] != nullptr)
    {
        return nullptr;
    }

    /* 静态池按电机编号索引，不 malloc；重复编号已在上方拦截。 */
    static Class_DJI_Motor pool[DJI_MOTOR_CNT];
    Class_DJI_Motor *motor = &pool[motor_id - 1];
    motor->Init(motor_can_handle, motor_id, K_P, K_I, K_D);
    motors[motor_id - 1] = motor;

    /* 向 bsp_can 注册本电机的反馈帧（0x201 ~ 0x204）。
     * ★ 注册失败必须回滚并报错：否则会拿到"对象存在、能发指令、
     *   却永远收不到反馈"的半初始化电机 —— 速度环拿冻结的转速去闭环，
     *   症状离案发现场十万八千里，极难排查。 */
    if (!CAN_Register_Device(motor_can_handle, 0x200U + motor_id, motor,
                             Motor_Feedback_Callback))
    {
        motors[motor_id - 1] = nullptr;
        return nullptr;
    }
    return motor;
}

Class_DJI_Motor *DJI_Motor_Get(Enum_DJI_Motor_Num Motor_Num)
{
    const uint8_t motor_id = (uint8_t)Motor_Num;
    return (motor_id >= 1 && motor_id <= DJI_MOTOR_CNT) ? motors[motor_id - 1] : nullptr;
}

bool DJI_Motor_Any_Target_Active(void)
{
    for (uint8_t i = 0U; i < DJI_MOTOR_CNT; i++)
    {
        if (motors[i] != nullptr && motors[i]->Get_Target_Speed_Rpm() != 0.0f)
        {
            return true;
        }
    }
    return false;
}

void Class_DJI_Motor::Init(CAN_HandleTypeDef *hcan, uint8_t Motor_Id,
                           float K_P, float K_I, float K_D)
{
    CAN_Handler = hcan;
    this->Motor_Id = Motor_Id;

    /* 速度环 1kHz：输出=电流指令，限幅 ±16384 */
    Speed_PID.Init(K_P, K_I, K_D, 0.0f, DJI_OUT_CURRENT_MAX, DJI_OUT_CURRENT_MAX, 0.001f);
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
        (Sys_Get_Micros() - feedback.Last_Rx_Time_Us) > DJI_FEEDBACK_TIMEOUT_US)
    {
        Out_Current = 0;
        Speed_PID.Reset();      /* 每拍清，幂等；恢复瞬间无历史负担 */
        return;
    }

    Speed_PID.Set_Target(Target_Speed_Rpm);
    Speed_PID.Set_Now(feedback.Measure.Speed_Rpm);
    Speed_PID.TIM_Calculate_PeriodElapsedCallback();
    Out_Current = (int16_t)Speed_PID.Get_Out();
}

void Class_DJI_Motor::Feedback_Parse(uint8_t *Data, uint16_t Length)
{
    if (Length != 8) { return; }

    /* 以已提交的帧为底，在栈上把这一帧凑齐再整体提交 ——
     * 读者拿到的 Measure 各字段保证来自同一次 CAN 反馈 */
    Struct_DJI_Motor_Feedback frame = Feedback;
    const uint16_t ecd      = (uint16_t)(Data[0] << 8 | Data[1]);
    const int16_t rotor_rpm = (int16_t)(Data[2] << 8 | Data[3]);
    frame.Measure.Ecd            = ecd;
    frame.Measure.Torque_Current = (int16_t)(Data[4] << 8 | Data[5]);
    frame.Measure.Temperature    = Data[6];
    frame.Measure.Speed_Rpm      = (float)rotor_rpm / DJI_GEAR_RATIO;

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

    /* ① 逐电机跑速度环 */
    for (uint8_t i = 0; i < DJI_MOTOR_CNT; i++)
    {
        if (motors[i] != nullptr) { motors[i]->Control(); }
    }

    /* ② 按组拼帧：0x200 = 1~4 号，0x1FF = 5~8 号（某组没电机就不发那帧） */
    for (uint8_t group = 0; group < 2; group++)
    {
        bool group_has_motor = false;
        uint8_t frame[8] = {};
        for (uint8_t j = 0; j < 4; j++)
        {
            Class_DJI_Motor *motor = motors[group * 4 + j];
            if (motor == nullptr) { continue; }
            group_has_motor = true;
            const int16_t current = motor->Get_Out_Current();
            frame[j * 2]     = (uint8_t)((uint16_t)current >> 8);   /* 大端 */
            frame[j * 2 + 1] = (uint8_t)((uint16_t)current);
        }
        if (group_has_motor)
        {
            CAN_Transmit(motor_can_handle, (group == 0) ? 0x200U : 0x1FFU, frame, 8);
        }
    }
}
