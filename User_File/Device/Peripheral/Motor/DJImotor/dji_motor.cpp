/**
 * @file    dji_motor.cpp
 * @brief   DJI 3508 电机实现
 *
 * @note    分工：CAN 收到反馈只做【解析】（快进快出）；
 *          控制在 1kHz 任务里统一跑 —— PID 的 D_T 必须和调用周期一致。
 */

#include "dji_motor.h"

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

    /* 静态池：全局存储、地址确定，与本工程其它模块一致（不 malloc） */
    static Class_DJI_Motor pool[DJI_MOTOR_CNT];
    static bool used[DJI_MOTOR_CNT] = {};
    uint8_t slot;
    for (slot = 0; slot < DJI_MOTOR_CNT; slot++)
    {
        if (!used[slot]) { break; }
    }
    if (slot == DJI_MOTOR_CNT) { return nullptr; }   /* 满池：不许越界写 pool[8] */

    Class_DJI_Motor *motor = &pool[slot];
    used[slot] = true;
    motor->Init(motor_can_handle, motor_id, K_P, K_I, K_D);
    motors[motor_id - 1] = motor;

    /* 向 bsp_can 注册本电机的反馈帧（0x201 ~ 0x204） */
    CAN_Register_Device(motor_can_handle, 0x200U + motor_id, motor,
                        Motor_Feedback_Callback);
    return motor;
}

Class_DJI_Motor *DJI_Motor_Get(Enum_DJI_Motor_Num Motor_Num)
{
    const uint8_t motor_id = (uint8_t)Motor_Num;
    return (motor_id >= 1 && motor_id <= DJI_MOTOR_CNT) ? motors[motor_id - 1] : nullptr;
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
    Speed_PID.Set_Target(Target_Speed_Rpm);
    Speed_PID.Set_Now(Measure.Speed_Rpm);
    Speed_PID.TIM_Calculate_PeriodElapsedCallback();
    Out_Current = (int16_t)Speed_PID.Get_Out();
}

void Class_DJI_Motor::Feedback_Parse(uint8_t *Data, uint16_t Length)
{
    if (Length != 8) { return; }

    const uint16_t last_ecd = Measure.Ecd;
    Measure.Ecd            = (uint16_t)(Data[0] << 8 | Data[1]);
    const int16_t rotor_rpm = (int16_t)(Data[2] << 8 | Data[3]);
    Measure.Torque_Current = (int16_t)(Data[4] << 8 | Data[5]);
    Measure.Temperature    = Data[6];
    Measure.Speed_Rpm      = (float)rotor_rpm / DJI_GEAR_RATIO;

    /* 多圈累计：8192 过零判向（350→10 是正转过零，10→350 是反转过零） */
    if (Ecd_Initialized)
    {
        int32_t delta = (int32_t)Measure.Ecd - (int32_t)last_ecd;
        if (delta > 4096)       { delta -= 8192; }
        else if (delta < -4096) { delta += 8192; }
        Measure.Total_Angle += (float)delta * DJI_ECD_ANGLE_COEF;
    }
    Ecd_Initialized = true;
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
