/**
 * @file    dji_motor.h
 * @brief   DJI 3508/C620 与 GM6020（电流模式）—— 反馈解析 + 转速闭环
 *
 * @note    协议（C620）：
 *            反馈 0x200+编号（0x201~），8 字节 = 角度/转速/电流/温度
 *            控制 0x200（1~4号）与 0x1FF（5~8号），8 字节 = 4 个 int16 电流
 *            电流指令 ±16384 ↔ ±20A；输出轴转速 = 转子转速 ÷ 19.2
 *
 *          协议（GM6020 电流模式）：
 *            反馈 0x204+编号，控制 0x1FE（1~4号）与 0x2FE（5~7号）
 *            电流指令 ±16384 ↔ ±3A；反馈转速即输出轴 rpm，减速比为 1
 *            固件 >= 1.0.11.2，并用 RoboMaster Assistant >= 2.7 开启电流环
 *          当前 CAN1 固定使用 3508 的 1~4 号和 GM6020 的 1、2 号。
 *
 *          反馈可用性契约：CAN 反馈由中断逐帧提交【整帧快照】（含接收时刻、
 *          序号、是否收到过）；速度环每拍检查新鲜度 —— 没收到过反馈或超过
 *          DJI_FEEDBACK_TIMEOUT_US 没更新，就输出零电流并清 PID 历史，
 *          绝不拿冻结/全零的旧值闭环。跨上下文读反馈统一走
 *          Get_Feedback_Snapshot()（整帧复制），不要直读内部对象。
 */

#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "bsp_can.h"
#include "alg_pid.h"

#define DJI_MOTOR_CNT       8           /* 每种型号的静态池容量；GM6020 只允许 1~7 号 */
#define DJI_ECD_ANGLE_COEF  0.043945f   /* 360 / 8192 */
#define DJI_GEAR_RATIO      19.2f       /* 3508 减速比 */
#define DJI_OUT_CURRENT_MAX 16384.0f    /* 原始电流指令限幅；C620 ±20A，GM6020 ±3A */

/** 反馈超时判定：超过这么久没收到新帧就判失联。
 *  反馈名义 1kHz，5ms = 连丢 5 帧 —— 容纳总线抖动，又不至于拖太久。 */
#define DJI_FEEDBACK_TIMEOUT_US 5000U

/**
 * @brief 电机硬件编号；对象由【型号 + 编号】共同标识
 * @note  本工程创建 3508 的 1~4 号、GM6020 的 1、2 号。
 *        C620 编号由电调设置，GM6020 编号由拨码开关设置。
 */
enum Enum_DJI_Motor_Num : uint32_t
{
    DJI_MOTOR_1 = 1,
    DJI_MOTOR_2 = 2,
    DJI_MOTOR_3 = 3,
    DJI_MOTOR_4 = 4,
    DJI_MOTOR_5 = 5,
    DJI_MOTOR_6 = 6,
    DJI_MOTOR_7 = 7,
    DJI_MOTOR_8 = 8,               /* C620 有 8 号；GM6020 不接受此编号 */
};

enum Enum_DJI_Motor_Type : uint8_t
{
    DJI_MOTOR_TYPE_M3508 = 0,
    DJI_MOTOR_TYPE_GM6020 = 1,
};

struct Struct_DJI_Motor_Measure
{
    uint16_t Ecd;                   /* 转子机械角度 0~8191 */
    float Speed_Rpm;                /* 输出轴 rpm：3508 除以 19.2，GM6020 直接使用反馈 */
    int16_t Torque_Current;         /* 反馈转矩电流原始值，单位不是 A */
    uint8_t Temperature;            /* 驱动器温度 °C */
    float Total_Angle;              /* 编码器多圈累计角度（度），首帧为零 */
};

/**
 * @brief 一帧完整的电机反馈：测量值 + 新鲜度（语义同姿态整帧快照）
 * @note  由 CAN 中断一次提交、读者一次复制，保证 Measure 里的每个字段
 *        都来自同一帧反馈；Sequence 每帧 +1，Last_Rx_Time_Us 是接收时刻。
 *        "创建成功"≠"电调在线"：Received == false 说明一帧都没收到过
 *        （电调没上电 / ID 不对 / 线没接）。
 */
struct Struct_DJI_Motor_Feedback
{
    Struct_DJI_Motor_Measure Measure;
    uint64_t Last_Rx_Time_Us = 0U;  /* 本帧接收时刻（Sys_Get_Micros） */
    uint32_t Sequence = 0U;         /* 每收到一帧 +1 */
    bool Received = false;          /* 是否至少收到过一帧 */
};

class Class_DJI_Motor
{
public:
    void Init(CAN_HandleTypeDef *hcan, uint8_t Motor_Id,
              float K_P, float K_I, float K_D,
              Enum_DJI_Motor_Type Motor_Type = DJI_MOTOR_TYPE_M3508);

    /* 目标由 Motor_Task 设置；零目标仍运行速度环，将转速控制到零。 */
    inline void Set_Target_Speed_Rpm(const float &rpm) { Target_Speed_Rpm = rpm; }

    /** 跑一次速度环（1kHz，由 DJI_Motor_Control_Task 逐个调用） */
    void Control();

    /** 解析本电机的反馈帧；拒绝空指针、非 8 字节和超范围编码器值 */
    void Feedback_Parse(uint8_t *Data, uint16_t Length);

    /** 整帧复制最近反馈（返回 Received）。跨上下文读反馈只走这个接口 */
    bool Get_Feedback_Snapshot(Struct_DJI_Motor_Feedback &__Out) const;

    inline float Get_Target_Speed_Rpm() const { return Target_Speed_Rpm; }
    /** 返回原始 CAN 电流指令；±16384 对应 C620 ±20A、GM6020 ±3A */
    inline int16_t Get_Out_Current() const { return Out_Current; }
    inline uint8_t Get_Motor_Id() const { return Motor_Id; }
    inline Enum_DJI_Motor_Type Get_Motor_Type() const { return Motor_Type; }

protected:
    CAN_HandleTypeDef *CAN_Handler = nullptr;
    uint8_t Motor_Id = 0U;          /* C620 1~8；GM6020 1~7 */
    Enum_DJI_Motor_Type Motor_Type = DJI_MOTOR_TYPE_M3508;
    float Gear_Ratio = DJI_GEAR_RATIO;
    /* 已提交的反馈整帧（CAN 中断写、任务读，两边都在短临界区里） */
    Struct_DJI_Motor_Feedback Feedback = {};
    uint16_t Last_Ecd = 0U;         /* 上一帧编码器值（算多圈增量） */
    Class_PID Speed_PID;
    float Target_Speed_Rpm = 0.0f;
    int16_t Out_Current = 0;
};

/** 绑定总线（system_init 里 CAN_Init 之后调一次） */
void DJI_Motor_Init(CAN_HandleTypeDef *hcan);

/** 创建电机并注册反馈；同型号/编号重复、编号非法或注册失败返回 nullptr。
 *  初始化阶段调用。默认型号为 3508，原有调用方式保持有效。 */
Class_DJI_Motor *DJI_Motor_Create(Enum_DJI_Motor_Num Motor_Num,
                                  float K_P, float K_I, float K_D,
                                  Enum_DJI_Motor_Type Motor_Type = DJI_MOTOR_TYPE_M3508);

/** 按型号和编号取电机指针，未创建或参数非法返回 nullptr；默认取 3508 */
Class_DJI_Motor *DJI_Motor_Get(Enum_DJI_Motor_Num Motor_Num,
                              Enum_DJI_Motor_Type Motor_Type = DJI_MOTOR_TYPE_M3508);

/** 1kHz 调用：统一跑速度环；3508 按 0x200/0x1FF、GM6020 按 0x1FE/0x2FE 拼帧 */
void DJI_Motor_Control_Task();

/** 是否有任一电机的目标转速非零（IMU 零偏标定前的活动检查） */
bool DJI_Motor_Any_Target_Active(void);

#endif /* DJI_MOTOR_H */
