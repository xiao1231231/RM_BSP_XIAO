/**
 * @file    dji_motor.h
 * @brief   DJI 3508 电机（C620 电调）—— 反馈解析 + 转速闭环
 *
 * @note    协议（C620）：
 *            反馈 0x200+编号（0x201~），8 字节 = 角度/转速/电流/温度
 *            控制 0x200（1~4号）与 0x1FF（5~8号），8 字节 = 4 个 int16 电流
 *            电流指令 ±16384 ↔ ±20A；输出轴转速 = 转子转速 ÷ 19.2
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

#define DJI_MOTOR_CNT       8
#define DJI_ECD_ANGLE_COEF  0.043945f   /* 360 / 8192 */
#define DJI_GEAR_RATIO      19.2f       /* 3508 减速比 */
#define DJI_OUT_CURRENT_MAX 16384.0f    /* C620 电流指令限幅（±20A） */

/** 反馈超时判定：超过这么久没收到新帧就判失联。
 *  反馈名义 1kHz，5ms = 连丢 5 帧 —— 容纳总线抖动，又不至于拖太久。 */
#define DJI_FEEDBACK_TIMEOUT_US 5000U

/**
 * @brief 四个电机的编号（本车约定）
 * @note  编号即 CAN 协议地址：反馈帧 = 0x200 + 编号（0x201~0x204），
 *        控制帧 0x200 里第 (编号-1) 对字节是它的电流指令。
 *        ★ 电调自身的 ID 必须和这里一致（上电"滴"声次数 = 电调当前编号），
 *          改电调 ID 用 RoboMaster Assistant + USB-CAN 模块。
 */
enum Enum_DJI_Motor_Num
{
    DJI_MOTOR_1 = 1,
    DJI_MOTOR_2 = 2,
    DJI_MOTOR_3 = 3,
    DJI_MOTOR_4 = 4,
};

struct Struct_DJI_Motor_Measure
{
    uint16_t Ecd;                   /* 转子机械角度 0~8191 */
    float Speed_Rpm;                /* 输出轴转速 = 转子转速 ÷ 19.2 */
    int16_t Torque_Current;         /* 实际转矩电流（反馈） */
    uint8_t Temperature;            /* 电调温度 °C */
    float Total_Angle;              /* 多圈累计角度（度） */
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
              float K_P, float K_I, float K_D);

    inline void Set_Target_Speed_Rpm(const float &rpm) { Target_Speed_Rpm = rpm; }

    /** 跑一次速度环（1kHz，由 DJI_Motor_Control_Task 逐个调用） */
    void Control();

    /** 解析本电机的反馈帧（bsp_can 按 ID 分发进来，Data 恒 8 字节） */
    void Feedback_Parse(uint8_t *Data, uint16_t Length);

    /** 整帧复制最近反馈（返回 Received）。跨上下文读反馈只走这个接口 */
    bool Get_Feedback_Snapshot(Struct_DJI_Motor_Feedback &__Out) const;

    inline float Get_Target_Speed_Rpm() const { return Target_Speed_Rpm; }
    inline int16_t Get_Out_Current() const { return Out_Current; }
    inline uint8_t Get_Motor_Id() const { return Motor_Id; }

protected:
    CAN_HandleTypeDef *CAN_Handler;
    uint8_t Motor_Id;               /* 1~8 */
    /* 已提交的反馈整帧（CAN 中断写、任务读，两边都在短临界区里） */
    Struct_DJI_Motor_Feedback Feedback = {};
    uint16_t Last_Ecd = 0U;         /* 上一帧编码器值（算多圈增量） */
    Class_PID Speed_PID;
    float Target_Speed_Rpm = 0.0f;
    int16_t Out_Current = 0;
};

/** 绑定总线（system_init 里 CAN_Init 之后调一次） */
void DJI_Motor_Init(CAN_HandleTypeDef *hcan);

/** 创建一个电机（内部自动向 bsp_can 注册它的反馈 ID）。重复创建同一编号返回 nullptr */
Class_DJI_Motor *DJI_Motor_Create(Enum_DJI_Motor_Num Motor_Num,
                                  float K_P, float K_I, float K_D);

/** 取电机指针，没创建过返回 nullptr */
Class_DJI_Motor *DJI_Motor_Get(Enum_DJI_Motor_Num Motor_Num);

/** 1kHz 调用：逐个跑速度环 → 按 0x200/0x1FF 两组拼帧发送 */
void DJI_Motor_Control_Task();

#endif /* DJI_MOTOR_H */
