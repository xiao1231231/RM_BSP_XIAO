/**
 * @file    sys_attitude.h
 * @brief   姿态解算对外接口（本工程唯一的姿态来源）
 *
 * @note    ★ 实现已换成 H7_BSP 那套（BMI088 状态机 + FIFO + VQF），
 *          但对外接口保持不变 —— 波形、日志、后续章节都只认这里。
 *
 *          数据流（细节见 Device/Onboard/BMI088/）：
 *            陀螺 INT3(PC5) 每 500µs 就绪 → EXTI → SPI DMA 读 FIFO
 *              → 回调解析 + 重建每帧时间戳 → 样本队列
 *              → BMI088_Task 逐样本跑 VQF → 姿态结果
 *            加速度 INT1(PC4) 就绪 → 读一次 → 下一帧解算时做重力修正
 *            温度 128ms 读一次（慢变量）
 *          Attitude_Task()（1ms 调用）只做一件事：把结果搬进下面这个结构体。
 *
 * @note    ★ VQF 的核心价值：在线零偏估计，尤其是【静止期】的估计
 *
 *          6 轴方案（加速度计 + 陀螺仪，无磁力计）下，绕重力轴的旋转不改变
 *          重力方向，加速度计看不见 —— 偏航角【不可观】，偏航零偏也就没有
 *          直接观测量。它只能靠两样东西压住：
 *            ① 静止时"角速度应为 0"这个先验（VQF 自动做，权重最高）
 *            ② 运动时水平方向适度估计；垂直方向权重只有水平方向的万分之一
 *          → VQF 跟踪的是【静止期的零偏慢变化（主要是温漂）】；
 *            运动中偏航零偏基本估不出来，别指望它。
 */

#ifndef SYS_ATTITUDE_H
#define SYS_ATTITUDE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 姿态输出结构。除三轴姿态外，另带两项 VQF 独有的诊断量：零偏估计和静止标志 */
typedef struct
{
    float q[4];          /**< 四元数 [w, x, y, z] */
    float Gyro[3];       /**< 机体系角速度 rad/s（未扣零偏的原始值） */
    float Accel[3];      /**< 机体系加速度 m/s^2 */

    float Roll;          /**< 横滚角，单位【度】 */
    float Pitch;         /**< 俯仰角，单位【度】 */
    float Yaw;           /**< 偏航角，单位【度】 */

    float GyroBias[3];   /**< ★ 在线估计的陀螺零偏 rad/s —— 持续跟踪温漂的关键 */
    uint8_t Rest_Detected;  /**< ★ 静止检测标志（判定静止时，零偏修正权重最高） */
} Struct_Attitude;

extern Struct_Attitude Attitude;

/* 并发契约：唯一写者是 1ms 任务的 Attitude_Task()；当前读者（波形输出）
 * 也在同一任务里，直读安全。将来跨任务读（控制环 / 上位机）时，
 * 单个 float 的原子性不能保证你拿到一组【同一拍】的数据 ——
 * 那时要么复制一份再用，要么把读也放进 1ms 任务。 */

/**
 * @brief 初始化姿态解算（SPI 层 + VQF 参数 + BMI088 硬件）
 * @note  必须在调度器启动前调用 —— 里面是【阻塞】的：
 *        每个配置步骤都带读回校验、失败重试 5 次（失败时每次等 100ms），
 *        最坏情况要几秒。所以不能放进任务里做。
 *        ⚠️ 期间板子必须静止放好 —— 最后一步要做 1 秒开机零偏标定
 *           （静止采 2000 个陀螺样本取平均，喂给 VQF 当零偏初值）。
 *           晃动着开机，标出来的就是"运动速度"，喂进去反而放大漂移。
 * @return 无返回值；初始化失败时姿态不会更新
 *         （串口只发波形、没有日志，表现就是波形上的角度一直 0 不动）
 */
void Attitude_Init(void);

/**
 * @brief 把姿态结果搬到对外结构体，1kHz 调用
 * @note  ★ 姿态解算本身【不在】这里跑：
 *        陀螺积分在 BMI088_Task 里（每来一帧样本算一次，2kHz），
 *        这里是纯搬运，耗时只有几十个浮点拷贝。
 */
void Attitude_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* SYS_ATTITUDE_H */
