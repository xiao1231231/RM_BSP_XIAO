/**
 * @file    sys_attitude.h
 * @brief   姿态解算对外接口（本工程唯一的姿态来源）
 *
 * @note    ★ 实现已换成 H7_BSP 那套（BMI088 状态机 + FIFO + VQF），
 *          波形输出和控制应用通过本接口获取整帧姿态结果。
 *
 *          数据流（细节见 Device/Onboard/BMI088/）：
 *            陀螺 INT3(PC5) 每 500µs 就绪 → EXTI → SPI DMA 读 FIFO
 *              → 回调解析 + 重建每帧时间戳 → 样本队列
 *              → BMI088_Task 逐样本跑 VQF → 姿态结果
 *            加速度 INT1(PC4) 就绪 → 读一次 → 下一帧解算时做重力修正
 *            温度 128ms 读一次（慢变量）
 *          BMI088_Task 每次解算后调用 Attitude_Task()，整帧发布姿态结果。
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
#include <stdbool.h>

/** 最大样本年龄：默认 100ms，与当前滤波器断流重置阈值一致。
 *  控制应用需要更短的故障响应时，应收紧此值并验证队列延迟。 */
#define ATTITUDE_MAX_SAMPLE_AGE_US 100000U

/* ── 零偏标定（KEY 长按触发，板载 30s 采样 → Flash，见 .cpp 内注释）── */

/** 零偏来源（调试器观察）：0=无 1=开机静止标定 2=Flash 里存的标定 */
extern volatile uint8_t Attitude_Bias_Source;

/** KEY 模块检测到"长按 4 秒"后调用：请求一次标定（重复请求/条件不满足自动忽略） */
void Attitude_Calibration_Request(void);

/** Calibration_Task 每 1ms 推进标定：采样、检查、写 Flash，结果提交到 USB。
 *  Flash 保存为同步操作，擦写期间可能暂停任务执行，耗时需在目标板验证。 */
void Attitude_Calibration_Service(void);

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

    /* ── 新鲜度：跨任务读姿态必须看这三项 ── */
    uint64_t Sample_Time_Us; /**< 本帧对应的【陀螺样本时刻】，不是"现在" */
    uint32_t Sequence;       /**< 发布序号，每产出新一帧 +1（判断有没有新帧） */
    uint8_t  Valid;          /**< 1 = 已产出且样本未过期；未初始化 / 断流超时为 0 */
} Struct_Attitude;

extern Struct_Attitude Attitude;

/**
 * @brief 整帧复制最近一帧姿态（★ 跨任务读姿态统一走这里）
 *
 * @note  BMI088_Task 在短临界区内整帧发布 Attitude；读者也必须整帧取用，
 *        避免逐个读字段时被发布任务抢占，混合不同帧的数据。
 *        本函数在关中断窗口内把整帧一次拷走（同时也挡住任务切换），
 *        保证 Out 里所有字段来自同一帧。
 *
 *        复制后会重新检查样本年龄，即使解算任务停止更新也会失效。
 *        数据是否可用看 Out->Valid；是否是新样本则比对 Sequence。
 *        过期时保留数值和时间戳，仅将输出副本的 Valid 置零。
 *
 * @param Out 复制结果
 * @return Out->Valid
 */
bool Attitude_Get_Snapshot(Struct_Attitude *Out);

/**
 * @brief 初始化姿态解算（SPI 层 + VQF 参数 + BMI088 硬件）
 * @note  必须在调度器启动前调用 —— 里面是【阻塞】的：
 *        每个配置步骤都带读回校验、失败重试 5 次（失败时每次等 100ms），
 *        最坏情况要几秒。所以不能放进任务里做。
 *        ⚠️ 零偏初值来源（见 Attitude_Bias_Source）：
 *           Flash 里有有效标定（按键触发的板载标定写入）→ 直接加载，
 *           【无需静止等待】；
 *           没有时退回 1 秒开机静止标定 —— 仅那一次需要静止放好。
 * @return 无返回值；初始化失败时姿态不会更新
 *         （串口只发波形、没有日志，表现就是波形上的角度一直 0 不动）
 */
void Attitude_Init(void);

/**
 * @brief 整帧发布姿态结果，由 BMI088_Task 每次解算后调用
 * @note  ★ 姿态解算本身【不在】这里跑：
 *        陀螺积分在 BMI088_Task 里（每来一帧样本算一次，2kHz），
 *        这里是纯搬运，耗时只有几十个浮点拷贝。
 */
void Attitude_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* SYS_ATTITUDE_H */
