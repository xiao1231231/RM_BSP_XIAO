/**
 * @file    sys_attitude.cpp
 * @brief   姿态解算对外接口实现（接 H7_BSP 那套 BMI088 + VQF）
 *
 * @note    本模块负责：
 *            ① 把 SPI 层和 BMI088 那套接起来（初始化）
 *            ② 配置 VQF 参数
 *            ③ 每次解算后整帧发布 Attitude，并提供跨任务快照
 *            ④ 按键触发的零偏标定、Flash 保存和 USB 回执
 *          真正的解算在 BMI088_Task 里（见 Task/BMI088_Task.cpp）。
 */

#include "sys_attitude.h"

#include "bsp_spi.h"        /* SPI_Init */
#include "bsp_bmi088.h"     /* BSP_BMI088 */
#include "bsp_flash.h"      /* 零偏标定的 Flash 存储 */
#include "sys_debug.h"      /* USB_Post_Printf —— 提交标定回执 */
#include "sys_timestamp.h"  /* 样本新鲜度使用单调微秒时间 */
#include "dji_motor.h"      /* DJI_Motor_Any_Target_Active —— 标定前检查电机空闲 */
#include "callback.h"       /* SPI1_Callback */
#include "spi.h"            /* hspi1 —— BMI088 挂在这条 SPI 上 */

#include "cmsis_os2.h"      /* osKernelGetTickCount —— 毫秒级相位计时 */
#include <math.h>           /* sqrtf —— 静止体检算标准差 */

#define RAD_2_DEG   57.29577951f

Struct_Attitude Attitude;

static bool Attitude_Sample_Is_Fresh(uint64_t sample_time_us)
{
    const uint64_t now_us = Sys_Get_Micros();
    return sample_time_us != 0U && sample_time_us <= now_us &&
           now_us - sample_time_us <= ATTITUDE_MAX_SAMPLE_AGE_US;
}

/* ══════════════ 零偏标定：KEY 长按 → 板载 30s 采样 → Flash ══════════════
 *
 * 数据流（借鉴 26h循迹 的按键+标定状态机模式，但只保留必要环节）：
 *   KEY 模块（1kHz 扫描）检测到"长按 4 秒" → Attitude_Calibration_Request()
 *   Calibration_Task 每 1ms 推进 Attitude_Calibration_Service() 状态机，
 *     IDLE ──启动前拒绝检查（IMU 就绪？电机没在转？）──
 *         → SETTLE：启动后静置 2 秒
 *         → SAMPLING：收集 30000 个新样本（正常调度约 30s）
 *         → SAVING：静止体检通过后同步保存 Flash
 *         → IDLE：提交 USB 结果回执；失败或中止也直接回到 IDLE
 *
 * 三道质量闸门（都是为了让写进 Flash 的数真的代表零偏）：
 *   ① 启动前：IMU 未就绪拒绝；有电机目标非零拒绝（板子不安静）
 *   ② 采样中：电机被启动 → 立即中止
 *   ③ 写入前：三轴标准差超限（板子被碰过）→ 拒绝写入
 *
 * 交互约定：
 *   · 长按不足 4 秒松开 = 放弃；标定进行中再按键不响应
 *   · 采样值 = 姿态快照的 Gyro（未扣零偏的原始陀螺 rad/s），均值即零偏，
 *     语义 = "VQF 要从原始值里减掉的量"，与 Flash 存储/启动加载共用一套约定
 */
volatile uint8_t Attitude_Bias_Source = 0U;

static volatile bool s_Calibration_Requested = false;   /* KEY 长按置位 */

enum Enum_Calibration_State
{
    CAL_IDLE = 0,       /* 无标定进行 */
    CAL_SETTLE,         /* 启动后的静置缓冲 */
    CAL_SAMPLING,       /* 静止采样中 */
    CAL_SAVING,         /* 同步保存 Flash */
};
static Enum_Calibration_State s_Cal_State = CAL_IDLE;

/** 失败原因（USB 回执已含文字说明，这里给调试器一个数值） */
enum Enum_Calibration_Fail
{
    CAL_FAIL_NONE = 0U,
    CAL_FAIL_IMU_NOT_READY,     /* IMU 没初始化完 */
    CAL_FAIL_MOTOR_ACTIVE,      /* 有电机目标非零：板子不安静 */
    CAL_FAIL_MOTION,            /* 静止体检不过：采样期间被碰了 */
    CAL_FAIL_SAMPLE_TIMEOUT,    /* 采样超时：姿态帧不再更新（IMU 断流） */
    CAL_FAIL_FLASH,             /* 写 Flash 或读回校验失败 */
    CAL_FAIL_SAMPLE_STALE,      /* 姿态无效或样本过期 */
};
static uint8_t s_Cal_Fail_Reason = CAL_FAIL_NONE;

/* ── Welford 单遍均值/方差 ──
 * 为什么不用 sum += x：float 只有 7 位有效数字，3 万个量级相近的角速度
 * 朴素累加会持续舍入，均值被推歪；Welford 是增量式、数值稳定。
 * 附带收益：M2 直接给出标准差，用来做"采样期间确实静止"的体检。 */
static uint32_t s_Cal_Count = 0U;
static float s_Cal_Mean[3] = {0.0f, 0.0f, 0.0f};
static float s_Cal_M2[3] = {0.0f, 0.0f, 0.0f};

static uint32_t s_Cal_Tick_Start = 0U;      /* 相位计时起点（RTOS ms 节拍） */
static uint32_t s_Cal_Last_Sequence = 0U;   /* 已计入的姿态帧序号（判新样本） */

/* 参数（都在这里调） */
static constexpr uint32_t CAL_SETTLE_MS = 2000U;       /* 启动后的静置缓冲 */
static constexpr uint32_t CAL_TOTAL_SAMPLES = 30000U;  /* 每轮最多接收一帧，正常调度约 30s */
static constexpr uint32_t CAL_SAMPLE_TIMEOUT_MS = 45000U; /* 采样硬超时（30s + 余量） */
static constexpr float    CAL_MAX_STD_RAD_S = 0.05f;   /* 静止体检阈值（≈2.9°/s） */

void Attitude_Calibration_Request(void)
{
    if (!BSP_BMI088.Is_Initialized()) { return; }         /* IMU 没就绪：忽略 */

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    s_Calibration_Requested = true;

    __DMB();
    __set_PRIMASK(primask);
}

/* ── 状态机内部辅助 ── */

/** Welford 单步：增量更新均值与 M2（数值稳定，见变量区注释） */
static void Calibration_Welford_Update(const float gyro[3])
{
    s_Cal_Count++;
    const float inv = 1.0f / (float)s_Cal_Count;
    for (uint8_t k = 0U; k < 3U; k++)
    {
        const float x = gyro[k];
        const float delta = x - s_Cal_Mean[k];
        s_Cal_Mean[k] += delta * inv;
        s_Cal_M2[k] += delta * (x - s_Cal_Mean[k]);
    }
}

/** 三轴标准差的最大值（写入前的静止体检） */
static float Calibration_Max_Std(void)
{
    float max_std = 0.0f;
    for (uint8_t k = 0U; k < 3U; k++)
    {
        const float variance = s_Cal_M2[k] / (float)s_Cal_Count;
        const float std = (variance > 0.0f) ? sqrtf(variance) : 0.0f;
        if (std > max_std) { max_std = std; }
    }
    return max_std;
}

/** 中止/失败：记录原因、提交 USB 回执、结束本次标定。 */
static void Calibration_Fail(uint8_t reason, const char *receipt)
{
    s_Cal_Fail_Reason = reason;
    USB_Post_Printf("%s", receipt);
    s_Cal_State = CAL_IDLE;
}

void Attitude_Calibration_Service(void)
{
    // 取走请求并清零，避免两个任务在此期间交叉访问
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const bool requested = s_Calibration_Requested;
    s_Calibration_Requested = false;

    __DMB();
    __set_PRIMASK(primask);

    switch (s_Cal_State)
    {
    case CAL_IDLE:
    {
        if (!requested) { return; }

        /* 启动前拒绝检查：条件不满足就不开始，原因回执出去 */
        if (!BSP_BMI088.Is_Initialized())
        {
            s_Cal_Fail_Reason = CAL_FAIL_IMU_NOT_READY;
            USB_Post_Printf("cal:reject,imu_not_ready");
            return;
        }
        if (DJI_Motor_Any_Target_Active())
        {
            s_Cal_Fail_Reason = CAL_FAIL_MOTOR_ACTIVE;
            USB_Post_Printf("cal:reject,motor_active");
            return;
        }

        s_Cal_Count = 0U;
        s_Cal_Mean[0] = s_Cal_Mean[1] = s_Cal_Mean[2] = 0.0f;
        s_Cal_M2[0] = s_Cal_M2[1] = s_Cal_M2[2] = 0.0f;
        Struct_Attitude snapshot = {};
        (void)Attitude_Get_Snapshot(&snapshot);
        s_Cal_Last_Sequence = snapshot.Sequence;
        s_Cal_Tick_Start = osKernelGetTickCount();
        s_Cal_State = CAL_SETTLE;
        USB_Post_Printf("cal:start,settle2s_then_sample30s");
        break;
    }

    case CAL_SETTLE:
    {
        /* 长按事件触发后静置 2 秒；此计时不等待按键松开。 */
        if (osKernelGetTickCount() - s_Cal_Tick_Start >= CAL_SETTLE_MS)
        {
            s_Cal_Count = 0U;
            s_Cal_Mean[0] = s_Cal_Mean[1] = s_Cal_Mean[2] = 0.0f;
            s_Cal_M2[0] = s_Cal_M2[1] = s_Cal_M2[2] = 0.0f;
            Struct_Attitude snapshot = {};
            (void)Attitude_Get_Snapshot(&snapshot);
            s_Cal_Last_Sequence = snapshot.Sequence;
            s_Cal_Tick_Start = osKernelGetTickCount();
            s_Cal_State = CAL_SAMPLING;
        }
        break;
    }

    case CAL_SAMPLING:
    {
        /* 采样中途电机被启动 → 立刻中止（板子不安静，数据不可信） */
        if (DJI_Motor_Any_Target_Active())
        {
            Calibration_Fail(CAL_FAIL_MOTOR_ACTIVE, "cal:abort,motor_active");
            break;
        }

        /* 采样超时：IMU 断流/故障时凑不满 3 万帧，到点中止而不是无限等 */
        if (osKernelGetTickCount() - s_Cal_Tick_Start >= CAL_SAMPLE_TIMEOUT_MS)
        {
            Calibration_Fail(CAL_FAIL_SAMPLE_TIMEOUT, "cal:abort,sample_timeout");
            break;
        }

        Struct_Attitude sample;
        if (!Attitude_Get_Snapshot(&sample))
        {
            Calibration_Fail(CAL_FAIL_SAMPLE_STALE, "cal:abort,sample_stale");
            break;
        }

        /* ★ 只接受【新】样本：姿态帧序号推进了才算一帧。
         *   不判序号的话，IMU 断流时姿态停在冻结值，同一帧会被数 3 万遍
         *   —— 标准差≈0 反而"通过体检"，把冻结值当零偏差写进 Flash。 */
        if (sample.Sequence != s_Cal_Last_Sequence)
        {
            s_Cal_Last_Sequence = sample.Sequence;
            Calibration_Welford_Update(sample.Gyro);
        }

        if (s_Cal_Count >= CAL_TOTAL_SAMPLES)
        {
            /* 静止体检：标准差超限 = 期间被碰过，拒绝写入 */
            if (Calibration_Max_Std() > CAL_MAX_STD_RAD_S)
            {
                Calibration_Fail(CAL_FAIL_MOTION, "cal:abort,motion");
                break;
            }
            s_Cal_State = CAL_SAVING;
        }
        break;
    }

    case CAL_SAVING:
    {
        /* 采样结束到本拍之间可能断流或启动电机，擦除前再检查一次。 */
        Struct_Attitude sample;
        if (!Attitude_Get_Snapshot(&sample))
        {
            Calibration_Fail(CAL_FAIL_SAMPLE_STALE, "cal:abort,sample_stale");
            break;
        }
        if (DJI_Motor_Any_Target_Active())
        {
            Calibration_Fail(CAL_FAIL_MOTOR_ACTIVE, "cal:abort,motor_active");
            break;
        }
        const float temperature = BSP_BMI088.Get_Temperature();

        /* 同步擦写 Flash 可能暂停任务执行，实际耗时需在目标板验证。 */
        if (BSP_Flash_Calibration_Save(s_Cal_Mean, temperature))
        {
            /* 通过"请求"接口喂给 VQF：由解算任务在样本边界应用 ——
             * 直接从本任务写滤波器会和 BMI088_Task 的零偏估计并发（见接口注释） */
            BSP_BMI088.Request_VQF_Bias_Estimate(Class_Matrix_f32<3, 1>(s_Cal_Mean));
            Attitude_Bias_Source = 2U;
            USB_Post_Printf("cal:done,ok,t=%.1f", (double)temperature);
        }
        else
        {
            s_Cal_Fail_Reason = CAL_FAIL_FLASH;
            USB_Post_Printf("cal:done,flash_fail");
        }

        s_Cal_State = CAL_IDLE;
        break;
    }
    }
}

void Attitude_Init(void)
{
    /* ══════════════════════════════════════════════════════════════════
     * ① SPI 层：绑定句柄 + 收帧回调
     *    回调实现在 System/callback/callback.cpp，按片选分发给 BMI088。
     * ══════════════════════════════════════════════════════════════════ */
    SPI_Init(&hspi1, SPI1_Callback);

    /* ══════════════════════════════════════════════════════════════════
     * ② VQF 参数：取值参照 H7_BSP 的 sys_imu.cpp（整车验证过的调参结果）
     *
     *    上游是【逐条显式赋值】的，这里照做 —— 好处是"这块板子上到底用了
     *    哪些值"一目了然，不用去翻算法库的默认值。
     *    ⚠️ 周期必须和【实际喂进算法的频率】严格一致 —— 这是两层概念，别混：
     *       陀螺 2kHz：硬件 FIFO 每 500µs 一帧、逐帧喂 → 0.0005 s；
     *       加速度：硬件配的是 1600Hz 采样，但 Calculate() 按 4ms 一桶
     *       节流，真正进 VQF 的观测是 ~250Hz → 0.004 s。
     *       写错的表现：滤波器系数和零偏估计的协方差全错，角度发飘。
     * ══════════════════════════════════════════════════════════════════ */
    Struct_BMI088_VQF_Config vqf_config;
    vqf_config.Gyro_D_T = 0.0005f;
    vqf_config.Accel_D_T = 0.004f;

    /* 加速度重力修正的时间常数：越大越平滑，但收敛越慢 */
    vqf_config.Parameter.Tau_Accel = 3.0f;

    /* 两条零偏估计路径都开：运动时缓慢跟踪，静止确认后提高可信度 */
    vqf_config.Parameter.Motion_Bias_Estimation_Enable = true;
    vqf_config.Parameter.Rest_Bias_Estimation_Enable = true;

    /* 初始零偏不确定度：越大，启动阶段允许零偏调整得越快 */
    vqf_config.Parameter.Bias_Sigma_Init_Deg_S = 0.5f;

    /* 零偏遗忘时间：越长长期估计越稳，但温漂后重新收敛越慢 */
    vqf_config.Parameter.Bias_Forgetting_Time = 100.0f;

    /* 零偏和残差的限幅：防止把真实转动误当成零偏 */
    vqf_config.Parameter.Bias_Clip_Deg_S = 2.0f;

    /* 运动状态的零偏观测噪声 */
    vqf_config.Parameter.Bias_Sigma_Motion_Deg_S = 0.1f;

    /* ★ 垂直方向（偏航）的遗忘因子：越小越保守。
     *   0.0001 让该方向的观测噪声放大一万倍 = "基本不信，但也不完全放弃" */
    vqf_config.Parameter.Bias_Vertical_Forgetting_Factor = 0.0001f;

    /* 静止状态的零偏观测噪声：比运动值小，所以静止时更可信 */
    vqf_config.Parameter.Bias_Sigma_Rest_Deg_S = 0.03f;

    /* 连续满足静止条件 1.5 秒才判定为静止 */
    vqf_config.Parameter.Rest_Min_Time = 1.5f;

    /* 静止检测的低通时间常数：抑制瞬时振动导致的反复切换 */
    vqf_config.Parameter.Rest_Filter_Tau = 0.5f;

    /* 静止门限：陀螺高频残差 3.5°/s、加速度残差 0.5 m/s² */
    vqf_config.Parameter.Rest_Threshold_Gyro_Deg_S = 3.5f;
    vqf_config.Parameter.Rest_Threshold_Accel = 0.5f;

    BSP_BMI088.Set_VQF_Config(vqf_config);

    /* ══════════════════════════════════════════════════════════════════
     * ③ 初始化 BMI088（加速度计 → 陀螺 → VQF）
     *    ⚠️ 每一步都带读回校验、失败重试 5 次，所以是【阻塞】的、可能几秒。
     *       失败时不打开内部的有效标志：姿态不会更新（而不是拿旧数据装正常）。
     * ══════════════════════════════════════════════════════════════════ */
    if (!BSP_BMI088.Init())
    {
        /* 初始化失败：内部有效标志不打开，姿态不会更新。
         * 串口上只发波形、没有日志可打 —— 现象就是波形里的角度一直是 0 不动。
         * 真要查，用 BSP_BMI088.Is_Initialized()，或者临时加一个波形通道。 */
        return;
    }

    /* ④ 初始化期间传感器已经在往 FIFO 里灌数据了：重写一次 FIFO 配置冲掉它们，
     *    否则第一批样本是"配置过程的残留"，时间戳也对不上 */
    BSP_BMI088.BMI088_Gyro.Start_FIFO_Acquisition();

    /* ══════════════════════════════════════════════════════════════════
     * ⑤ 零偏初值：优先 Flash 里存的离线标定，没有才退回开机静止标定
     *
     *   为什么需要零偏初值：
     *     偏航轴的零偏在运动时几乎不可观测（垂直方向权重只有水平方向的
     *     1/10000），VQF 只能等静止期慢慢收 —— 表现就是【上电后 yaw 漂移】。
     *     给一个好的初值，在线估计只需要跟温漂的残余。
     *
     *   来源一（首选）：Flash 里存过的标定 —— 由按键触发的板载 30 秒标定
     *     写入（见本文件的标定状态机）。统计时长 30 秒、写入前有静止体检，
     *     存的是 40°C 稳态工况下的值（恒温的意义所在）。
     *     开机直接加载，【无需静止等待】。
     *   来源二（兜底）：首次使用 / Flash 校验失败时，开机静止采 1 秒取平均。
     *     ⚠️ 只有走这条路的那一次需要静止放好 —— 晃动着开机，标出来的
     *        就是"运动速度"，喂进去反而更糟。
     * ══════════════════════════════════════════════════════════════════ */
    {
        float stored_bias[3];
        float stored_temperature;

        if (BSP_Flash_Calibration_Load(stored_bias, &stored_temperature))
        {
            BSP_BMI088.Set_VQF_Bias_Estimate(Class_Matrix_f32<3, 1>(stored_bias));
            Attitude_Bias_Source = 2U;
        }
        else
        {
            const uint32_t sample_count = 2000U;          /* 2kHz 陀螺 ≈ 1 秒 */
            float bias_sum[3] = {0.0f, 0.0f, 0.0f};
            for (uint32_t i = 0U; i < sample_count; i++)
            {
                const Class_Matrix_f32<3, 1> gyro = BSP_BMI088.BMI088_Gyro.Get_Raw_Gyro();
                bias_sum[0] += gyro[0][0];
                bias_sum[1] += gyro[1][0];
                bias_sum[2] += gyro[2][0];
                Sys_Delay_S(0.0005f);                      /* 500µs，对齐陀螺的 2kHz 采样 */
            }

            const float bias_average[3] = {
                bias_sum[0] / (float)sample_count,
                bias_sum[1] / (float)sample_count,
                bias_sum[2] / (float)sample_count};
            BSP_BMI088.Set_VQF_Bias_Estimate(Class_Matrix_f32<3, 1>(bias_average));
            Attitude_Bias_Source = 1U;
        }
    }
}

void Attitude_Task(void)
{
    /* ── 从设备层取【整帧】结果 ──
     * 一次原子复制，保证四元数 / 欧拉角 / 原始量 / 零偏来自同一帧；
     * 不再逐个 getter 拼数据（那会拿到"半帧"，见结构体注释）。 */
    Struct_BMI088_Attitude_Frame frame;
    const bool valid = BSP_BMI088.Get_Attitude_Frame(frame) &&
                       Attitude_Sample_Is_Fresh(frame.Sample_Time_Us);

    /* 以上一帧为底：无效时只把 Valid 拉低、让 Sample_Time_Us / Sequence 停住，
     * 数值保持上一帧 —— 波形上看到的是"冻住的最后一帧"，比跳回 0 好排查。 */
    Struct_Attitude next = Attitude;

    if (valid)
    {
        /* ── 四元数 ── */
        next.q[0] = frame.Quaternion[0];
        next.q[1] = frame.Quaternion[1];
        next.q[2] = frame.Quaternion[2];
        next.q[3] = frame.Quaternion[3];

        /* ── 欧拉角 ──
         * ★ 顺序是 [Yaw, Pitch, Roll]（标准 ZYX 约定），单位【弧度】，乘 180/π 转度。
         *   别信名字，用数学验：绕 X 轴转 30° 的四元数 (cos15°, sin15°, 0, 0) 代入库的
         *   公式，只有 result[2] 得到 30°，而绕 X 轴转就是 Roll。 */
        next.Yaw   = frame.Euler_Angle[0][0] * RAD_2_DEG;
        next.Pitch = frame.Euler_Angle[1][0] * RAD_2_DEG;
        next.Roll  = frame.Euler_Angle[2][0] * RAD_2_DEG;

        /* ── VQF 独有的两个诊断量 ── */
        next.GyroBias[0] = frame.Gyro_Bias[0][0];
        next.GyroBias[1] = frame.Gyro_Bias[1][0];
        next.GyroBias[2] = frame.Gyro_Bias[2][0];
        next.Rest_Detected = frame.Rest_Detected;

        /* ── 原始数据（未扣零偏，机体系）── */
        next.Gyro[0] = frame.Gyro[0][0];
        next.Gyro[1] = frame.Gyro[1][0];
        next.Gyro[2] = frame.Gyro[2][0];
        next.Accel[0] = frame.Accel[0][0];
        next.Accel[1] = frame.Accel[1][0];
        next.Accel[2] = frame.Accel[2][0];

        next.Sample_Time_Us = frame.Sample_Time_Us;
        next.Sequence       = frame.Sequence;
    }
    next.Valid = valid ? 1U : 0U;

    /* ── 一次提交 ──
     * 读者（将来的控制环 / 上位机）走 Attitude_Get_Snapshot() 整帧取走，
     * 不会读到"改了一半"的结构体。 */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Attitude = next;
    __DMB();
    __set_PRIMASK(primask);
}

bool Attitude_Get_Snapshot(Struct_Attitude *Out)
{
    if (Out == NULL) { return false; }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *Out = Attitude;
    __DMB();
    __set_PRIMASK(primask);

    Out->Valid = (Out->Valid != 0U &&
                  Attitude_Sample_Is_Fresh(Out->Sample_Time_Us)) ? 1U : 0U;
    return Out->Valid != 0U;
}
