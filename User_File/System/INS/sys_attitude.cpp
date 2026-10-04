/**
 * @file    sys_attitude.cpp
 * @brief   姿态解算对外接口实现（接 H7_BSP 那套 BMI088 + VQF）
 *
 * @note    它只做三件事：
 *            ① 把 SPI 层和 BMI088 那套接起来（初始化）
 *            ② 配置 VQF 参数
 *            ③ 每毫秒把结果搬进对外的 Attitude 结构体
 *          真正的解算在 BMI088_Task 里（见 Task/BMI088_Task.cpp）。
 */

#include "sys_attitude.h"

#include "bsp_spi.h"        /* SPI_Init */
#include "bsp_bmi088.h"     /* BSP_BMI088 */
#include "callback.h"       /* SPI1_Callback */
#include "spi.h"            /* hspi1 —— BMI088 挂在这条 SPI 上 */

#define RAD_2_DEG   57.29577951f

Struct_Attitude Attitude;

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
     * ⑤ 开机零偏标定：静止采 1 秒陀螺，把平均值当作 VQF 的零偏初值
     *
     *   为什么需要这一步：
     *     上游那套依赖"板级标定常量"（他们那块板子的实测值），换一块板子就不能用，
     *     所以陀螺的原始零偏只能靠 VQF 在线估计。而偏航轴的零偏在运动时几乎不可
     *     观测（垂直方向权重只有水平方向的 1/10000），只能等静止期慢慢收 —— 表现
     *     就是【上电后 yaw 以 0.2~0.3°/s 漂一两分钟】。
     *     这里把开机这 1 秒的平均值先喂进去，在线估计只需要跟温漂的残余，
     *     上电漂移立刻降一个量级（旧驱动那 6000 点标定起的就是这个作用）。
     *
     *   ⚠️ 采样这一秒里板子必须静止放好 —— 否则标定出来的就是"运动速度"，
     *      喂进去反而更糟。这是旧驱动一直以来的要求，别在晃动的桌面上开机。
     * ══════════════════════════════════════════════════════════════════ */
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
        /* 标定结果不打印（串口只发波形）。想确认它标了多少：
         * 波形里 yaw 上电后的漂移速度就反映了残余；或者临时加一个通道
         * 显示 Attitude.GyroBias[2]*1000（那是 VQF 收敛后的零偏）。 */
    }
}

void Attitude_Task(void)
{
    /* ── 从设备层取【整帧】结果 ──
     * 一次原子复制，保证四元数 / 欧拉角 / 原始量 / 零偏来自同一帧；
     * 不再逐个 getter 拼数据（那会拿到"半帧"，见结构体注释）。 */
    Struct_BMI088_Attitude_Frame frame;
    const bool valid = BSP_BMI088.Get_Attitude_Frame(frame);

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

    return Out->Valid != 0U;
}
