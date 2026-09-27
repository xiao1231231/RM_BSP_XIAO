/**
 * @file    TIM_1ms_Task.cpp
 * @brief   1ms 周期任务 —— 整个工程的"心跳"
 * @note    任务的创建（osThreadNew）由 CubeMX 生成的 Core/Src/freertos.c 负责，
 *          本文件只提供它的【实现】—— 覆盖 freertos.c 里那个 __weak 空壳。
 */
#include "cmsis_os2.h"
#include "main.h"
#include "usart.h"
#include "sys_debug.h"
#include "sys_timestamp.h"
#include "bsp_uart.h"
#include "led.h"
#include "sys_attitude.h"   /* Attitude_Task() / Attitude */
#include "bsp_bmi088.h"     /* BMI088 的 1ms / 128ms 服务回调 */

/**
 * @brief 累计【丢失的节拍数】
 * @note  不为 0 说明有节拍没在 1ms 内跑完 —— 超时一次，把丢掉的那几拍【一起】累加进来，
 *        所以它读出来是"总共丢了多少拍"，不只是"超时过几次"。
 *        它不会让功能停摆（灯照样闪），所以必须【主动看】这个变量，
 *        否则你不知道控制周期已经不准了。
 */
volatile uint32_t task_overrun_count = 0;

/** 任务的真实周期（秒），供调试器观察 */
volatile float task_period_s = 0.0f;

/**
 * @brief 1ms 周期任务
 * 心跳灯
 *判断程序是否正常跑起来
 */
extern "C" void TIM_1ms_Task(void *argument)
{
    /* 绝对节拍网格：wake 记录"下一拍该在第几个 tick 醒来"。
    * 用 osDelayUntil(wake) 而不是 osDelay(1)：
    *     osDelay(1)         唤醒时刻 = 上一拍 + 执行时间 + 1ms  → 误差累积
    *     osDelayUntil(wake) 唤醒时刻 = 固定的整数 tick 边界    → 周期恒定 1.000ms */
    uint32_t wake = osKernelGetTickCount();
    uint32_t blink_div = 0;
    uint32_t attitude_div = 0;    /** 128ms 分频（温度读取） */
    bool led_mode = false; //绿灯现在的亮灭

    static uint32_t last_cycle = 0;   /** DWT 时间戳（本函数私有，跨循环保持） */


    for (;;)
    {
        /* 用 DWT 测量"上一拍到这一拍"的真实时长 */
        task_period_s = Sys_Get_DeltaTime(&last_cycle);

        /* ★ 姿态解算相关：两个服务周期 + 一次搬运
         *
         *   BMI088_TIM_1ms_Service...()   1kHz：兜底轮询（陀螺 INT 万一丢了就自己
         *                                 发起一读）+ 传输调度 + SPI 超时/恢复检查
         *   BMI088_TIM_128ms_Calculate... 128Hz÷... 128ms：读一次温度（慢变量）
         *   Attitude_Task()               1kHz：把姿态结果搬进 Attitude 结构体
         *
         *   ★ 真正的解算【不在这里】：陀螺样本入队后由 BMI088_Task 逐帧跑 VQF
         *     （2kHz）。这里只是"服务 + 搬运"，耗时都是微秒级。
         *
         *   放在跳拍保护之前 —— 它们耗时多少，下面的 overrun 会如实记下来。 */
        BMI088_TIM_1ms_Service_PeriodElapsedCallback();

        if (++attitude_div >= 128U)
        {
            attitude_div = 0;
            BMI088_TIM_128ms_Calculate_PeriodElapsedCallback();
        }

        Attitude_Task();

        //串口接收看门狗，每 1ms 检查一次"接收有没有停"，停了就重启，放在 1ms 任务里而不是收帧回调里，因为它是"兜底"逻辑，只在出错后才起作用，不需要实时响应
        BSP_UART_Recover_PeriodElapsedCallback();

        //绿灯闪烁：1ms x 500 = 500ms闪烁一次
        if (++blink_div >= 500)
        {
            blink_div = 0;
            led_mode = !led_mode;
            if (led_mode){LED_Green();}
            else{LED_Off();}
        }

        /* ── 关于 DWT 微秒时钟（Sys_Get_Micros）─────────────────────
         *  它是"累加"实现，要求调用间隔 < 25.57s（32 位 CYCCNT 一回绕）。
         *  现在【不必】专门保活：IMU 链路一活跃，BMI088 驱动每秒会调它几千次
         *  （样本时间戳、传输超时判定），天然的调用者足够密。
         *  ⚠️ 但将来若把 IMU 停掉、而又有别人用它取绝对时间，记得加回定期调用。 */

        /* ── VOFA+ 波形输出：三轴姿态 @ 50Hz ─────────────────────────
         *
         *   只发三个欧拉角 —— 一行就是 "roll,pitch,yaw"，看图不会认错：
         *     ch0 = roll    横滚角（度）
         *     ch1 = pitch   俯仰角（度）
         *     ch2 = yaw     偏航角（度）
         *   （通道名在 VOFA+ 的通道列表里配，配一次就够）
         *
         * ★ FireWater 的真实格式（非常容易写错）：
         *        任意前缀:数值0,数值1,...\r\n
         *                 ↑ 冒号右边才是数据，逗号分隔
         *   冒号左边是前缀文本。★ 不能逐通道写名字 —— "roll:..,pitch:.." 是错的：
         *   VOFA+ 只认第一个冒号，会把 pitch: 那串当非法数值丢掉，只剩 1 个通道。
         *
         * ★ 启动时日志已经自动关闭（见 System_Init 末尾），所以这里发出去
         *   的就是干净的数据流。
         *
         * ── 需要排查 IMU 时（本工程就是这么定位到 SPI 恢复自持循环的）──
         *   下面这套诊断量随时可以加回来，一行一个，它们都在 BSP_BMI088 里：
         *     Get_Calculating_Time()              单次解算耗时 μs（应 < 350）
         *     BMI088_Gyro.Get_Queue_Depth()       样本队列深度（应接近 0）
         *     BMI088_Gyro.Get_Queue_Drop_Count()  丢样本数（应恒 0）
         *     BMI088_Gyro.Get_FIFO_Interrupt_Count()  中断数（斜率应 2000/秒）
         *     Get_SPI_Recovery_Counter()          重量级恢复次数（应恒 0）
         *     Get_SPI_Recovery_Last_Reason()      最近一次恢复原因（位掩码）
         *     task_overrun_count                  1ms 任务漏拍（应恒 0）
         *
         * 带宽核算（115200 波特 = 11520 字节/秒）：
         *   一帧 3 通道 ≈ 30 字节，50Hz ≈ 1500 B/s ≈ 13% —— 余量很大。
         *   想让曲线更顺滑，可以把下面的阈值 20 改成 10（100Hz，也才 26%）。 */
        static uint32_t wave_div = 0;
        if (++wave_div >= 20U)      /* 20 × 1ms = 50Hz */
        {
            wave_div = 0;
            Wave_Output("imu:%.2f,%.2f,%.2f",
                        (double)Attitude.Roll, (double)Attitude.Pitch,
                        (double)Attitude.Yaw);
        }

        /* ── 节拍推进 + 跳拍保护 ──
         * ★ 必须放在【这一拍所有工作之后】——
         *   日志和波形（浮点格式化的耗时大头）还没跑就判断，等于没覆盖完整负载。
         *
         * ① wake += 1：目标时刻推进 1ms
         * ② 用 (int32_t)(现在 - 目标) >= 0 判断，不能写 (现在 >= 目标)：
         *    tick 会回绕，直接比大小在回绕时会判错。
         *    ★ 用 >= 而不是 >：正好卡在目标时刻也算超时 —— 因为
         *      osDelayUntil(目标) 在目标已到时【不会阻塞】，会直接连着执行下一拍。
         * ③ 超时不追赶（补跑会让好几拍挤在一起，比丢一拍更糟），
         *    把网格重新锚定到【下一个未来节拍】(now + 1)，
         *    保证这一拍的延时一定真正生效。
         * ④ task_overrun_count 的语义：累计【丢失的节拍数】（不是"超时次数"）*/
        wake += 1U;
        const uint32_t now_tick = osKernelGetTickCount();
        if ((int32_t)(now_tick - wake) >= 0)
        {
            task_overrun_count += (now_tick - wake) + 1U;
            wake = now_tick + 1U;
        }

        osDelayUntil(wake);
    }
}