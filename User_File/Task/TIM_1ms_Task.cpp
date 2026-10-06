/**
 * @file    TIM_1ms_Task.cpp
 * @brief   1ms 周期任务 —— 工程的心跳
 * @note    由 CubeMX 生成的 freertos.c 创建（osThreadNew），本文件提供实现，
 *          覆盖那里的 __weak 空壳。
 */
#include "cmsis_os2.h"
#include "main.h"
#include "can.h"
#include "sys_debug.h"
#include "sys_timestamp.h"
#include "bsp_uart.h"
#include "led.h"
#include "key.h"
#include "sys_attitude.h"
#include "bsp_bmi088.h"
#include "dji_motor.h"

/** 累计丢失的节拍数。恒为 0 才说明 1ms 周期真的守住了 */
volatile uint32_t task_overrun_count = 0;

/** 上一拍到这一拍的真实周期（秒），供调试器观察 */
volatile float task_period_s = 0.0f;

extern "C" void TIM_1ms_Task(void *argument)
{
    /* 绝对节拍网格：osDelayUntil(wake) 周期恒定 1ms；
     * osDelay(1) 的唤醒时刻会累积执行时间误差 */
    uint32_t wake = osKernelGetTickCount();
    uint32_t blink_div = 0;
    uint32_t heat_div = 0;
    uint32_t attitude_div = 0;
    uint32_t wave_div = 0;
    bool led_mode = false;

    static uint32_t last_cycle = 0;

    for (;;)
    {
        /* 时间基保活：Sys_Get_Micros 的 64 位累加依赖"至少每 25.57s 被调一次"
         * （CYCCNT 回绕周期）。它的其它调用点全在 IMU/串口的回调链里 ——
         * IMU 初始化失败时那些路径全部不可达，只有这里是无条件执行的。 */
        (void)Sys_Get_Micros();

        task_period_s = Sys_Get_DeltaTime(&last_cycle);

        /* IMU：1ms 服务（兜底轮询 + 传输调度 + 超时恢复）、
         * 500Hz 恒温 + 128ms 温度读取、然后搬运姿态结果 */
        BMI088_TIM_1ms_Service_PeriodElapsedCallback();

        /* 恒温 500Hz（C 板官方工程同款周期） */
        if (++heat_div >= 2U)
        {
            heat_div = 0;
            BSP_BMI088.Heater_Control();
        }

        if (++attitude_div >= 128U)
        {
            attitude_div = 0;
            BMI088_TIM_128ms_Calculate_PeriodElapsedCallback();
        }

        Attitude_Task();

        /* 按键扫描（长按 4 秒 = 请求零偏标定）+ 标定状态机
         * （采样中蓝灯闪；写 Flash 时全机冻结 1~2 秒；完成红灯快闪 4 次） */
        Key_Service();
        if (Key_Get_LongPress())
        {
            Attitude_Calibration_Request();
        }
        Attitude_Calibration_Service();

        /* 串口接收看门狗：兜底逻辑，只在收停止后才起作用 */
        BSP_UART_Recover_PeriodElapsedCallback();

        /* CAN 总线巡检：bus-off / 总线错误计数（只观察，不做动作） */
        BSP_CAN_Service_PeriodElapsedCallback(&hcan1);

        /* DJI 电机：速度环 + 分组发送，1kHz */
        DJI_Motor_Control_Task();

        /* 绿灯 500ms 闪烁，用于观察程序是否正常运行
         * （零偏标定状态机占用 LED 期间让位 —— 它在闪蓝/红） */
        if (!Attitude_Calibration_Active())
        {
            if (++blink_div >= 500)
            {
                blink_div = 0;
                led_mode = !led_mode;
                if (led_mode){LED_Green();}
                else{LED_Off();}
            }
        }

        /* 三轴姿态，50Hz，走 USB 虚拟串口（VOFA+ FireWater 格式）
         * 通道：roll, pitch, yaw */
        //待打包成弧度制去调参
        if (++wave_div >= 20U)
        {
            wave_div = 0;
            USB_Printf("imu:%.2f,%.2f,%.2f",
                        (double)Attitude.Roll, (double)Attitude.Pitch,
                        (double)Attitude.Yaw);
        }

        /* 节拍推进 + 跳拍保护：必须在所有工作之后。
         * (int32_t) 差值比较是为了不受 tick 回绕影响；
         * 超时不追赶，把网格重新锚定到下一个未来节拍 */
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
