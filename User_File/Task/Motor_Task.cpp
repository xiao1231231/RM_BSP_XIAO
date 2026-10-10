#include "cmsis_os2.h"
#include "dji_motor.h"
#include "sys_timestamp.h"

// 调试器观察：累计错过的节拍数、相邻两次控制完成的间隔（秒）
volatile uint32_t motor_task_overrun_count = 0;
volatile float motor_task_period_s = 0.0f;

extern "C" void Motor_Task(void *argument)
{
    (void)argument;

    uint32_t wake = osKernelGetTickCount();
    uint32_t last_cycle = Sys_Get_Cycle();

    // 获取已经创建好的4个3508电机，motor[0~3] 是指向它们的指针
    Class_DJI_Motor *motors[4] = {
        DJI_Motor_Get(DJI_MOTOR_1),
        DJI_Motor_Get(DJI_MOTOR_2),
        DJI_Motor_Get(DJI_MOTOR_3),
        DJI_Motor_Get(DJI_MOTOR_4)
    };
    //定义target_rpm数组来存储四个电机目标转速
    float target_rpm[4] = {0,0,0,0};

    for (;;)
    {

        //target_rpm[0],target_rpm[1],target_rpm[2],target_rpm[3]分别对应id为1~4的3508电机的转速
        // 先运行你的控制算法，把四个计算结果写入 target_rpm。
        // target_rpm[0] 对应 1 号电机，依次类推。
        // 单位为输出轴 rpm
        for (uint8_t i = 0; i < 4U; ++i)
        {
            if (motors[i] != nullptr)// 找到了这个对象，才设置目标转速
            {
                motors[i]->Set_Target_Speed_Rpm(target_rpm[i]);
            }
        }
        // 使用刚更新的目标，计算速度闭环并发送电流指令
        DJI_Motor_Control_Task();




        motor_task_period_s = Sys_Get_DeltaTime(&last_cycle);
        // 推进到下一个 1ms 节拍
        wake += 1U;
        const uint32_t now_tick = osKernelGetTickCount();

        // 已错过唤醒时刻：记录丢拍，重新定位到未来节拍
        if ((int32_t)(now_tick - wake) >= 0)
        {
            motor_task_overrun_count += (now_tick - wake) + 1U;
            wake = now_tick + 1U;
        }

        osDelayUntil(wake);
    }
}
