#include "buzzer.h"

#include "tim.h"
#include "sys_timestamp.h"

//定时器计数时钟
#define BUZZER_TIMER_CLK_HZ    1000000U

//频率限幅
#define BUZZER_FREQ_MIN_HZ     100.0f
#define BUZZER_FREQ_MAX_HZ     10000.0f

void Buzzer_Init(void)
{
    Buzzer_Off();
}

void Buzzer_Off(void)
{
    HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
}

void Buzzer_On(float freq_hz)
{
    //限幅
    if (freq_hz < BUZZER_FREQ_MIN_HZ) { freq_hz = BUZZER_FREQ_MIN_HZ; }
    if (freq_hz > BUZZER_FREQ_MAX_HZ) { freq_hz = BUZZER_FREQ_MAX_HZ; }

    //f = 1MHz / (ARR+1) → ARR = 1MHz / f − 1 + 0.5f 是四舍五入 —— 直接截断在高音区误差会偏大
    const uint32_t arr = (uint32_t)((float)BUZZER_TIMER_CLK_HZ / freq_hz + 0.5f) - 1U;

    //50% 占空比：方波的平均功率最大，声音最响
    const uint32_t ccr = (arr + 1U) / 2U;

    __HAL_TIM_SET_AUTORELOAD(&htim4, arr);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, ccr);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

void Buzzer_Beep(float freq_hz,float seconds)
{
    Buzzer_On(freq_hz);
    Sys_Delay_S(seconds);
    Buzzer_Off();
}