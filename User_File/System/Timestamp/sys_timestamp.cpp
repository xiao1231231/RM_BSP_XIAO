#include "sys_timestamp.h"

/* ── 绝对时间累加状态（只有 Sys_Get_Micros 用）──
 * static 表示"只在本文件可见"，外面无法直接访问这两个变量。 */
static uint64_t s_total_cycles = 0;   /* 64 位累计周期数 */
static uint32_t s_last_cycle   = 0;

void Sys_Timestamp_Init(void)
{
    /* ★ Cortex-M4：三行搞定
     *   注意这里【不需要】DWT->LAR = 0xC5ACCE55 那行解锁 ——
     *   那是 Cortex-M7 才有的"锁访问寄存器"。F4 的头文件里根本没有 LAR 成员，
     *   照抄 H7 的代码会直接编译报错。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* ① 打开跟踪总开关 */
    DWT->CYCCNT = 0;                                   /* ② 计数器清零 */
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;              /* ③ 启动计数 */

    s_total_cycles = 0;
    s_last_cycle   = 0;
}

float Sys_Get_DeltaTime(uint32_t *last_cycle)
{
    uint32_t now = DWT->CYCCNT;

    /* 无符号减法：即使 now 回绕变小，差值依然正确。
     * 前提：两次调用间隔 < 2^32 / 168MHz = 25.57 秒 */
    uint32_t delta = now - *last_cycle;
    *last_cycle = now;

    return (float)delta / (float)(SYS_CPU_FREQ_MHZ * 1000000U);
}

uint64_t Sys_Get_Micros(void)
{
    uint32_t now;
    uint64_t snapshot;

    const uint32_t primask = __get_PRIMASK();   /* 记下进临界区前的中断状态 */
    __disable_irq();

    now = DWT->CYCCNT;
    s_total_cycles += (uint32_t)(now - s_last_cycle);
    s_last_cycle = now;
    snapshot = s_total_cycles;                  /* 64 位读也要在临界区里 */

    __set_PRIMASK(primask);                     /* 原样恢复，可嵌套 */

    /* cycles → us：168MHz 下 168 个周期正好 1us，直接除 168
 * （乘法/除法仍然在临界区外面） */
    return snapshot / SYS_CPU_FREQ_MHZ;
}

void Sys_Delay_US(float us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (uint32_t)(us * (float)SYS_CPU_FREQ_MHZ);

    /* 同样用无符号差值比较，不受回绕影响 */
    while ((DWT->CYCCNT - start) < ticks)
    {
        /* busy wait —— 忙等 */
    }
}

void Sys_Delay_S(float s)
{
    Sys_Delay_US(s * 1000000.0f);
}