#include "led.h"
#include "main.h"       /* LED_R/G/B_Pin —— CubeMX 标签，引脚的唯一真相源 */

/* 极性：SET = 亮，RESET = 灭（CubeMX 里配置 GPIO 输出低 = 熄灭起始态）。
 * 引脚分配（main.h 标签）：LED_B = PH10，LED_G = PH11，LED_R = PH12。
 * ★ 不要在这里手写 GPIO_PIN_xx —— CubeMX 重分配引脚后标签会跟着变，
 *   手写的魔数不会，结果是"编译照过、灯色错乱"。 */

void LED_Init(void)
{
    LED_Off();
}

void LED_Off(void)
{
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
}

void LED_Blue(void)
{
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
}

void LED_Red(void)
{
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
}

void LED_Green(void)
{
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
}
