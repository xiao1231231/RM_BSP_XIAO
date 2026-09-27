//
// Created by xiao on 2026/9/24.
//

#ifndef LED_H_
#define LED_H_

#ifdef __cplusplus
extern "C" {
#endif

/**@brief 初始化：三路全灭，在MX_GPIO_Init()之后调用*/
void LED_Init(void);

/**@brief 三个灯都灭*/
void LED_Off(void);

/**@brief 亮红灯*/
void LED_Red(void);

/**@brief 亮绿灯*/
void LED_Green(void);

/**@brief 亮蓝灯*/
void LED_Blue(void);

#ifdef __cplusplus
}
#endif

#endif