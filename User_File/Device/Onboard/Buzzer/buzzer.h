//板载无源蜂鸣器
#ifndef BUZZER_H_
#define BUZZER_H_

#ifdef __cplusplus
extern "C" {
#endif

//初始化：上电不响
void Buzzer_Init(void);

//以指定频率开始发声
void Buzzer_On(float freq_hz);

//停止
void Buzzer_Off(void);

//响一声就停，阻塞式，用于初始化
void Buzzer_Beep(float freq_hz,float seconds);

#ifdef __cplusplus
}
#endif

#endif