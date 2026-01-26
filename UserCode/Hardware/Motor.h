#include "zf_common_headfile.h"
#ifndef __MOTOR_H
#define __MOTOR_H
void Motor_Init(void);
void Motor_SetPWM(uint8 CH,int16_t PWM);										//第一个参数表示初始化哪一个通道，1或2，第二个参数表示PWM值,最大为10000

/************************
*接线说明
*A0,A1对应通道1和2PWM
*A2,A3对应通道1逻辑电源
*B10,B11对应通道2逻辑电源
*************************/
#endif
