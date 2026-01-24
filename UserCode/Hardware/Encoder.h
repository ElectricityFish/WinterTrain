#include "zf_common_headfile.h"
#ifndef __ENCODER_H
#define __ENCODER_H
void Encoder_Init(void);
int16_t Get_Count1(void);
int16_t Get_Count2(void);
void Encoder_Clear(void);
/*
接线说明
B4,B5 对应编码器1
B6,B7 对应编码器2
*/

#endif
