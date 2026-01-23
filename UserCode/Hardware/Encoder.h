#include "zf_common_headfile.h"
#ifndef __ENCODER_H
#define __ENCODER_H
void Encoder_Init(void);
int16 Get_Count(uint8_t n);	//参数表示要查看哪一个编码器的（只能是1或2），调用函数后计数自动清0

/*
接线说明
B4,B5 对应编码器1
B6,B7 对应编码器2
*/

#endif
