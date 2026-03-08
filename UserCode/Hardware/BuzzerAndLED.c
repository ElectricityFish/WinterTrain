#include "zf_common_headfile.h"

void BuzzerAndLED_Init(void)
{
	gpio_init(B1, GPO, GPIO_LOW, GPO_PUSH_PULL);	// 蜂鸣器引脚初始化
	gpio_init(B2, GPO, GPIO_LOW, GPO_PUSH_PULL);	// LED引脚初始化
}  

void Promopt(void)
{
	 gpio_set_level (B1, 1);	
	 gpio_set_level (B2, 1);
}

void StopPromopt(void)
{
	gpio_set_level (B1, 0);	
	gpio_set_level (B2, 0);
}
