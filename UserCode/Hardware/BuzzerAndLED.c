#include "zf_common_headfile.h"


void BuzzerAndLED_Init(void)
{
	gpio_init(D7, GPO, GPIO_LOW, GPO_PUSH_PULL);//蜂鸣器引脚初始化
	gpio_init(B13, GPO, GPIO_LOW, GPO_PUSH_PULL);//LED引脚初始化
	
}  

void Promopt(uint8_t Mode)
{
	if(Mode==1)gpio_set_level (D7, 1);//模式1蜂鸣器响
	if(Mode==2)gpio_set_level (B13, 0);//模式2LED亮
	if(Mode==3)gpio_set_level (D7, 0);//模式3蜂鸣器关
	if(Mode==4)gpio_set_level (B13, 1);//模式4LED关
}


