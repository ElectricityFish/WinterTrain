#include "zf_common_headfile.h"

#define PWM_CH1                 (TIM5_PWM_CH1_A0)
#define PWM_CH2                 (TIM5_PWM_CH2_A1)


void Motor_Init(void)
{
	//逻辑电源初始化
	
	//CH1对应
	gpio_init(A2, GPO, GPIO_LOW, GPO_PUSH_PULL);                             // 初始化输出 默认低电平 推挽输出模式
    gpio_init(A3, GPO, GPIO_LOW, GPO_PUSH_PULL);                             // 初始化输出 默认低电平 推挽输出模式
	
	//CH2对应
	gpio_init(B10, GPO, GPIO_LOW, GPO_PUSH_PULL);                             // 初始化输出 默认低电平 推挽输出模式
    gpio_init(B11, GPO, GPIO_LOW, GPO_PUSH_PULL);                             // 初始化输出 默认低电平 推挽输出模式
	
	
	pwm_init(PWM_CH1, 17000, 0);                                                // 初始化 PWM 通道 频率 17KHz 初始占空比 0%
    pwm_init(PWM_CH2, 17000, 0);                                                // 初始化 PWM 通道 频率 17KHz 初始占空比 0%

}	

void Moto_SetPWM(uint8 CH,int16_t PWM)											//第一个参数表示初始化哪一个通道，1或2，第二个参数表示PWM值,最大为10000
{
	if(CH==1){
		if(PWM>=0)
		{
			gpio_set_level (A2, 1);
			gpio_set_level (A3, 0);
			pwm_set_duty(PWM_CH1, PWM);
		}else{
				gpio_set_level (A2, 0);
				gpio_set_level (A3, 1);
				pwm_set_duty(PWM_CH1,-PWM);
		}
	}
	
	if(CH==2){
		if(PWM>=0)
		{
			gpio_set_level (B10, 1);
			gpio_set_level (B11, 0);
			pwm_set_duty(PWM_CH2, PWM);
		}else{
			gpio_set_level (B10, 0);
			gpio_set_level (B11, 1);
			pwm_set_duty(PWM_CH2,-PWM);
		}
		
	}
	
}

