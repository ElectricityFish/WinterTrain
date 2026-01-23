#include "zf_common_headfile.h"

#define ENCODER_QUADDEC1                (TIM3_ENCODER)                          // 正交编码器对应使用的编码器接口 这里使用 TIM3 的编码器功能
#define ENCODER_QUADDEC_A1              (TIM3_ENCODER_CH1_B4)                   // A 相对应的引脚
#define ENCODER_QUADDEC_B1              (TIM3_ENCODER_CH2_B5)                   // B 相对应的引脚
 
#define ENCODER_QUADDEC2		        (TIM4_ENCODER)                          
#define ENCODER_QUADDEC_A2              (TIM4_ENCODER_CH1_B6)                   
#define ENCODER_QUADDEC_B2              (TIM4_ENCODER_CH2_B7)                   



void Encoder_Init(void)
{
	encoder_quad_init(ENCODER_QUADDEC1, ENCODER_QUADDEC_A1, ENCODER_QUADDEC_B1);   // 初始化编码器模块与引脚 正交解码编码器模式
	encoder_quad_init(ENCODER_QUADDEC2, ENCODER_QUADDEC_A2, ENCODER_QUADDEC_B2);   // 初始化编码器模块与引脚 正交解码编码器模式

	encoder_clear_count(ENCODER_QUADDEC1);
	encoder_clear_count(ENCODER_QUADDEC2);
	

}

//encoder_get_count(ENCODER_QUADDEC);
int16 Get_Count(uint8_t n)															//参数表示要查看哪一个编码器的（只能是1或2），调用函数后计数自动清0
{
	if(n==1){
		encoder_clear_count(ENCODER_QUADDEC1);
		return encoder_get_count(ENCODER_QUADDEC1);
	}
	
	if(n==2){
		encoder_clear_count(ENCODER_QUADDEC2);
		return encoder_get_count(ENCODER_QUADDEC2);
	}
	
	
	encoder_clear_count(ENCODER_QUADDEC1);
	encoder_clear_count(ENCODER_QUADDEC2);
	return 0;
}

