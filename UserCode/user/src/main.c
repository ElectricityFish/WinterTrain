#include "zf_common_headfile.h"
#include "Motor.h"
#include "Encoder.h"
#include "Sensor.h"
#include "BuzzerAndLED.h"

#define PIT                             (TIM6_PIT )                             // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_PRIORITY                    (TIM6_IRQn)                             // 对应周期中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体


int16_t Speed1;
int16_t Speed2;
int main (void)
{
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 Debug UART
	
	BuzzerAndLED_Init();
	pit_ms_init(PIT, 100);                                                      // 初始化 PIT 为周期中断 100ms 周期
    //interrupt_set_priority(PIT_PRIORITY, 0);                                    // 设置 PIT 对周期中断的中断优先级为 0
	
	
	Motor_Init();
	Encoder_Init();
	Sensor_Init();
    
	oled_set_dir(OLED_CROSSWISE);
	oled_init();
	oled_set_font (OLED_8X16_FONT);
	oled_clear ();
	
	
	
	
	Moto_SetPWM(1,8000);
	Moto_SetPWM(2,8000);
	
    while(1)
    {
		
		
		oled_show_int(0,0,Speed1,3);
		oled_show_int(0,2,Speed2,3);
		
		
		
        

      
    }
}




void pit_handler (void)
{

	Speed2=Get_Count2();///44.f/0.1/9.27666*21.35;
	Speed1=Get_Count1();///44.f/0.1/9.27666*21.35;
	Encoder_Clear();
	
	//Promopt(3);
	//Promopt(4);
}


