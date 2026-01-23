#include "zf_common_headfile.h"
#include "Motor.h"

#define PIT                             (TIM6_PIT )                             // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_PRIORITY                    (TIM6_IRQn)                             // 对应周期中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体



int main (void)
{
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 Debug UART
	Motor_Init();
    
	pit_ms_init(PIT, 100);                                                      // 初始化 PIT 为周期中断 100ms 周期
    interrupt_set_priority(PIT_PRIORITY, 0);                                    // 设置 PIT 对周期中断的中断优先级为 0
	
	Moto_SetPWM(1,500);
	Moto_SetPWM(2,500);
	
    while(1)
    {
       
        

      
    }
}


void pit_handler (void)
{
//    encoder_data_quaddec = encoder_get_count(ENCODER_QUADDEC);                  // 获取编码器计数
//    encoder_data_dir = encoder_get_count(ENCODER_DIR);                          // 获取编码器计数 
//    encoder_clear_count(ENCODER_QUADDEC);                                       // 清空编码器计数
//    encoder_clear_count(ENCODER_DIR);                                           // 清空编码器计数
}



