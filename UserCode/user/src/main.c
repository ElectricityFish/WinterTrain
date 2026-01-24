/*********************************************************************************************************************
* MM32F327X-G8P Opensourec Library 即（MM32F327X-G8P 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MM32F327X-G8P 开源库的一部分
* 
* MM32F327X-G8P 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.37
* 适用平台          MM32F327X_G8P
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-08-10        Teternal            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"


// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程 完成上电
// 
// 2.可以看到核心板上两个 LED 呈流水灯状闪烁
// 
// 3.将 SWITCH1 / SWITCH2 两个宏定义对应的引脚分别按照 00 01 10 11 的组合接到 1-VCC 0-GND 或者波动对应主板的拨码开关
// 
// 3.不同的组合下 两个 LED 流水灯状闪烁的频率会发生变化
// 
// 4.将 KEY1 / KEY2 / KEY3 / KEY4 两个宏定义对应的引脚接到 1-VCC 0-GND 或者 按对应按键
// 
// 5.任意引脚接 GND 或者 按键按下会使得两个 LED 一起闪烁 松开后恢复流水灯
// 
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查


// **************************** 代码区域 ****************************

#include "zf_device_oled.h"
#include "zf_driver_timer.h"
#include "zf_device_key.h"
#include "zf_device_mpu6050.h"
#include "zf_driver_pit.h"
#include "zf_driver_delay.h"
#include "zf_device_bluetooth_ch9141.h"
#include "Kfilter.h"

#define LED1                    (H2 )
#define LED2                    (B13)

#define KEY1                    (E2 )
#define KEY2                    (E3 )
#define KEY3                    (E4 )
#define KEY4                    (E5 )

#define SWITCH1                 (D3 )
#define SWITCH2                 (D4 )


float yaw = 0;
float pitch = 0;
float roll = 0;

//使用卡尔曼滤波
KalmanFilter KF;

int main (void)
{
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 Debug UART
	
	//初始化OLED（测试）
	oled_set_dir(OLED_CROSSWISE);
	oled_init();
	oled_set_font(OLED_6X8_FONT);
	oled_clear();
	
	//初始化Timer
	timer_init(TIM_1, TIMER_US);
	timer_clock_enable(TIM_1);
	uint16_t time = 0;
	timer_start(TIM_1);
	
	//初始化Key
	key_init(1000);
	uint8_t Key_flag;
	uint16_t Key_num = 0;
	
	//初始化MPU6050
	mpu6050_init();
	
	//初始化中断 （isr.c文件）
	pit_ms_init(TIM1_PIT, 1);
	
	//初始化卡尔曼滤波
	/*
		在中断（1ms）中计算，中断在 @isr.c 文件中的 @void TIM1_UP_IRQHandler (void)
		roll角还有问题，我有空会优化的
	*/
	Kalman_Init(&KF,0.0001f,0.003f,0.03f);
	
	//蓝牙串口初始化
	bluetooth_ch9141_init();
	
    // 此处编写用户代码 例如外设初始化代码等

    while(1)
    {
		key_scanner();
		
		
        // 此处编写需要循环执行的代码
		oled_show_string(0, 0,"Gyro     Acc");
		oled_show_int(0, 1, mpu6050_gyro_x, 4);
		oled_show_int(60, 1, mpu6050_acc_x, 4);
		oled_show_int(0, 2, mpu6050_gyro_y, 4);
		oled_show_int(60, 2, mpu6050_acc_y, 4);
		oled_show_int(0, 3, mpu6050_gyro_z, 4);
		oled_show_int(60, 3, mpu6050_acc_z, 4);
		
		oled_show_string(0,4,"Yaw:");
		oled_show_string(0,5,"Pitch:");
		oled_show_string(0,6,"Roll:");
		oled_show_float(80,4,yaw,3,3);
		oled_show_float(80,5,pitch,3,3);
		oled_show_float(80,6,roll,3,3);
		
		
    }
}
// **************************** 代码区域 ****************************

