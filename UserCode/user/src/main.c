#include "zf_common_headfile.h"
#include "Motor.h"
#include "Encoder.h"
#include "Sensor.h"
#include "BuzzerAndLED.h"
#include "Kfilter.h"

#define PIT                             (TIM6_PIT )                             // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_PRIORITY                    (TIM6_IRQn)                             // 对应周期中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体


float gyro_yaw = 0, gyro_pitch = 0, gyro_roll = 0;
float acc_yaw = 0, acc_pitch = 0, acc_roll = 0;
int16 AX, AY, AZ;
float Offset;
float yaw, pitch, roll;
KalmanFilter KF;


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


	// 此处编写用户代码
	//中断中获取角速度和加速度（1ms）
	mpu6050_get_gyro();
	mpu6050_get_acc();
	
	//姿态解算，使用卡尔曼滤波算法
	float Alpha = 0.001;
	
	//yaw角解算（无加速度计校准）
	gyro_yaw += (mpu6050_gyro_transition(mpu6050_gyro_z / 100 * 100) * 0.001);
	yaw = gyro_yaw;
	
	//pitch角解算（加速度计校准）
	//加速度计简陋滤波
	AX = mpu6050_acc_x / 100 * 100;
	AY = mpu6050_acc_y / 100 * 100;
	AZ = mpu6050_acc_z / 100 * 100;
	pitch = calculatePitchAngle(AX, AY, AZ, (mpu6050_gyro_y / 100 * 100) , 0.01, &KF)-Offset;
	
	//roll角解算（加速度计校准）
	//使用互补滤波
	gyro_roll += (mpu6050_gyro_transition(mpu6050_gyro_z / 100 * 100) * 0.001);
	acc_roll = atan2(mpu6050_acc_transition(mpu6050_acc_y / 100 * 100) * 0.001, mpu6050_acc_transition(mpu6050_acc_z / 100 * 100) * 0.001) / 3.14159 * 180;
	roll = (1 - Alpha) * gyro_roll + Alpha * acc_roll;
	
	
}


