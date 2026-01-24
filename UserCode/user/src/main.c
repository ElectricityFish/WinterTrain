#include "zf_common_headfile.h"
#include "Motor.h"
#include "Encoder.h"
#include "Sensor.h"
#include "BuzzerAndLED.h"
#include "Kfilter.h"
#include "menu.h"


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
	pit_ms_init(TIM6_PIT, 1);                                                      // 初始化 PIT 为周期中断 1ms 周期
    interrupt_set_priority(TIM6_IRQn, 0);                                // 设置 PIT 对周期中断的中断优先级为 0
	
	
	Motor_Init();
	Encoder_Init();
	Sensor_Init();
    

	Kalman_Init(&KF,0.0001f,0.003f,0.03f);
    key_init(10);
	Menu_Init();//初始化菜单，内含OLED初始化

    while(1)
    {	
		system_delay_ms(10);
        key_scanner();
        if (key_get_state(KEY_4) == KEY_SHORT_PRESS) {
            Menu_Up();
        } else if (key_get_state(KEY_3) == KEY_SHORT_PRESS) {
            Menu_Down();
        } else if (key_get_state(KEY_2) == KEY_SHORT_PRESS) {
            Menu_Forward();
        } else if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
            Menu_Backward();
        } else if (key_get_state(KEY_2) == KEY_LONG_PRESS) {
            Menu_SavePIDToFlash();
        }
    }

      
    
}




void pit_handler (void)
{

	static uint8_t Count=0;
	Count++;

	if(Count>=10)//每10ms进行一次姿态解算
	{
		Count = 0;
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
	}
	
	Speed2=Get_Count2();
	Speed1=Get_Count1();
	Encoder_Clear();


	
	
	
	
	
	
}


