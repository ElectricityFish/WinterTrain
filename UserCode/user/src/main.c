#include "zf_common_headfile.h"
#include "Motor.h"
#include "Encoder.h"
#include "Sensor.h"
#include "BuzzerAndLED.h"
#include "Kfilter.h"
#include "menu.h"
#include "PID.h"

float gyro_yaw = 0, gyro_pitch = 0, gyro_roll = 0;
float acc_yaw = 0, acc_pitch = 0, acc_roll = 0;
int16 AX, AY, AZ;
float Offset;
float yaw, pitch, roll;
KalmanFilter KF;

boot_mode CarMode=IDLE;

float SpeedLeft,SpeedRight;
float AveSpeed,DifSpeed;
int16_t LeftPWM,RightPWM;
int16_t AvePWM,DifPWM;
PID_t AnglePID={
	.Kp=500.0,
	.Ki=0.0,
	.Kd=800.0,
	
	.OutOffset=0.0,	//输出偏移值,让电机动起来的最小PWM
	.Target=0.f,
	.OutMax=10000,
	.OutMin=-10000,

};

PID_t SpeedPID={	//速度环通过控制目标角度控制行进
	.Kp=0.0,
	.Ki=0.0,
	.Kd=0.0,
	
	.Target=0.f,
	.OutMax=15,//最大倾斜角度
	.OutMin=-15,
	
};

PID_t TurnPID={
	.Kp=0.0,
	.Ki=0.0,
	.Kd=0,
	
	.Target=0.f,
	.OutMax=0.0,
	.OutMin=-0.0, 
	
};

int main (void)
{
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 Debug UART
	
	BuzzerAndLED_Init();
	pit_ms_init(TIM6_PIT, 1);                                            // 初始化 PIT 为周期中断 1ms 周期
    interrupt_set_priority(TIM6_IRQn, 0);                                // 设置 PIT 对周期中断的中断优先级为 0
	
	
	Motor_Init();
	Encoder_Init();
	Sensor_Init();
    

	Kalman_Init(&KF,0.0001f,0.003f,0.03f);
    key_init(10);
	Menu_Init();//初始化菜单，内含OLED初始化
	mpu6050_init();
	
    while(1)
    {	
		CarMode=Menu_GetCurMode();

    }

      
    
}




void pit_handler (void)
{

	static uint8_t Count0=0;
	static uint8_t Count1=5; //初始值不同进行错峰更新
	static uint8_t Count2=0;
	Count0++;
	Count1++;
	Count2++;
	
	
	if(Count1>=10)//每10ms进行一次菜单更新
	{
		Count1=0;
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

	if(Count0>=10)//每10ms进行一次姿态解算，和平衡态控制
	{
		
		
		Count0 = 0;
		mpu6050_get_gyro();
		mpu6050_get_acc();
		
		//姿态解算，使用卡尔曼滤波算法
		
		//yaw角解算（无加速度计校准）
		gyro_yaw += (mpu6050_gyro_transition(mpu6050_gyro_z / 100 * 100) * 0.001);
		yaw = gyro_yaw;
		
		//pitch角解算（加速度计校准）
		//加速度计简陋滤波
		AX = mpu6050_acc_x / 100 * 100;
		AY = mpu6050_acc_y / 100 * 100;
		AZ = mpu6050_acc_z / 100 * 100;
		pitch = calculatePitchAngle(AX, AY, AZ, (mpu6050_gyro_y / 100 * 100) , 0.01, &KF)-Offset;
		
		if(CarMode!=IDLE)
		{
			//角度过大保护
			if (pitch > 50 || pitch < -50)		
			{
				CarMode =IDLE;
				Motor_SetPWM(1,0);
				Motor_SetPWM(2,0);
			}
			
			AnglePID.Actual=pitch;
			PID_Update(&AnglePID);
			
			AvePWM=AnglePID.Out;
			
			LeftPWM=AvePWM+DifPWM/2;
			RightPWM=AvePWM-DifPWM/2;
			
			
			if(LeftPWM>10000)LeftPWM=10000;else if(LeftPWM<-10000)LeftPWM=-10000;
			if(RightPWM>10000)RightPWM=10000;else if(RightPWM<-10000)RightPWM=-10000;
			Motor_SetPWM(1,LeftPWM);
			Motor_SetPWM(2,RightPWM);
		}else{
			Motor_SetPWM(1,0);
			Motor_SetPWM(2,0);
		}
	}
	
	if(Count2>=50)//每50ms获取一次编码器计数值
	{
		SpeedLeft=Get_Count2();
		SpeedRight=Get_Count1();
		Encoder_Clear();
		Count2=0;

	}


	
	
	
	
	
	
}


