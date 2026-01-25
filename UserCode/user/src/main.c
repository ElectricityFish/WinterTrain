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
	.Kp=850.0,
	.Ki=0.0,
	.Kd=1400.0,
	
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


void Menu_UpDate(void);//封装后的菜单更新函数
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



/**
 * @brief 封装后的中断函数,每1ms在isr的中断函数里调用一次
 * @note 功能：姿态解算，平衡PID计算和执行，菜单更新
 * @note 格式说明：各个功能均已封装到响应的文件里的函数，姿态解算在Kfilter.c，PID计算在PID.c里，菜单更新在main.c里
 * @return 无
 */
void pit_handler (void)
{

	static uint8_t Count0=0;
	static uint8_t Count1=5; //初始值不同进行错峰更新
	static uint8_t Count2=0;
	Count0++;
	Count1++;
	Count2++;
	
	
	if(Count1>=10)//每10ms进行一次按钮检测
	{
		Count1=0;
		Menu_UpDate();
	}

	if(Count0>=10)//每10ms进行一次姿态解算，和平衡态控制
	{
		Count0 = 0;
		
		Get_Angle();

		if(CarMode!=IDLE)
		{
			Angle_PIDControl();//角度环控制函数，详见PID.c
		}else{
			Motor_SetPWM(1,0);
			Motor_SetPWM(2,0);
		}
	}
	
	
	if(Count2>=50)//每50ms获取一次编码器计数值
	{
		Count2=0;
		SpeedAndTurn_PIDControl();
	}
	
}


/**
 * @brief 封装后的菜单跟新函数
 * @return 无
 */
void Menu_UpDate(void)
{
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

