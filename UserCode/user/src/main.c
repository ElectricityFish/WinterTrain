#include "zf_common_headfile.h"
#include "Motor.h"
#include "Encoder.h"
#include "Sensor.h"
#include "BuzzerAndLED.h"
#include "Kfilter.h"
#include "Menu.h"
#include "PID.h"
#include "diskio.h"
#include "turn_control.h"
#include "Inertial_Navigation.h"
#include "BlueSerial.h"

/* ==============================================================================================
                                        全局变量声明
   ============================================================================================== */

///////////////////////////////////////////////////////////////////////////////////////////////// 姿态解算
float gyro_yaw = 0, gyro_pitch = 0, gyro_roll = 0;
float acc_yaw = 0, acc_pitch = 0, acc_roll = 0;
int16 AX, AY, AZ;
float Offset;
float yaw, pitch, roll;
KalmanFilter KF;
volatile uint32_t system_time_ms = 0;

extern double yaw_offset;
///////////////////////////////////////////////////////////////////////////////////////////////// 小车模式

boot_mode CarMode = IDLE;
boot_mode Prev_CarMode = IDLE;

///////////////////////////////////////////////////////////////////////////////////////////////// 小车控制
float SpeedLeft,SpeedRight;
float AveSpeed,DifSpeed;
int16_t LeftPWM,RightPWM;
int16_t AvePWM,DifPWM;
int16_t turn_flag = 0;
int16_t stop_flag = 0;

//循迹需要
extern double speed;
extern int cur_track_state;
uint8_t previouscur_track_state;	//记录上一时刻的循迹状态，用于任务二的声光提示
uint8_t onLinePromoptFlag=0;			//用于任务二的声光提示

PID_t AnglePID={
	.Kp=660.0,
	.Ki=0.0,
	.Kd=1700.0,
	
	.OutOffset=0.0,	// 输出偏移值,让电机动起来的最小PWM
	.Target = 0.0f,
	.OutMax=10000,
	.OutMin=-10000,

};

PID_t SpeedPID={	
	.Kp=-1200,
	.Ki=-1200.0 / 200.0,
	.Kd=0.0,
	
	.Target=0.0f,
	.OutMax=10000,
	.OutMin=-10000,
	
};

PID_t TurnPID={
	.Kp=-2500.0,
	.Ki=0.0f,
	.Kd=0.0,
	
	.Target=0.0f,
	.OutMax=5000.0,
	.OutMin=-5000.0, 
	
};

PID_t SensorPID = {
    .Kp         = 8.5f,
    .Ki         = 0.0f,
    .Kd         = 7.5f,

	.Target     = 0.0f,
    .OutMax     = 10000.0f,
    .OutMin     = -10000.0f,
};
/////////////////////////////////////////////////////////////////////////////////////////////////

/* ==============================================================================================
                                        函数声明
   ============================================================================================== */

void TaskTwoPromopt(void);								//任务二提示函数
void Menu_UpDate(void); //封装后的菜单更新函数

/* ==============================================================================================
                                        主函数
   ============================================================================================== */

int main (void)
{
/////////////////////////////////////////////////////////////////////////////////////////////////
    clock_init(SYSTEM_CLOCK_120M);                       	// 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                        	// 初始化默认 Debug UART
	
	BuzzerAndLED_Init();									// 蜂鸣器初始化
	pit_ms_init(TIM6_PIT, 1);                            	// 初始化 PIT 为周期中断 1ms 周期
    interrupt_set_priority(TIM6_IRQn, 0);                	// 设置 PIT 对周期中断的中断优先级为 0
	
	Motor_Init();											// 电机初始化
	Encoder_Init();											// 编码器初始化
	Sensor_Init();											// 循迹模块初始化
    
	Kalman_Init(&KF, 0.0001f, 0.003f, 0.03f);				// 滤波初始化
    key_init(10);											// 按键初始化
	Menu_Init();											// 初始化菜单，内含OLED初始化
	mpu6050_init();											// 姿态传感器初始化
	
	BludeSerial_Init();										//蓝牙初始化
	
/////////////////////////////////////////////////////////////////////////////////////////////////
	
    while(1)
	{	
		Prev_CarMode = CarMode;
		CarMode = Menu_GetCurMode();
		if(key_get_state(KEY_2)) 
		{
			gyro_yaw = 0;
			speed = 3.0f;
			yaw_offset = 0;
		}
		
		//蓝牙遥控代码
		if(CarMode==MODE_5)
		{
			BlueSerial_Control(&SpeedPID.Target,&TurnPID.Target);
		}
		
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
	static uint8_t Count2=2;
	Count0++;
	Count1++;
	Count2++;
	
	system_time_ms++;  // 增加系统时间
	
	if (CarMode == MODE_2) {
		SpeedPID.Ki = 0.0f;
	} else {
		SpeedPID.Ki = -1200.0 / 200.0f;
	}

	
	if(Count1>=10)//每10ms进行一次按钮检测，和菜单更新，并从菜单获取最新参数
	{
		Count1 = 0;

		Menu_UpDate();
		Menu_JustRefreshValue();
	}
///////////////////////////////////////////////////////////////////////////////////////////////// 姿态解算，角度环控制
	if (Count0 >= 10) // 每10ms进行一次姿态解算，和平衡态控制
	{
		Count0 = 0;
		
		Get_Angle();
		SpeedLeft = Get_Count1();
		SpeedRight = Get_Count2();
		Encoder_Clear();
		
		if(CarMode!=IDLE)
		{
			Balance_PIDControl();//直立PID控制函数，详见PID.c
			
			if (Is_Angle_Turning())
			{
				Update_Angle_Turn();
			}
		}
		else
		{
			Motor_SetPWM(1,0);
			Motor_SetPWM(2,0);
			SpeedPID.ErrorInt = 0;
			SensorPID.ErrorInt = 0;
		}
	}
///////////////////////////////////////////////////////////////////////////////////////////////// 任务二
	if(Count2 >= 15)
	{
		Count2 = 0;
		previouscur_track_state=cur_track_state;
		Sensor_PIDControl();
		TaskTwoPromopt();
		
		
		if(CarMode == MODE_2)
		{
			// 刚检测到断线
			if (cur_track_state == 1) 
			{
				stop_flag ++;
				gyro_yaw = 0.0f;
				TurnPID.Target = 0.0f;
				SensorPID.Ki = 0.0f;
				
				if(stop_flag == 3)
				{
					SpeedPID.Target  = 0.0f;
					Menu_SetRunningMode(MODE_1);
				}
			}
			// 持续断线状态
			else if (cur_track_state == 2) 
			{
				TurnPID.Target = 0.0f;
				SensorPID.Ki = 0.0f;
			}
			// 正常状态
			else
			{
				TurnPID.Target = SensorPID.Out;
				SensorPID.Ki = 0.0f;
			}
			
			if(previouscur_track_state!=cur_track_state)onLinePromoptFlag=1;
				
		}
		
	}
	
}


/**
 * @brief 封装后的菜单跟新函数
 * @note 根据按键状态更新菜单的同时，将最新参数赋值给参与计算的变量
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
	
	//将最新参数赋值给参与计算的变量
//	AnglePID.Kp = Menu_GetValue(STAND_PID_MENU, 0)*1000;
//	AnglePID.Ki = Menu_GetValue(STAND_PID_MENU, 1)*1000;
//	AnglePID.Kd = Menu_GetValue(STAND_PID_MENU, 2)*1000;

//	SpeedPID.Kp = Menu_GetValue(SPEED_PID_MENU, 0);
//	SpeedPID.Ki = Menu_GetValue(SPEED_PID_MENU, 1);
//	SpeedPID.Kd = Menu_GetValue(SPEED_PID_MENU, 2);

//	TurnPID.Kp = Menu_GetValue(TURNING_PID_MENU, 0);
//	TurnPID.Ki = Menu_GetValue(TURNING_PID_MENU, 1);
//	TurnPID.Kd = Menu_GetValue(TURNING_PID_MENU, 2);

//	SensorPID.Kp = Menu_GetValue(SENSOR_PID_MENU, 0);
//	SensorPID.Ki = Menu_GetValue(SENSOR_PID_MENU, 1);
//	SensorPID.Kd = Menu_GetValue(SENSOR_PID_MENU, 2);
	   
}

void TaskTwoPromopt(void)								//任务二提示函数
{
	static uint8_t PromoptFlag=0;
	static uint8_t PromoptCount=0;
	
	if(onLinePromoptFlag==1)
	{
		PromoptCount=20;
		onLinePromoptFlag=0;
	}
	

		
		if(PromoptCount>0)
		{
			Promopt();
			PromoptCount--;
		}else{
			StopPromopt();
		}
		
}



