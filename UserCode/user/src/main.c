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
#include "nav_flash.h"
#include <math.h>

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
float Plus_Left = 0;
float Plus_Right = 0;

float Total_Encoder_L = 0;
float Total_Encoder_R = 0;
static float Last_SpeedLeft = 0;
static float Last_SpeedRight = 0;

//循迹需要
extern double speed;
extern int cur_track_state;
uint8_t previouscur_track_state;	    //记录上一时刻的循迹状态，用于任务二的声光提示
uint8_t onLinePromoptFlag=0;			//用于任务二的声光提示

//任务三相关标志位
#define  TASK3_TURN_ANGLE          40   //每次转动的角度
uint8_t task3_stop_flag = 0;            //完成标志位，用于判断小车是否跑完四圈
uint8_t task3_direction_flag = 1;       //方向标志位，1->逆时针,-1->顺时针
uint8_t task3_mode_flag = 0;            //模式切换标志位, 0->任务三循迹, 1->任务一停车

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
	
	quaternion_init();                                      // 初始化四元数模块,用于修正yaw角
	
	BludeSerial_Init();										//蓝牙初始化
	
	Init_Nag();
	
/////////////////////////////////////////////////////////////////////////////////////////////////
	
    while(1)
	{	
		Prev_CarMode = CarMode;
		CarMode = Menu_GetCurMode();
		
		// 检测模式变化
		if (Prev_CarMode != CarMode) {
			// 进入录制模式
			if (CarMode == MODE_4_RECORD) {
				// 清空路径缓冲区，重置记录索引
				N.Save_index = 0;
				N.Current_X = 0;
				N.Current_Y = 0;
				N.Mileage_All = 0;
				N.End_f = 0;
				N.Nag_Stop_f = 0;
				memset(Nav_Record_Buffer, 0, sizeof(Nav_Record_Buffer));
				// 启动录制状态机（状态1）
				N.Nav_System_Run_Index = 1;
			}
			// 进入复现模式
			else if (CarMode == MODE_4_REPLAY) {
				// 从Flash加载路径
				if (flash_load_nag()) {
					N.Run_index = 0;
					N.Current_X = 0;
					N.Current_Y = 0;
					N.Nag_Stop_f = 0;
					// 启动复现状态机（状态2）
					N.Nav_System_Run_Index = 2;
				} else {
					// 无有效路径数据，强制退出模式
					Menu_SetRunningMode(IDLE);
				}
			}
			// 退出录制模式时保存路径
			else if (Prev_CarMode == MODE_4_RECORD && CarMode == IDLE) {
				flash_save_nag();   // 将录制的路径存入Flash
				N.Nav_System_Run_Index = 0;   
			}
			// 退出复现模式时只需停止状态机（路径已在Flash中）
			else if (Prev_CarMode == MODE_4_REPLAY && CarMode == IDLE) {
				N.Nav_System_Run_Index = 0;
			}
		}

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
	static uint8_t Count3=8;
	Count0++;
	Count1++;
	Count2++;
	Count3++;
	
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
		
		// 调用四元数更新（内部会读取最新的陀螺仪数据并积分）
//		quaternion_update();
		
//		// 获取四元数解算出的欧拉角（单位：度）
//		Quaternion *att = get_eular_angles();
//		yaw = att->yaw;
		
		SpeedLeft = Get_Count1();
		SpeedRight = Get_Count2();
		Encoder_Clear();
		
		Plus_Left += SpeedLeft;
		Plus_Right += SpeedRight;
		
//		if(CarMode == MODE_1)
//		{
//			Balance_PIDControl();//直立PID控制函数，详见PID.c
//			
////			if (Is_Angle_Turning())
////			{
////				Update_Angle_Turn();
////			}
//		}
//		else
//		{
//			Motor_SetPWM(1,0);
//			Motor_SetPWM(2,0);
//			SpeedPID.ErrorInt = 0;
//			SensorPID.ErrorInt = 0;
//		}
		switch(CarMode){
			case MODE_1:
			case MODE_2:
			case MODE_3:
			case MODE_5:
				Balance_PIDControl();
				break;
			case MODE_4_RECORD:
				// 录制模式：确保电机停止，不执行平衡控制
				Motor_SetPWM(1, 0);
				Motor_SetPWM(2, 0);
				SpeedPID.ErrorInt = 0;
				SensorPID.ErrorInt = 0;
				break;
			case MODE_4_REPLAY:
				// 复现模式需要平衡控制
				Balance_PIDControl();
				break;
			default:
				Motor_SetPWM(1, 0);
				Motor_SetPWM(2, 0);
				SpeedPID.ErrorInt = 0;
				SensorPID.ErrorInt = 0;
				break;
		}
		/*
		惯导的代码放在这，与yaw角的获取频率一致
		*/
		if(N.Nav_System_Run_Index != 0)
		{
			
			// 声明静态变量，用于记录复现模式下的历史速度 (计算加速度用)
			static float Replay_Last_L = 0.0f;
			static float Replay_Last_R = 0.0f;

			// =========================================================
			// 【模式 1：录制模式 (Teach)】- 绝对纯净，0 延迟，0 限幅
			// =========================================================
			if(N.Nav_System_Run_Index == 1) 
			{
				// 录制时：完全信任真实脉冲，不漏掉任何一个微小的转角
				Total_Encoder_L = SpeedLeft;
				Total_Encoder_R = SpeedRight;
				
				// 实时同步历史值，防止未来切入复现模式瞬间产生巨大跳变
				Replay_Last_L = SpeedLeft;
				Replay_Last_R = SpeedRight;
			}
			// =========================================================
			// 【模式 3：复现模式 (Replay)】- 开启双重防打滑保护
			// =========================================================
			else if(N.Nav_System_Run_Index == 2)
			{
				
				float nav_L = SpeedLeft;
				float nav_R = SpeedRight;
				
				// --- 第一重保护：加速度限幅 (防起步/急刹瞬间打滑) ---
				float delta_L = nav_L - Replay_Last_L;
				float delta_R = nav_R - Replay_Last_R;
				
				// 【参数】加速度阈值：2ms内脉冲突变不允许超过 15。
				// (15相当于极强的物理推背感，超过这个值99%是车轮空转打滑)
				float slip_threshold = 40.0f; 
				
				if (fabsf(delta_L) > slip_threshold) {
					nav_L = Replay_Last_L + (delta_L > 0 ? slip_threshold : -slip_threshold);
				}
				if (fabsf(delta_R) > slip_threshold) {
					nav_R = Replay_Last_R + (delta_R > 0 ? slip_threshold : -slip_threshold);
				}
				
				// --- 第二重保护：绝对物理极限限幅 (防彻底腾空空转) ---
				// 【参数】最大车速阈值：假设车子物理极限速度是 120cm/s (约 150脉冲/2ms)
				float max_abs_speed = 150.0f; 
				if(nav_L > max_abs_speed) nav_L = max_abs_speed; else if(nav_L < -max_abs_speed) nav_L = -max_abs_speed;
				if(nav_R > max_abs_speed) nav_R = max_abs_speed; else if(nav_R < -max_abs_speed) nav_R = -max_abs_speed;
				
				// 更新历史值供下个2ms使用
				Replay_Last_L = nav_L;
				Replay_Last_R = nav_R;
				
				// 喂给导航系统进行安全积分
				Total_Encoder_L = nav_L;
				Total_Encoder_R = nav_R;
			}
			
			Nag_System(); // 执行惯导核心 (10ms 一次)
			
			// 如果是复现模式，将惯导输出的转向量作为转向目标
			if (CarMode == MODE_4_REPLAY) {
				TurnPID.Target = N.Final_Out;
			}
			
			if(N.Nag_Stop_f == 1)
            {
                SpeedPID.Target = 0.0f;    // 1. 速度归零 (刹车)
                TurnPID.Target = 0.0f;        // 2. 转向归零 (回正)
				Menu_SetRunningMode(MODE_1);
                N.Nav_System_Run_Index = 0; // 3. 退出惯导状态机              
            }
        }
		else
		{
			Total_Encoder_L = 0;
			Total_Encoder_R = 0;
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
/////////////////////////////任务三////////////////////////////////////
	if(Count3 >= 18)  // 每18ms执行一次
	{
		Count3 = 0;
		if(CarMode == MODE_3)
		{
			// 保存之前的循迹状态
			previouscur_track_state = cur_track_state;
			
			// 更新传感器状态
			Sensor_PIDControl();
			
			// 任务3的声光提示
			if(previouscur_track_state != cur_track_state) {
				onLinePromoptFlag = 1;
			}
			TaskTwoPromopt();
			
			// 任务3的核心逻辑
			switch(task3_mode_flag) {
				case 0:  // 循迹模式
					// 检测到断点
					if(cur_track_state == 1) {
						task3_stop_flag++;
						
						SpeedPID.Target = 0.0f;
						TurnPID.Target = 0.0f;
						SensorPID.Ki = 0.0f;
						gyro_yaw = 0.0f;

						// 开始转向
						Start_Angle_Turn(task3_direction_flag * TASK3_TURN_ANGLE);
						task3_mode_flag = 1;  // 切换到直行模式

					}
					break;
					
				case 1:  // 直行模式
					// 检查是否还在转向
					if(Is_Angle_Turning()) {
						Update_Angle_Turn();
					} else {
						// 转向完成，开始直行
						SpeedPID.Target = 1.5f;
						TurnPID.Target = 0.0f;  // 保持直行
						
						// 检测是否重新检测到赛道
						if(cur_track_state == 0) {
							// 重新开始循迹
							task3_mode_flag = 0;
							SpeedPID.Target = speed;  // 恢复正常速度
							
							// 切换转向方向（为下次转向做准备）
							task3_direction_flag = -task3_direction_flag;
						}
					}
					
					// 检查是否完成四圈
					if(task3_stop_flag >= 9) {  // 完成四圈（每圈2次断线）
						task3_stop_flag = 0;
						task3_mode_flag = 0;
						SpeedPID.Target = 0.0f;
						Menu_SetRunningMode(MODE_1);  // 最终停车
					}
					break;
			}
		}
	}
///////////////////////////////////////任务4////////////////////////////////
	if(CarMode == MODE_4_RECORD)
	{
		Motor_SetPWM(1,0);
		Motor_SetPWM(2,0);
		N.Nav_System_Run_Index = 1;
	}
	else if(CarMode == MODE_4_REPLAY)
	{
		N.Nav_System_Run_Index = 2;
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
