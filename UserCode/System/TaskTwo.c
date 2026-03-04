#include "zf_common_headfile.h"
#include "PID.h"
#include "BuzzerAndLED.h"
#include "menu.h"

uint8_t onLinePromoptFlag=0;			//用于任务二的声光提示
uint8_t previouscur_track_state;
int16_t stop_flag = 0;
extern float gyro_yaw ;
extern PID_t TurnPID;
extern PID_t SensorPID;
extern PID_t SpeedPID;


/**
 * @brief 封装后的任务二函数
 * @note 功能：执行任务二
 * @return 无
 */
void TaskTwoRun(void)
{
	
	// 刚检测到断线
	if (cur_track_state == 1) 
	{
		stop_flag ++;
		gyro_yaw = 0.0f;
		TurnPID.Target = 0.0f;
		SensorPID.Ki = 0.0f;
		
		if(stop_flag == 3)//任务二完成
		{
			SpeedPID.Target  = 0.0f;
			Menu_SetRunningMode(MODE_1);
		}
	}
	// 持续断线状态，直行
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


/**
 * @brief 任务二提示函数
 * @note 功能：进行声光提示
 * @return 无
 */
void TaskTwoPromopt(void)								
{
	static uint8_t PromoptFlag=0;
	static uint8_t PromoptCount=0;
	
	previouscur_track_state=cur_track_state;
	
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



