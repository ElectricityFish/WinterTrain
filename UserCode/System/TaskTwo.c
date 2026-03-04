#include "zf_common_headfile.h"
#include "PID.h"
#include "BuzzerAndLED.h"
#include "menu.h"

uint8_t onLinePromoptFlag=0;			//用于任务二的声光提示
int16_t stop_flag = 0;
extern float gyro_yaw ;
extern PID_t TurnPID;
extern PID_t SensorPID;
extern PID_t SpeedPID;
extern  uint8_t previouscur_track_state;

/**
 * @brief 封装后的任务二函数
 * @note 功能：执行任务二
 * @return 无
 */
 extern PID_t yawPID;
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
		yawPID.Target=0;
		Yaw_PIDControl(1);
		TurnPID.Target = yawPID.Out;
		SensorPID.Ki = 0.0f;
	}
	// 正常状态
	else
	{
		Yaw_PIDControl(0);
		TurnPID.Target = SensorPID.Out;
		SensorPID.Ki = 0.0f;
	}
			
	if(previouscur_track_state!=cur_track_state)onLinePromoptFlag=1;
	
}

