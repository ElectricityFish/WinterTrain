#include "zf_common_headfile.h"
#include "PID.h"
#include "BuzzerAndLED.h"
#include "menu.h"
#include "Kfilter.h"

uint8_t onLinePromoptFlag=0;			//用于任务二的声光提示
uint8_t stop_flag = 0;
extern float gyro_yaw ;
extern PID_t TurnPID;
extern PID_t SensorPID;
extern PID_t SpeedPID;
extern  uint8_t previouscur_track_state;//用于任务二的声光提示

void TaskTwoFlagClear(void);
/**
 * @brief 封装后的任务二函数
 * @note 功能：执行任务二
 * @return 无
 */
extern float yaw, pitch, roll;
extern PID_t yawPID;
void TaskTwoRun(void)
{
	
	// 刚检测到断线
	if (cur_track_state == 1) 
	{
		stop_flag ++;
		gyro_yaw = 0.0f;
		TurnPID.ErrorInt=0;
		TurnPID.Target = 0.0f;
		SensorPID.Ki = 0.0f;
		
		if(stop_flag == 3)//任务二完成
		{
			SpeedPID.Target  = 0.0f;
			TaskTwoFlagClear();
			Menu_SetRunningMode(MODE_1);
		}
	}
	// 持续断线状态，直行
	else if (cur_track_state == 2) 
	{
		TurnPID.Target = 0;
		SpeedPID.Target=4.0;
	}
	//正常状态
	else
	{
		SpeedPID.Target=3.0;
		TurnPID.Target = SensorPID.Out;
		SensorPID.Ki = 0.0f;
	}
	
	//判断是否提示
	if(previouscur_track_state!=cur_track_state)onLinePromoptFlag=1;
	
}

/**
 * @brief 任务二标志位清零函数
 * @note 功能：执行任务二标志位的 清理防止出错
 * @return 无
 */
void TaskTwoFlagClear(void)
{
	stop_flag = 0;
	previouscur_track_state=0;
	cur_track_state=0;
	cur_track_state=0;
	TurnPID.ErrorInt=0;
	onLinePromoptFlag=1;//最后提示一次
}


