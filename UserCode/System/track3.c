#include "zf_common_headfile.h"
#include "track3.h"

//#define TRACK3_TURN_ANGLE     48  //转动角度，稍大一点
#define LEFT_PLUS_TO_CM       (6.8  * 3.1415926f / 257)
#define RIGHT_PLUS_TO_CM      (6.8  * 3.1415926f / 293)

//外部引用
extern int cur_track_state;
extern uint8_t previouscur_track_state;	    //记录上一时刻的循迹状态
extern uint8_t onLinePromoptFlag;			//声光提示
extern float Plus_Left;
extern float Plus_Right;
extern PID_t SpeedPID;
extern PID_t TurnPID;
extern PID_t SensorPID;
extern PID_t AnglePID;

int8_t track3_flag = 0;          //两边有点区别，用这个标志位来判断
int8_t track3_end_flag = 0;      //结束标志位
int8_t track3_dir_flag = -1;     //转向方向

int16_t distance_Left = 0;
int16_t distance_Right = 0;
int16_t distance_track3 = 0;

//引用
void Task3_flag_Clear();

//量程转换
void Distance_Cal()
{
	distance_Left = Plus_Left * LEFT_PLUS_TO_CM;
	distance_Right = Plus_Right * RIGHT_PLUS_TO_CM;
	distance_track3 = (distance_Left + distance_Right) / 2;
}

//路径清零
void Distance_Init()
{
	Plus_Left = 0;
	Plus_Right = 0;
}

void Track3_Start()
{
	Distance_Cal();
	//黑线进白线，转角
	if(cur_track_state == 1)
	{
		track3_flag = !track3_flag;
		track3_dir_flag = -track3_dir_flag;
		if(track3_flag == 1)
		{
			Start_Angle_Turn(track3_dir_flag * TRACK3_TURN_ANGLE1);
		}
		else if(track3_flag == 0)
		{
			Start_Angle_Turn(track3_dir_flag * TRACK3_TURN_ANGLE2);
		}
		track3_end_flag ++;
		Distance_Init();
		//任务三结束判断
		if(track3_end_flag >= 9)
		{
			SpeedPID.Target  = 0.0f;
			Task3_flag_Clear();
			Menu_SetRunningMode(MODE_1);
		}
	}
	// 持续断线状态，直行+转角+小段直行
	else if (cur_track_state == 2) 
	{
		if(Is_Angle_Turning()) {
			Update_Angle_Turn();
		} 
		else {
			TurnPID.Target = 0.0f;  // 保持直行
			if(track3_flag == 1)
			{
				if(distance_track3 >= 0 && distance_track3 <= 100)
				{
					SpeedPID.Target = 3.0f;   //提速冲刺
				}
				else if(distance_track3 > 100 && distance_track3 < 130)
				{
					SpeedPID.Target = 2.5f;    //稍微降速
				}
				else if(distance_track3 >= 130)   //快要到时转回直线
				{
					SpeedPID.Target = 2.0f;	
					Start_Angle_Turn(-track3_dir_flag * TRACK3_TURN_ANGLE1);
					Distance_Init();
				}
			}
			else if(track3_flag == 0)
			{
				if(distance_track3 >= 0 && distance_track3 <= 100)
				{
					SpeedPID.Target = 3.0f;   //提速冲刺
				}
				else if(distance_track3 > 100 && distance_track3 <= 130)
				{
					SpeedPID.Target = 2.5f;    //稍微降速
				}
				else if(distance_track3 >= 131)   //快要到时转回直线
				{
					SpeedPID.Target = 2.0f;	
					Start_Angle_Turn(-track3_dir_flag * TRACK3_TURN_ANGLE2);
					Distance_Init();
				}
			}

		}
	}
	//正常状态
	else
	{
		SpeedPID.Target=2.5;
		TurnPID.Target = SensorPID.Out;
		SensorPID.Ki = 0.0f;
	}
	
	//判断是否提示
	if(previouscur_track_state!=cur_track_state)onLinePromoptFlag=1;
}

void Task3_flag_Clear()
{
	//标志位清0
	track3_end_flag = 0; 
	track3_dir_flag = -1;
}
