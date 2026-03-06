#include "zf_common_headfile.h"
#include "PID.h"

//状态宏定义
#define Turn 1 
#define Turning 2
#define Direct 3
#define Slow 4
#define TrackLine 5


//角度距离宏定义
#define TurnAngle 50
#define DirectDistance 200 
#define AllDistance 150 

uint8_t CircleCount=0;  //统计圈数
uint8_t TaskState;		//运行模式
uint8_t RunFlag=0;		//运行标志用于第一次转向


//外部引用
extern int cur_track_state;					//循迹状态
extern uint8_t previouscur_track_state;	    //记录上一时刻的循迹状态
extern float Plus_Left;
extern float Plus_Right;
extern PID_t SpeedPID;
extern PID_t TurnPID;
extern PID_t SensorPID;
extern PID_t AnglePID;
extern PID_t SpeedPID;
extern PID_t TurnPID;


int16_t distance_Left = 0;
int16_t distance_Right = 0;
int16_t distance_track3 = 0;

//量程转换
#define LEFT_PLUS_TO_CM       (6.8  * 3.1415926f / 257)
#define RIGHT_PLUS_TO_CM      (6.8  * 3.1415926f / 293)
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


void TaskThreeStateUpDate(void)
{
	if(RunFlag==0)
	{
		TaskState=Turn;			//刚在白线上发车先转弯
		RunFlag=0;
	}
	else{
		switch(TaskState)
		{
			case Turn:
			{
				TaskState=Turning;
				break;
			}
			case Turning:
			{
				if(Is_Angle_Turning()) {
					Update_Angle_Turn();
				}else{
					TaskState=Direct;
				}
				break;
			}
			case Direct:
			{
				if(distance_track3>DirectDistance)
				{
					TaskState=Slow;
				}
				break;
			}
			case Slow:
			{
				if(distance_track3>AllDistance)
				{
					TaskState=TrackLine;
					Distance_Init();	//距离清0
					TurnPID.ErrorInt=0;
				}
				break;
			}
			case TrackLine:
			{
				if(cur_track_state!=0)
				{
					TurnPID.ErrorInt=0;
					TaskState=Turning;
				}
				break;
			}
				
		}
		
		
		
	}
}


void TaskThreeRun(void)
{
	TaskThreeStateUpDate();
	
	switch(TaskState)
	{
		case Turn:
		{
			SpeedPID.Target=0;
			Start_Angle_Turn(TurnAngle);
			break;
		}
		case Direct:
		{
			SpeedPID.Target=3.f;
			break;
		}
		case Slow:
		{
			SpeedPID.Target=2.f;
			break;
		}
		case TrackLine:
		{
			SpeedPID.Target=2.f;
			TurnPID.Target = SensorPID.Out;
			break;
		}
		
	}
	
}	

