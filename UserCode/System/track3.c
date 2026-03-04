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

int8_t track3_flag = 0;          //0->循迹   1->直走
int8_t track3_turn_flag = 0;     //转向标志位
int8_t track3_end_flag = 0;      //结束标志位
int8_t track3_dir_flag = -1;     //转向方向

int16_t distance_Left = 0;
int16_t distance_Right = 0;
int16_t distance_track3 = 0;

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
	
//	// 更新传感器状态
//	Sensor_PIDControl();
	
//	if(previouscur_track_state != cur_track_state)
//    {
//    	track3_flag = !track3_flag;
//		track3_turn_flag = (track3_turn_flag + 1) % 4;
//		if(track3_turn_flag == 1 || track3_turn_flag == 3)
//		{
//			track3_dir_flag = -track3_dir_flag;
//		}		
//		track3_end_flag ++;
//		Start_Angle_Turn(track3_dir_flag * TRACK3_TURN_ANGLE);
//		Distance_Init();
//    }
	
	switch(track3_flag){
		//循迹
		case 0:
			SpeedPID.Target = 2.0f;    //?
			TurnPID.Target = 0.0f;
			SensorPID.Ki = 0.0f;					
			break;
		//直走	
		case 1:
			if(Is_Angle_Turning()) {
				Update_Angle_Turn();
			} else {
				TurnPID.Target = 0.0f;  // 保持直行
				if(distance_track3 >= 0 && distance_track3 <= 100)
				{
					SpeedPID.Target = 2.5f;   //提速冲刺
				}
				else if(distance_track3 > 100 && distance_track3 <= 129)
				{
					SpeedPID.Target = 2.0f;    //稍微降速
				}
				else if(distance_track3 >= 131)   //快要到时转回直线
				{
					SpeedPID.Target = 1.5f;	
					Start_Angle_Turn(-track3_dir_flag * TRACK3_TURN_ANGLE);
					Distance_Init();
				}
			}
			break;
		}
	
	//四圈绕8字完成标志
	if(track3_end_flag == 17)
	{
		track3_flag = 0;
		track3_turn_flag = 0;
		track3_dir_flag = -1;
		track3_end_flag = 0; 
		SpeedPID.Target = 0.0f;
		Menu_SetRunningMode(MODE_1);  // 最终停车
	}
}
