#include "zf_common_headfile.h"
#include "PID.h"
#include "Sensor.h"
#include "Motor.h"
#include "Encoder.h"
#include "Menu.h"
#include "distance_control.h"
#include "turn_control.h"
#include <math.h>

void PID_Update(PID_t *p)			// 一般PID函数
{
	p->Error1 = p->Error0;
	p->Error0 = p->Target - p->Actual;
	
	if (p->Ki != 0)
	{
		p->ErrorInt += p->Error0;
	}
	else
	{
		p->ErrorInt = 0;
	}
	
	if(p->ErrorInt>=p->OutMax/2)p->ErrorInt=p->OutMax/2.f;	// 积分限幅
	if(p->ErrorInt<=p->OutMin/2)p->ErrorInt=p->OutMin/2.f;
	
	
	p->Out = p->Kp * p->Error0
		   + p->Ki * p->ErrorInt
	+ p->Kd * (p->Error0 - p->Error1);				
		   
	//-p->Kd*(p->Actual-p->Actual1);//微分先行，将对误差的微分改为对实际值的微分
	
	//	if(p->Out>0){p->Out+=p->OutOffset;}			//输出偏移
	//	if(p->Out<0){p->Out-=p->OutOffset;}
	
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}
	
	p->Actual1=p->Actual;
}

/** 
 * @brief 平衡PID函数
 * @note 参数定义在main函数里方便修改
 * @return 无返回值，当角度过大会触发大角度保护自动停止
 */
extern float yaw, pitch, roll;
extern PID_t AnglePID;
extern int16_t LeftPWM,RightPWM;
extern int16_t AvePWM,DifPWM;
extern PID_t SpeedPID;
extern PID_t TurnPID;
extern float SpeedLeft,SpeedRight;
extern float AveSpeed,DifSpeed;
extern boot_mode CarMode;
void Balance_PIDControl(void)
{
	//角度过大保护
	if (pitch > 50 || pitch < -50)		
	{
		Motor_SetPWM(1,0);
		Motor_SetPWM(2,0);
		SpeedPID.ErrorInt = 0;
		return;
	}
			
	AnglePID.Actual = pitch;
	PID_Update(&AnglePID);
	
	AveSpeed = (SpeedLeft + SpeedRight) / 2.0f;
	DifSpeed = SpeedLeft - SpeedRight;
	
	// 重要：如果位置控制未启用，速度环目标应为0
    // if (!is_distance_control_enabled && !is_distance_reached) 
	// {
    //     SpeedPID.Target = 0.0f;
    // }
		
	SpeedPID.Actual = AveSpeed;
	PID_Update(&SpeedPID);
	
	TurnPID.Actual = DifSpeed;
	PID_Update(&TurnPID);
	DifPWM = TurnPID.Out;
	
	AvePWM = AnglePID.Out + SpeedPID.Out;
		
	LeftPWM = AvePWM + DifPWM / 2;
	RightPWM = AvePWM - DifPWM / 2;
			
	if(LeftPWM > 10000) LeftPWM = 10000; else if(LeftPWM < -10000) LeftPWM = -10000;
	if(RightPWM > 10000) RightPWM = 10000; else if(RightPWM < -10000) RightPWM = -10000;
	Motor_SetPWM(1, LeftPWM);
	Motor_SetPWM(2, RightPWM);
}

/** 
 * @brief 循迹环
 * @note 参数定义在main函数里方便修改
 * @return 无返回值，运行时进行调控，不运行时仍会测速
 */
extern PID_t SensorPID;
extern double yaw_offset;
int sign = 1;
double speed = 0.1f;
void Sensor_PIDControl(void)				//循迹PID函数，至于为啥不叫Trace，这是个历史遗留问题（哭）
{	
	if (!(CarMode == MODE_2 || CarMode == MODE_3)) return;

	static int yaw_offset_counter = 0;
	yaw_offset_counter++;
	if (yaw_offset_counter > 100) {
		yaw_offset = Sensor_GetSensorError();
		yaw_offset_counter = 0;
	}

	static int prev_track_state = 0;
	static int cur_track_state = 0;	// 这么搞主要是为了检测跳变
	prev_track_state = cur_track_state;
	cur_track_state = Sensor_CheckTrack();

	if (prev_track_state == 0 && cur_track_state == 1) { // 刚丢线
		// yaw_offset = 0
		if (CarMode == MODE_3) {
			TurnPID.Target = 3.0f;
		}
	} else if (prev_track_state == 1 && cur_track_state == 1) { // 持续丢线
		if (CarMode == MODE_3) {
			TurnPID.Target -= 0.05f;
		}
	}

	SensorPID.Actual = (float)Sensor_ComplementaryFilteredError(0.9f);

	PID_Update( &SensorPID );

	TurnPID.Target = SensorPID.Out;
	SpeedPID.Target = speed;		// 这个速度倒时候看需求
	
}

