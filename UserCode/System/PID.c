#include "zf_common_headfile.h"
#include "PID.h"
#include "Sensor.h"
#include "Motor.h"
#include "Encoder.h"
#include "Menu.h"
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
extern float gyro_yaw;

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
	
	float speed_filter = 1.0; 
    if(fabs(pitch) < 3.5f ) {
    speed_filter = 0.1f;  // 静态强滤波
    } else {
    speed_filter = 0.5f; // 动态弱滤波
    }
    
    AveSpeed = AveSpeed * (1-speed_filter) + (SpeedLeft+SpeedRight)/2.f * speed_filter;//低通滤波
    DifSpeed=SpeedLeft-SpeedRight;
		
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
double speed = 2.0f;

int prev_track_state = 0;
int cur_track_state = 0;	// 这么搞主要是为了检测跳变

void Sensor_PIDControl(void)				//循迹PID函数，至于为啥不叫Trace，这是个历史遗留问题（哭）
{	
	if (!(CarMode == MODE_2 || CarMode == MODE_3)) return;

	static int yaw_offset_counter = 0;
	yaw_offset_counter++;
	if (yaw_offset_counter > 100) {
		yaw_offset = Sensor_GetSensorError();
		yaw_offset_counter = 0;
	}

	prev_track_state = cur_track_state;
	cur_track_state = Sensor_CheckTrack();
	/*
		我的转向角需要放在主函数的while里，所以把断线部分的
		单独拉到出来
	*/
	
//	if(CarMode == MODE_2){
//		if (prev_track_state == 0 && cur_track_state == 1) { // 刚丢线
//			// yaw_offset = 0
//			gyro_yaw = 0;
//			TurnPID.Target = 0.0f;
//		} else if (prev_track_state == 1 && cur_track_state == 1) { // 持续丢线			
//			gyro_yaw = 0;
//			TurnPID.Target = 0.0f;
//		}
//		else{TurnPID.Out = 0.0f;}
//	}
	
//	if(CarMode == MODE_3){
//		if (prev_track_state == 0 && cur_track_state == 1) { // 刚丢线
//			// yaw_offset = 0
//			Start_Angle_Turn(37);
//			TurnPID.Target = 0.0f;
//		} else if (prev_track_state == 1 && cur_track_state == 1) {		}
//		else{TurnPID.Out = 0.0f;}
//	}
	SensorPID.Actual = (float)Sensor_ComplementaryFilteredError(0.9f);

	PID_Update( &SensorPID );

	TurnPID.Target = SensorPID.Out;
	SpeedPID.Target = speed;		// 这个速度倒时候看需求
	
}
