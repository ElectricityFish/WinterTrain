#include "zf_common_headfile.h"
#include "PID.h"
#include "Sensor.h"
#include "Motor.h"
#include "Encoder.h"
#include "mMenu.h"

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
		SpeedPID.ErrorInt=0;
		return;
	}
			
	AnglePID.Actual=pitch;
	PID_Update(&AnglePID);
	
	AveSpeed=(SpeedLeft+SpeedRight)/2.f;
	DifSpeed=SpeedLeft-SpeedRight;
		
	SpeedPID.Actual=AveSpeed;
	PID_Update(&SpeedPID);
	
	TurnPID.Actual=DifSpeed;
	PID_Update(&TurnPID);
	DifPWM=TurnPID.Out;
	
	AvePWM=AnglePID.Out+SpeedPID.Out;
		
	LeftPWM=AvePWM+DifPWM/2;
	RightPWM=AvePWM-DifPWM/2;
			
	if(LeftPWM>10000)LeftPWM=10000;else if(LeftPWM<-10000)LeftPWM=-10000;
	if(RightPWM>10000)RightPWM=10000;else if(RightPWM<-10000)RightPWM=-10000;
	Motor_SetPWM(1,LeftPWM);
	Motor_SetPWM(2,RightPWM);
}


float TracePID_Update(void)				//循迹PID函数
{

    static float Current_Error,Previous_Error,Error_Sum;
	static float kp,ki,kd;
	float PID_Out;

	kp = 
	
        Previous_Error = Current_Error;
        Current_Error = Sensor_GetError();

        Error_Sum+=Current_Error;

        float Error_Init=ki*Error_Sum;
        //积分限幅
        if(Error_Init>=5)Error_Init=5;
        if(Error_Init<=-5)Error_Init=-5;

        PID_Out=kp*Current_Error+Error_Init+kd*(Current_Error-Previous_Error);
        //PID限幅
        if(PID_Out>=15)PID_Out=15;
        if(PID_Out<=-15)PID_Out=-15;

        //更改目标速度
        	
		return PID_Out;
   
}

