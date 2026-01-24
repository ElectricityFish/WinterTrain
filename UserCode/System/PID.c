#include "zf_common_headfile.h"
#include "PID.h"
#include "Sensor.h"

void PID_Update(PID_t *p)			//一般PID函数
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
	
	if(p->ErrorInt>=p->OutMax/2)p->ErrorInt=p->OutMax/2.f;	//积分限幅
	if(p->ErrorInt<=p->OutMin/2)p->ErrorInt=p->OutMin/2.f;
	
	
	p->Out = p->Kp * p->Error0
		   + p->Ki * p->ErrorInt
	+ p->Kd * (p->Error0 - p->Error1);				
		   
	//-p->Kd*(p->Actual-p->Actual1);微分先行，将对误差的微分改为对实际值的微分
	
//	if(p->Out>0){p->Out+=p->OutOffset;}			//输出偏移
//	if(p->Out<0){p->Out-=p->OutOffset;}
	
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}
	
	p->Actual1=p->Actual;
}



float TracePID_Update(void)				//循迹PID函数
{

    static float Current_Error,Previous_Error,Error_Sum;
	static float kp,ki,kd;
	float PID_Out;
	
	
        Previous_Error=Current_Error;
        Current_Error=Sensor_Get_Error();
       

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

