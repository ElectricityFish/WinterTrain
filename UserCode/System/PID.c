#include "zf_common_headfile.h"
#include "PID.h"
#include "Sensor.h"
#include "Motor.h"
#include "Encoder.h"
#include "menu.h"

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
		   
//-p->Kd*(p->Actual-p->Actual1);微分先行，将对误差的微分改为对实际值的微分	
//	if(p->Out>0){p->Out+=p->OutOffset;}			//输出偏移
//	if(p->Out<0){p->Out-=p->OutOffset;}
	
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}
	
	p->Actual1=p->Actual;
}

/** 
 * @brief 角度环PID函数
 * @note 参数定义在main函数里方便修改
 * @return 无返回值，当角度过大会触发大角度保护自动停止
 */
extern float yaw, pitch, roll;     	// 3轴
extern PID_t AnglePID;             	// 角度PID
extern int16_t LeftPWM,RightPWM;   	// 左右电机设定值
extern int16_t AvePWM,DifPWM;      	// 左右均值、差分
void Angle_PIDControl(void)
{
	//角度过大保护
	if (pitch > 50 || pitch < -50)		
	{
		Motor_SetPWM(1,0);
		Motor_SetPWM(2,0);
		return;
	}
			
	AnglePID.Actual=pitch;
	PID_Update(&AnglePID);
		
	AvePWM=AnglePID.Out;
	LeftPWM  = AvePWM + DifPWM / 2;
	RightPWM = AvePWM - DifPWM / 2;

	if (LeftPWM>10000) 	LeftPWM	=	10000;	else if (LeftPWM<-10000)	LeftPWM	=	-10000;
	if (RightPWM>10000)	RightPWM=	10000;	else if (RightPWM<-10000)	RightPWM=	-10000;

	Motor_SetPWM(1,LeftPWM);
	Motor_SetPWM(2,RightPWM);
}

/** 
 * @brief 速度环和转向环PID函数
 * @note 参数定义在main函数里方便修改
 * @return 无返回值，运行时进行调控，不运行时仍会测速
 */
extern PID_t SpeedPID;
extern PID_t TurnPID;
extern float SpeedLeft,SpeedRight;			// 测得的速度
extern float AveSpeed,DifSpeed;				// 平均速度、速度差分
extern boot_mode CarMode;
void SpeedAndTurn_PIDControl(void)
{
		SpeedLeft = Get_Count2();
		SpeedRight = Get_Count1();
		Encoder_Clear();

		AveSpeed = (SpeedLeft + SpeedRight) / 2.f;
		DifSpeed = SpeedLeft - SpeedRight;
		
		if (CarMode != IDLE)
		{
			///////////////////////////////速度环
			SpeedPID.Actual = AveSpeed;
			PID_Update(&SpeedPID);
			AnglePID.Target = SpeedPID.Out;
			///////////////////////////////转向环
			TurnPID.Actual = DifSpeed;
			PID_Update(&TurnPID);
			DifPWM = TurnPID.Out;
		}
}

/** 
 * @brief 循迹环
 * @note 参数定义在main函数里方便修改
 * @return 无返回值，运行时进行调控，不运行时仍会测速
 */
extern PID_t SensorPID;
void Sensor_PIDControl(void)				//循迹PID函数，至于为啥不叫Trace，这是个历史遗留问题（哭）
{	
	if (!(CarMode == MODE_2 || CarMode == MODE_3)) return;

	static int prev_track_state = 0;
	static int cur_track_state = 0;	// 这么搞主要是为了检测跳变

	prev_track_state = cur_track_state;
	cur_track_state = Sensor_CheckTrack();

	if (prev_track_state == 0 && cur_track_state == 1) { // 看看有没有丢线，丢了就要做另外的事了，而且得注意，这个操作是一次性的，我们得置一个标志位
		// 这里缺声光模块的代码 WIP
		if (CarMode == MODE_2) {
			TurnPID.Target = 0.0f; // 直行呗，扫到线了再说
		} else {
			// WIP 这个得好好想想，我们可能需要实际的去测如何转向，首先是不能一直把Target设为一个定值
			// 这里要填写一个转向函数，然后置一个标志位 WIP
			// 转完之后的TurnPID Target理应是0，所以只需要一个转向函数就够了
		}
		return; // 直接return掉防止干扰
	} else if (prev_track_state == 1 && cur_track_state == 0) {
		// 这里缺声光模块的代码 WIP
	} else if (prev_track_state == 1 && cur_track_state == 1) {
		return ; // 丢线状态下面PID就别算了吧，哈
	}				

	double sensor_error = Sensor_GetError();
	SensorPID.Actual = sensor_error;

	PID_Update(&SensorPID);

	TurnPID.Target = SensorPID.Out;
	SpeedPID.Target = 100.0f;		// 这个速度倒时候看需求

}

