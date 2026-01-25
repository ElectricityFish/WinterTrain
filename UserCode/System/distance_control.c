/************************************************************************************************
* @name   位置环PID控制
* @note   目前只有一个大概，
		  我在这个文件里需要完成对编码器读取得到距离，
		  PID控制电机运动到相同的距离
* @author 
*************************************************************************************************/
#include "zf_common_headfile.h"
#include "distance_control.h"
#include "menu.h"
#include "Encoder.h"
#include "Motor.h"

#define pi 3.14159265358979323846f

static float wheel_perimeter = 2.0f * pi * 3.40;

static float distance_right = 0.0f;
static float distance_left = 0.0f;

static uint16_t right_getvalue = 0;
static uint16_t left_getvalue = 0;

void Right_EncodeGet(void)
{
	right_getvalue = Get_Count1();
}

void Left_EncodeGet(void)
{
	left_getvalue = Get_Count2();
}

float Right_Distance(void)
{
	distance_right += (right_getvalue * (wheel_perimeter / 44));
	
	return distance_right;
}

float Left_Distance(void)
{
	distance_left += (left_getvalue * (wheel_perimeter / 44));
	
	return distance_left;
}

//结构体初始化位置环PID
Distance_PID distance_pid = {
	.Kp = 0.0f,
	.Ki = 0.0f,
	.Kd = 0.0f,
	
	.OutMax = 10.0f,//暂定
	.OutMin = -10.0f,
};

//这里添加一个通过菜单调参的函数
void distance_pid_UpdatePID(void)
{
	
}

/*
	功能：使小车能够前进/后退指定距离后停止
	使用：run_distance(100)    //前进100cm
*/
void run_distance(uint16_t distance)
{
	distance_pid.Error0 = distance;
	distance_pid.Error1 = distance_pid.Error0;
	
	if(distance_pid.Ki != 0){distance_pid.ErrorInt += distance_pid.Error0;}
	else{distance_pid.ErrorInt = 0;}
	
	distance_pid.Out = distance_pid.Kp * distance_pid.Error0 + distance_pid.Ki * distance_pid.ErrorInt 
						+ distance_pid.Kd * (distance_pid.Error0 - distance_pid.Error1);
	
	//输出限幅
	if(distance_pid.Out > distance_pid.OutMax){distance_pid.Out = distance_pid.OutMax;}
	if(distance_pid.Out < distance_pid.OutMin){distance_pid.Out = distance_pid.OutMin;}
	
	/*
		关于初始PWM，不是0，应该为平衡车PID的PWM值，先填0，倒时候在改
	*/
	if(distance_pid.Out != 0)
	{
		Moto_SetPWM(1,0+distance_pid.Out);
	}
	else
	{
		Moto_SetPWM(2,0+distance_pid.Out);
	}
}
