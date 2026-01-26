/************************************************************************************************
* @name   转向环PID控制
* @note   目前大概只是一个框架，实际效果还不知道，我是指用法可以再改
		  转向环PID的参值与菜单相连
* @author 
*************************************************************************************************/
#include "zf_common_headfile.h"
#include "turn_control.h"
#include "menu.h"
#include "Motor.h"

extern float yaw;

//结构体初始化
Turn_PID turn_pid = {
	.Kp = 0.0f,
	.Kd = 0.0f,
	.Ki = 0.0f,
	
	.OutMax = 10.0f,//暂定
	.OutMin = -10.0f,
};

//可以通过菜单调参
void turn_pid_UpdatePID(void)
{
    turn_pid.Kp = Menu_GetValue(TURNING_PID_MENU, 0);
    turn_pid.Ki = Menu_GetValue(TURNING_PID_MENU, 1);
    turn_pid.Kd = Menu_GetValue(TURNING_PID_MENU, 2);
}

/*
	下面的赋值放入中断函数中，turn_angle()函数前
*/
void turn_angle_start(uint16 angle)
{
	uint16_t present_angle = yaw;
	turn_pid.Target = present_angle + angle;
}

//定向转动 
/*
	使用方法:turn_angle_start(30);     //向右转30°，左右方向可能有误，到时候根据实际情况更改
	        turn_angle_pid(void)
*/
//后续跟队友代码结合后打算改成void类型，直接使用，不作用到PWM输出的计算里
//这里写上注释提醒一下自己
void turn_angle_pid(void)
{
	turn_pid.Actual = yaw;
	turn_pid.Error0 = turn_pid.Target - turn_pid.Actual;
	turn_pid.Error1 = turn_pid.Error0;
	
	if(turn_pid.Ki != 0){turn_pid.ErrorInt += turn_pid.Error0;}
	else{turn_pid.ErrorInt = 0;}
	
	turn_pid.Out = turn_pid.Kp * turn_pid.Error0 + turn_pid.Ki * turn_pid.ErrorInt 
					+ turn_pid.Kd * (turn_pid.Error0 - turn_pid.Error1);
	
	//输出限幅
	if(turn_pid.Out > turn_pid.OutMax){turn_pid.Out = turn_pid.OutMax;}
	if(turn_pid.Out < turn_pid.OutMin){turn_pid.Out = turn_pid.OutMin;}
	
	/*
	关于初始PWM，不是0，应该为平衡车PID的PWM值，先填0，倒时候在改
	*/
	if(turn_pid.Out != 0)
	{
		Moto_SetPWM(1,0 + turn_pid.Out/2);   
		Moto_SetPWM(2,0 - turn_pid.Out/2);
	}
	else
	{
		Moto_SetPWM(1,0);
		Moto_SetPWM(2,0);
	}
}
