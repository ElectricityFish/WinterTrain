/************************************************************************************************
* @name   转向环PID控制
* @note   目前大概只是一个框架，实际效果还不知道，我是指用法可以再改
		  转向环PID的参值与菜单相连
* @author 
*************************************************************************************************/
#include "zf_common_headfile.h"
#include "turn_control.h"
#include "menu.h"

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

//定向转动 
/*
	使用:左轮PWM+out值，右轮PWM-out值（先这么确定）
*/
float turn_angle(uint16_t angle)
{
	turn_pid.Error0 = angle;
	turn_pid.Error1 = turn_pid.Error0;
	
	if(turn_pid.Ki != 0){turn_pid.ErrorInt += turn_pid.Error0;}
	else{turn_pid.ErrorInt = 0;}
	
	turn_pid.Out = turn_pid.Kp * turn_pid.Error0 + turn_pid.Ki * turn_pid.ErrorInt 
	                + turn_pid.Kd * (turn_pid.Error0 - turn_pid.Error1);
	
	if(turn_pid.Out > turn_pid.OutMax){turn_pid.Out = turn_pid.OutMax;}
	if(turn_pid.Out < turn_pid.OutMin){turn_pid.Out = turn_pid.OutMin;}
	
	return turn_pid.Out;
}
