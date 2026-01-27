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
#include "PID.h"
#include <math.h>

extern int16_t LeftPWM,RightPWM;
extern int16_t AvePWM,DifPWM;
extern PID_t TurnPID;
extern float yaw;

// 串级PID结构：该PID作为外环，内环嵌入TurnPID
Angle_Position_PID angle_pos_pid = {
    .Kp = 0.5f,      // 需要根据实际调试
    .Ki = 0.0f,
    .Kd = 0.0f,
    
    .OutMax = 50.0f,         // 最大输出角度速度（°/s）
    .OutMin = -50.0f,        // 最小输出角度速度
};

uint8_t is_angle_turning = 0;           // 角度转向任务标志位
float target_angle_increment = 0.0f;    // 目标角度增量
float initial_yaw = 0.0f;               // 开始转向时的初始角度

// 串级PID外环：角度位置环PID计算
static void AnglePositionPID_Update(Angle_Position_PID *pid)
{
    // 计算角度误差
    pid->error = pid->target - pid->actual;
    
    // 积分项（带积分限幅）
    if (pid->Ki != 0)
    {
        pid->error_integral += pid->error;
        // 积分限幅
        if (pid->error_integral > pid->integral_max)
            pid->error_integral = pid->integral_max;
        if (pid->error_integral < -pid->integral_max)
            pid->error_integral = -pid->integral_max;
    }
    
    // 微分项
    float error_diff = pid->error - pid->last_error;
    pid->last_error = pid->error;
    
    // PID计算
    pid->output = pid->Kp * pid->error + 
                  pid->Ki * pid->error_integral + 
                  pid->Kd * error_diff;
    
    // 输出限幅
    if (pid->output > pid->OutMax) pid->output = pid->OutMax;
    if (pid->output < pid->OutMin) pid->output = pid->OutMin;
}

// 开始角度转向任务
// angle: 目标角度增量，>0顺时针，<0逆时针
//使用方法：根据其他任务标志位在main中的while调用Start_Angle_Turn(30);
void Start_Angle_Turn(float angle)
{
    if (is_angle_turning) return; // 如果已经在转向中，直接返回
    
    is_angle_turning = 1;
    target_angle_increment = angle;
    initial_yaw = yaw;
    
    // 初始化角度位置环PID参数
    angle_pos_pid.target = initial_yaw + angle;
    angle_pos_pid.actual = initial_yaw;
    angle_pos_pid.error = 0;
    angle_pos_pid.last_error = 0;
    angle_pos_pid.error_integral = 0;
    angle_pos_pid.integral_max = 100.0f; // 可根据需要调整
    
    // 设置转向PID的目标速度为0，由外环控制
    TurnPID.Target = 0.0f;
}

// 停止角度转向任务
void Stop_Angle_Turn(void)
{
    is_angle_turning = 0;
    TurnPID.Target = 0.0f; // 停止转向
    angle_pos_pid.error_integral = 0; // 清空积分
}

// 角度转向任务更新函数（需要在中断中调用）
// 返回: 1-转向完成，0-转向中
uint8_t Update_Angle_Turn(void)
{
    if (!is_angle_turning) return 1;
    
    // 更新当前角度
    angle_pos_pid.actual = yaw;
    
    // 更新角度位置环PID
    AnglePositionPID_Update(&angle_pos_pid);
    
    // 角度位置环的输出作为转向速度环的目标
    // note：这里需要根据实际方向调整符号
    TurnPID.Target = -angle_pos_pid.output; // 负号可能需要根据实际转向方向调整
    
    // 检查是否达到目标角度（带有死区，防止震荡）
    float angle_error = fabs(angle_pos_pid.target - yaw);
    if (angle_error < 1.0f) // 1度误差范围内认为完成
    {
        Stop_Angle_Turn();
        return 1; // 转向完成
    }
    
    return 0; // 转向中
}

// 检查转向状态
uint8_t Is_Angle_Turning(void)
{
    return is_angle_turning;
}

// 获取角度误差
float Get_Angle_Turn_Error(void)
{
    return angle_pos_pid.error;
}
