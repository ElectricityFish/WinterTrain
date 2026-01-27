/************************************************************************************************
* @name   位置环PID控制
* @note   串级PID控制：位置环(外环) -> 速度环(内环) -> 角度环(最内环)
* @author 
*************************************************************************************************/
#include "zf_common_headfile.h"
#include "distance_control.h"
#include "menu.h"
#include "Encoder.h"
#include "Motor.h"
#include "PID.h"
#include <math.h>

// 声明外部变量
extern int16_t LeftPWM, RightPWM;
extern PID_t SpeedPID;
extern boot_mode CarMode;

#define PI_self 3.14159265358979323846f

// 车轮周长 (cm) = 2πr
static float wheel_perimeter = 2.0f * PI_self * WHEEL_RADIUS_CM;

// 距离相关变量(cm)
static float distance_right_cm = 0.0f;  // 右轮行驶距离
static float distance_left_cm = 0.0f;   // 左轮行驶距离
static float distance_cm = 0.0f;        // 平均距离

// 编码器脉冲数
static uint16_t right_pulse = 0;
static uint16_t left_pulse = 0;

// 位置控制状态标志
uint8_t is_distance_control_enabled = 0;
uint8_t is_distance_reached = 0;

// 位置环PID结构体
Distance_PID distance_pid = {
    .Kp = 0.1f,     // 需要根据实际情况调整
    .Ki = 0.01f,    
    .Kd = 0.1f,     
    
    .OutMax = 5.0f,   // 最大输出速度(cm/s)
    .OutMin = -5.0f,  // 最小输出速度(cm/s)
    
    .Target = 0.0f,
    .Actual = 0.0f,
    .Out = 0.0f,
    .Error0 = 0.0f,
    .Error1 = 0.0f,
    .ErrorInt = 0.0f
};

// 编码器读取函数
void Right_EncodeGet(void)
{
    right_pulse = Get_Count1();
}

void Left_EncodeGet(void)
{
    left_pulse = Get_Count2();
}

// 计算距离函数（单位：cm）
// 每个脉冲对应的距离 = 车轮周长 / 编码器分辨率
float Right_Distance(void)
{
    float distance_per_pulse = wheel_perimeter / ENCODER_RESOLUTION;
    distance_right_cm += (right_pulse * distance_per_pulse);
    
    return distance_right_cm;
}

float Left_Distance(void)
{
    float distance_per_pulse = wheel_perimeter / ENCODER_RESOLUTION;
    distance_left_cm += (left_pulse * distance_per_pulse);
    
    return distance_left_cm;
}

/*
 * 函数：启动位置控制
 * 参数：target_distance_cm - 目标距离(cm)，正数前进，负数后退
 */
void run_distance_start(float target_distance_cm)
{
    // 重置距离累计
    distance_right_cm = 0.0f;
    distance_left_cm = 0.0f;
    distance_cm = 0.0f;
    
    // 设置目标距离
    distance_pid.Target = target_distance_cm;
    distance_pid.Actual = 0.0f;
    distance_pid.ErrorInt = 0.0f;  // 重置积分项
    
    // 启用位置控制
    is_distance_control_enabled = 1;
    is_distance_reached = 0;
}

/*
 * 函数：位置环PID计算（串级控制的外环）
 * 说明：计算位置误差，输出目标速度给速度环
 */
void run_distance_pid(void)
{
    if (!is_distance_control_enabled || is_distance_reached) {return;}
    
    //获取当前平均距离
    distance_cm = (Right_Distance() + Left_Distance()) / 2.0f;
    distance_pid.Actual = distance_cm;
    
    //计算位置误差
    distance_pid.Error0 = distance_pid.Target - distance_pid.Actual;
    
    //计算PID输出（目标速度）
    if (distance_pid.Ki != 0) 
    {
        distance_pid.ErrorInt += distance_pid.Error0;
        
        // 积分限幅
        if (distance_pid.ErrorInt > distance_pid.OutMax / distance_pid.Ki) {
            distance_pid.ErrorInt = distance_pid.OutMax / distance_pid.Ki;
        }
        if (distance_pid.ErrorInt < distance_pid.OutMin / distance_pid.Ki) {
            distance_pid.ErrorInt = distance_pid.OutMin / distance_pid.Ki;
        }
    } 
    else 
    {
        distance_pid.ErrorInt = 0;
    }
    
    // PID计算：位置环输出 = 目标速度(cm/s)
    distance_pid.Out = distance_pid.Kp * distance_pid.Error0 
                     + distance_pid.Ki * distance_pid.ErrorInt 
                     + distance_pid.Kd * (distance_pid.Error0 - distance_pid.Error1);
    
    //输出限幅
    if (distance_pid.Out > distance_pid.OutMax) 
    {
        distance_pid.Out = distance_pid.OutMax;
    }
    if (distance_pid.Out < distance_pid.OutMin) 
    {
        distance_pid.Out = distance_pid.OutMin;
    }
    
    //设置速度环的目标值（串级控制的关键）
    SpeedPID.Target = distance_pid.Out;
    
    //保存本次误差
    distance_pid.Error1 = distance_pid.Error0;
    
    //检查是否到达目标位置   判断标准：误差小于0.5cm
    float position_error = fabs(distance_pid.Error0);
    if (position_error < 0.5f) 
    {
        is_distance_reached = 1;
        SpeedPID.Target = 0.0f;  // 停止运动
        distance_pid.Out = 0.0f;
    }
}

/*
 * 函数：完整的位置控制函数
 * 参数：target_distance_cm - 目标距离(cm)
 */
void distance_position_control(float target_distance_cm)
{
    reset_distance_control();
    run_distance_start(target_distance_cm);
}

/*
 * 函数：位置控制更新（需要在主循环或定时中断中调用）
 */
void distance_control_update(void)
{
    if (!is_distance_control_enabled) {
        return;
    }
    
    // 获取编码器数据
    Right_EncodeGet();
    Left_EncodeGet();
    
    // 运行位置环PID
    run_distance_pid();
}

/*
 * 函数：检查位置控制是否完成
 * 返回：1-完成，0-未完成
 */
uint8_t is_distance_control_completed(void)
{
    return is_distance_reached;
}

/*
 * 函数：重置位置控制
 */
void reset_distance_control(void)
{
    is_distance_control_enabled = 0;
    is_distance_reached = 0;
    distance_pid.Out = 0.0f;
    distance_pid.ErrorInt = 0.0f;
    
    // 重置距离累计
    distance_right_cm = 0.0f;
    distance_left_cm = 0.0f;
    distance_cm = 0.0f;
}
