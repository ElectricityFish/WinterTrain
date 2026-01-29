#include "zf_common_headfile.h"
#include "distance_control.h"
#include "menu.h"
#include "Encoder.h"
#include "Motor.h"
#include "PID.h"
#include <math.h>

// 外部变量
extern int16_t LeftPWM, RightPWM;
extern PID_t SpeedPID;
extern PID_t TurnPID;
extern boot_mode CarMode;
extern float SpeedLeft;   
extern float SpeedRight;  

#define PI_self 3.14159265358979323846f

// 车轮周长 (cm) = 2πr
static float wheel_perimeter = 2.0f * PI_self * WHEEL_RADIUS_CM;

// 距离相关变量(cm)
static float distance_right_cm = 0.0f;  // 右轮累计行驶距离(cm)
static float distance_left_cm = 0.0f;   // 左轮累计行驶距离(cm)
static float distance_cm = 0.0f;        // 平均累计距离(cm)

// 控制周期（ms
#define CONTROL_PERIOD_MS 10
#define CONTROL_PERIOD_S (CONTROL_PERIOD_MS / 1000.0f)

// 每脉冲对应距离 (cm)
static float distance_per_pulse(void)
{
    static float dpp = 0.0f;
    if (dpp == 0.0f) dpp = wheel_perimeter / (float)ENCODER_RESOLUTION;
    return dpp;
}


// 计算并返回当前累计平均距离（cm）
// 注意：不再直接读取硬件编码器计数，而用 pit_handler 里保存的 SpeedLeft/SpeedRight（每周期脉冲数）
float get_average_distance(void)
{
    // distance_per_pulse 为 cm / count
    float dpp = distance_per_pulse();

    // SpeedLeft/SpeedRight 为每周期（CONTROL_PERIOD_MS）读取到的脉冲增量（在 pit_handler 已赋值）
    // 把脉冲数换算成 cm 并累计
    float right_increment_cm = SpeedRight * dpp;
    float left_increment_cm  = SpeedLeft  * dpp;

    distance_right_cm += right_increment_cm;
    distance_left_cm  += left_increment_cm;
    distance_cm = (distance_right_cm + distance_left_cm) / 2.0f;

    return distance_cm;
}

//串级PID结构，内嵌速度环
//串级PID结构，内嵌速度环
Distance_Position_PID distance_pos_pid = {
    .Kp = 5.0f,     // 增大比例系数，让响应更快
    .Ki = 0.1f,     // 添加一点积分，消除静态误差
    .Kd = 2.0f,     // 添加微分，防止超调
    
    .OutMax = 4.0f,   // 增大最大输出速度
    .OutMin = -4.0f,  // 允许反向
};

uint8_t is_running = 0;                     //判断位置任务标志位
float target_distance_increment = 0;        //目标位置增量
float initial_distance = 0;                 //初始位置为0

// 串级PID外环：位置环PID计算
static void DistancePositionPID_Update(Distance_Position_PID *pid)
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

// 开始定位置移动任务
// distance: 目标位置增量，>0前进，<0后退
//使用方法：根据其他任务标志位在main中的while调用Start_Run_Distance(5);
void Start_Run_Distance(float distance)
{
    if (is_running) return; // 如果已经在移动中，直接返回
    
    is_running = 1;
    target_distance_increment = distance;
    initial_distance = get_average_distance();
    
    // 初始化位置环PID参数
    distance_pos_pid.target = initial_distance + distance;
    distance_pos_pid.actual = initial_distance;
    distance_pos_pid.error = 0;
    distance_pos_pid.last_error = 0;
    distance_pos_pid.error_integral = 0;
    distance_pos_pid.integral_max = 100.0f; // 可根据需要调整
    
    // 设置转向环目标为0，确保直行
    TurnPID.Target = 0.0f;
    
    // 初始速度设为0，由位置环逐渐加速
    SpeedPID.Target = 0.0f;
    
    // 重置速度环积分，防止历史积分影响
    SpeedPID.ErrorInt = 0;
    
}

// 停止位置移动任务
void Stop_Run_Distance(void)
{
    is_running = 0;
    SpeedPID.Target = 0.0f; // 停止转向
    distance_pos_pid.error_integral = 0; // 清空积分
}

// 位置移动任务更新函数（需要在中断中调用）
// 返回: 1-转向完成，0-转向中
uint8_t Update_Run_Distance(void)
{
    if (!is_running) return 1;
    
    // 更新当前位置
    distance_pos_pid.actual = get_average_distance();
    
    // 更新位置环PID
    DistancePositionPID_Update(&distance_pos_pid);
    
    // 位置环的输出作为速度环的目标
    // 注意：根据你的电机方向调整符号
    float target_speed = distance_pos_pid.output;
    
    // 添加一个死区，防止微小误差导致抖动
    if (fabs(target_speed) < 0.5f) 
    {
        target_speed = 0;
    }
    
    // 设置速度环目标
    SpeedPID.Target = target_speed;
    
    // 检查是否达到目标位置（带有死区）
    float distance_error = fabs(distance_pos_pid.target - distance_pos_pid.actual);
    if (distance_error < 0.5f) // 0.5cm误差范围内认为完成
    {
        // 逐渐减速到停止
        static float decel_factor = 1.0f;
        if (decel_factor > 0)
        {
            decel_factor -= 0.05f;
            if (decel_factor < 0) decel_factor = 0;
            SpeedPID.Target *= decel_factor;
            return 0; // 还在减速过程中
        }
        
        Stop_Run_Distance();
        return 1; // 移动完成
    }
    
    return 0; // 移动中
}


// 检查移动状态
uint8_t Is_Running(void)
{
    return is_running;
}

// 获取位置误差
float Get_Run_Distance_Error(void)
{
    return distance_pos_pid.error;
}
