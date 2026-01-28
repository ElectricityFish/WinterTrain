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

// 位置控制状态标志
uint8_t is_distance_control_enabled = 0;
uint8_t is_distance_reached = 0;

// 位置控制目标
static float target_distance_cm = 0.0f;
// 判断标志位
static uint8_t position_control_active = 0;

// 外环 PID
Distance_PID distance_pid = {
    .Kp = 0.2f,     // 调参用，单位: (cm/s)
    .Ki = 0.05f,
    .Kd = 0.5f,
    
    .OutMax = 4.0f,   // 例如最大 4 cm/s（请按实际需要调）
    .OutMin = -4.0f,  // 允许反向
    .Target = 0.0f,
    .Actual = 0.0f,
    .Out = 0.0f,
    .Error0 = 0.0f,
    .Error1 = 0.0f,
    .ErrorInt = 0.0f
};

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

// 启动位置控制
void start_position_control(float distance_target_cm)
{
    if (position_control_active) return;

    target_distance_cm = distance_target_cm;

    // 重置累计距离
    distance_right_cm = 0.0f;
    distance_left_cm = 0.0f;
    distance_cm = 0.0f;

    // 重置 PID 状态
    distance_pid.Target = target_distance_cm;
    distance_pid.Actual = 0.0f;
    distance_pid.ErrorInt = 0.0f;
    distance_pid.Error0 = 0.0f;
    distance_pid.Error1 = 0.0f;
    distance_pid.Out = 0.0f;

    // 启用控制
    is_distance_control_enabled = 1;
    is_distance_reached = 0;
    position_control_active = 1;

    // 清速率环积分，避免历史影响
    SpeedPID.ErrorInt = 0.0f;
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

// 更新位置环 PID（外环），并把期望速度转换为 SpeedPID.Target（counts per control period）
void update_position_pid(void)
{
    if (!position_control_active || is_distance_reached) {
        return;
    }

    // 更新累计距离（使用 SpeedLeft/Right）
    float current_distance = get_average_distance();
    distance_pid.Actual = current_distance;

    // 误差（cm）
    distance_pid.Error0 = distance_pid.Target - distance_pid.Actual;

    // 到达判断（死区）
    float error_abs = fabsf(distance_pid.Error0);
    const float POSITION_DEADBAND_CM = 0.5f; // 可以调，小于该值视为到达
    if (error_abs < POSITION_DEADBAND_CM) {
        // 进入稳定判断：把速度目标设 0，等待几周期稳定后结束
        is_distance_reached = 1;
        distance_pid.Out = 0.0f;

        // 停止速度环输出
        SpeedPID.Target = 0.0f;
        SpeedPID.ErrorInt = 0.0f;

        static uint16_t stable_count = 0;
        stable_count++;
        if (stable_count > 50) { // 50 * CONTROL_PERIOD_MS = 500ms 稳定后关闭控制
            reset_distance_control();
            stable_count = 0;
        }
        return;
    }

    // 积分管理（只在接近目标时累积积分，防止远距离积分风暴）
    const float INTEGRAL_ENABLE_RANGE_CM = 10.0f; // 在 10cm 范围内开启积分
    if (fabsf(distance_pid.Error0) < INTEGRAL_ENABLE_RANGE_CM && distance_pid.Ki != 0.0f) 
	{
        distance_pid.ErrorInt += distance_pid.Error0;
        // 积分限幅（基于输出限幅）
        float max_integral = 0.0f;
        if (distance_pid.Ki != 0.0f) 
		{
            max_integral = distance_pid.OutMax / distance_pid.Ki;
            if (max_integral < 0.0f) max_integral = -max_integral;
        }
        if (max_integral > 0.0f) 
		{
            if (distance_pid.ErrorInt > max_integral) distance_pid.ErrorInt = max_integral;
            if (distance_pid.ErrorInt < -max_integral) distance_pid.ErrorInt = -max_integral;
        }
    } 
	else 
	{
        distance_pid.ErrorInt = 0.0f;
    }

    // PID 计算（输出单位：cm/s）
    float P = distance_pid.Kp * distance_pid.Error0;
    float I = distance_pid.Ki * distance_pid.ErrorInt;
    float D = distance_pid.Kd * (distance_pid.Error0 - distance_pid.Error1);

    float desired_speed_cm_per_s = P + I + D;

    // 输出限幅（cm/s）
    if (desired_speed_cm_per_s > distance_pid.OutMax) desired_speed_cm_per_s = distance_pid.OutMax;
    if (desired_speed_cm_per_s < distance_pid.OutMin) desired_speed_cm_per_s = distance_pid.OutMin;

    distance_pid.Out = desired_speed_cm_per_s;

    // 把外环的 cm/s 转换为 SpeedPID.Target 的单位
    float dpp = distance_per_pulse(); // cm / count
    float desired_counts_per_period = 0.0f;
    if (dpp > 0.0f) 
	{
        desired_counts_per_period = (desired_speed_cm_per_s / dpp) * CONTROL_PERIOD_S;
    }

    // 将转换后的目标赋给速度环（SpeedPID）
    SpeedPID.Target = desired_counts_per_period;

    distance_pid.Error1 = distance_pid.Error0;
}

// 外部接口：启动位置控制
void distance_position_control(float target_distance_cm)
{
    start_position_control(target_distance_cm);
}

// 外部接口：在主循环或 pit_handler 中周期调用
void distance_control_update(void)
{
    if (!is_distance_control_enabled) return;
    update_position_pid();
}

// 重置位置控制
void reset_distance_control(void)
{
    is_distance_control_enabled = 0;
    is_distance_reached = 0;
    position_control_active = 0;
    distance_pid.Out = 0.0f;
    distance_pid.ErrorInt = 0.0f;
    distance_pid.Error0 = 0.0f;
    distance_pid.Error1 = 0.0f;

    // 重置距离累计
    distance_right_cm = 0.0f;
    distance_left_cm = 0.0f;
    distance_cm = 0.0f;

    // 清速度环积分与目标
    SpeedPID.ErrorInt = 0.0f;
    SpeedPID.Target = 0.0f;
}

// 查询是否完成
uint8_t is_distance_control_completed(void)
{
    return is_distance_reached;
}
