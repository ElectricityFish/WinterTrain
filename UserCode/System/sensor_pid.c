/************************************************************************************************
* @name   红外循迹的PID计算与差方转换模块
* @note   PID计算在这里！要注意的是这不是转向PID，而是循迹转向PID！
* @author MxBz_
*************************************************************************************************/
#include "zf_common_headfile.h"
#include "sensor.h"
#include "sensor_pid.h"
#include "menu.h"

/* ==============================================================================================
                                        全局变量定义
   ============================================================================================== */

double Sensor_Kp = 1.0f;
double Sensor_Ki = 1.0f;
double Sensor_Kd = 1.0f;

double integral = 0.0f;
double prev_error = 0.0f;

double integral_limit = INTEGRAL_LIMIT;
double output_limit = OUTPUT_MAX;

/* ==============================================================================================
                                        函数定义
   ============================================================================================== */
/**
 * @brief 更新PID值
 * @note 过渡
 * @return 
 */
void Sensor_pid_UpdatePID(void)
{
    Sensor_Kp = Menu_GetValue(SENSOR_PID_MENU, 0);
    Sensor_Ki = Menu_GetValue(SENSOR_PID_MENU, 1);
    Sensor_Kd = Menu_GetValue(SENSOR_PID_MENU, 2);
}

/**
 * @brief 由 `Sensor.h` 获得的 `Error` 计算 `差分值`
 * @note 这个函数应该被放在一个 10ms 左右的pit中反复调用。说实话，我也不知道这个返回的单位是什，但公式做题准没错，电机那边就是 Left_Motor_Speed +，Right_Motor_Speed - （正负号不确定哦）
 * @return 
 */
double Sensor_pid_calculate(void) 
{
    Sensor_pid_UpdatePID();
    int error = Sensor_GetError();
    // Proportional
    double proportional = Sensor_Kp * error;

    // Integral
    integral += DT * error;
    // Integral limit
    if (integral > integral_limit) integral = integral_limit;
    else if (integral < -integral_limit) integral = -integral_limit;
    
    double cur_integral = integral * Sensor_Ki;

    // Derivative
    double derivative = Sensor_Kd * (error - prev_error) / DT;

    // output
    double output = proportional + cur_integral + derivative;
    // output limit
    if (output > output_limit) output = output_limit;
    else if (output < -output_limit) output = -output_limit;

    prev_error = error;

    return output;
} 