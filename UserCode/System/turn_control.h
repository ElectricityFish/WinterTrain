#ifndef __TURN_CONTROL_H
#define __TURN_CONTROL_H

#include "zf_common_headfile.h"

// 角度位置环PID结构体（串级PID的外环）
typedef struct {
    float target;           // 目标角度
    float actual;           // 实际角度
    float output;           // 输出（目标转向速度）
    
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float Kd;               // 微分系数
    
    float error;            // 当前误差
    float last_error;       // 上次误差
    float error_integral;   // 误差积分
    float integral_max;     // 积分限幅
    
    float OutMax;           // 最大输出（最大转向速度）
    float OutMin;           // 最小输出（最小转向速度）
} Angle_Position_PID;

void Start_Angle_Turn(float angle);      // 开始角度转向
void Stop_Angle_Turn(void);              // 停止角度转向
uint8_t Update_Angle_Turn(void);         // 更新角度转向（在中断中调用）
uint8_t Is_Angle_Turning(void);          // 检查是否正在转向
float Get_Angle_Turn_Error(void);        // 获取当前角度误差

#endif
