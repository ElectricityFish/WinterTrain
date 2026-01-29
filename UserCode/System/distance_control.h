#ifndef __DISTANCE_CONTROL_H
#define __DISTANCE_CONTROL_H

#include "zf_common_headfile.h"

#define WHEEL_RADIUS_CM 3.4f
#define ENCODER_RESOLUTION 44  // 编码器分辨率

// 位置环PID结构体（串级PID的外环）
typedef struct {
    float target;           // 目标位置
    float actual;           // 实际位置
    float output;           // 输出（目标转向速度）
    
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float Kd;               // 微分系数
    
    float error;            // 当前误差
    float last_error;       // 上次误差
    float error_integral;   // 误差积分
    float integral_max;     // 积分限幅
    
    float OutMax;           // 最大输出（最大前进速度）
    float OutMin;           // 最小输出（最大后退速度）
} Distance_Position_PID;

extern uint8_t is_running;

void Start_Run_Distance(float distance);      // 开始移动
void Stop_Run_Distance(void);                 // 停止移动
uint8_t Update_Run_Distance(void);            // 更新位置（在中断中调用）
uint8_t Is_Running(void);                     //查是否正在移动
float Get_Run_Distance_Error(void);           // 获取当前位置误差

#endif
