#ifndef __DISTANCE_CONTROL_H
#define __DISTANCE_CONTROL_H

#include "zf_common_headfile.h"

typedef struct {
    float Target;
    float Actual;
    float Out;
    
    float Kp;
    float Ki;
    float Kd;
    
    float Error0;
    float Error1;
    float ErrorInt;
    
    float OutMax;
    float OutMin;
} Distance_PID;

// 声明为extern（不分配内存）
extern Distance_PID distance_pid;
extern uint8_t is_distance_control_enabled;
extern uint8_t is_distance_reached;

#define WHEEL_RADIUS_CM 3.4f
#define ENCODER_RESOLUTION 44  // 编码器分辨率

void Right_EncodeGet(void);
void Left_EncodeGet(void);
float Right_Distance(void);
float Left_Distance(void);
void distance_pid_UpdatePID(void);

// 位置环控制函数
void run_distance_start(float target_distance_cm);
void run_distance_pid(void);

// 新增：位置环串级控制函数
void distance_position_control(float target_distance_cm);
void distance_control_update(void);
uint8_t is_distance_control_completed(void);
void reset_distance_control(void);

#endif
