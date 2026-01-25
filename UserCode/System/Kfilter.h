#include "zf_common_headfile.h"
#ifndef __KFILTER_H
#define __KFILTER_H

typedef struct {
    float Q_angle;   // 角度过程噪声协方差
    float Q_bias;    // 偏差过程噪声协方差
    float R_measure; // 测量噪声协方差
    
    float angle;     // 计算出的角度（滤波后）
    float bias;      // 陀螺仪偏差
    float rate;      // 未经滤波的角速度
    
    float P[2][2];   // 误差协方差矩阵
} KalmanFilter;

void Kalman_Init(KalmanFilter* kf,float Q_angle,float Q_bias,float R_measure);
float calculatePitchAngle(float ax, float ay, float az, float gy, float dt, KalmanFilter* kf);
float getAccelAngle(float ax, float ay, float az);
void GetOffset(float *Offset,float Angle, uint8_t Flag);
void Get_Angle(void);//封装后的姿态解算函数
#endif
