#include "zf_common_headfile.h"
#include <math.h>
#include <stdio.h>

// 卡尔曼滤波结构体
typedef struct {
    float Q_angle;   // 角度过程噪声协方差
    float Q_bias;    // 偏差过程噪声协方差
    float R_measure; // 测量噪声协方差
    
    float angle;     // 计算出的角度（滤波后）
    float bias;      // 陀螺仪偏差
    float rate;      // 未经滤波的角速度
    
    float P[2][2];   // 误差协方差矩阵
} KalmanFilter;

// 初始化卡尔曼滤波器
void Kalman_Init(KalmanFilter* kf,float Q_angle,float Q_bias,float R_measure) {//三个可调参数：角度过程噪声，零偏过程噪声，测量噪声
    // 设置过程噪声协方差
    kf->Q_angle = Q_angle;
    kf->Q_bias = Q_bias;
    kf->R_measure = R_measure;

    // 初始角度为0度
    kf->angle = 0.0f;
    kf->bias = 0.0f;
    
    // 初始化误差协方差矩阵
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

// 卡尔曼滤波计算
float Kalman_Calculate(KalmanFilter* kf, float newAngle, float newRate, float dt) {
    // 预测步骤
    // 1. 更新先验状态估计
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;
    
    // 2. 更新先验误差协方差
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;
    
    // 更新步骤
    // 1. 计算卡尔曼增益
    float S = kf->P[0][0] + kf->R_measure;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;
    
    // 2. 计算后验状态估计
    float y = newAngle - kf->angle;
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;
    
    // 3. 更新后验误差协方差
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;
    
    return kf->angle;
}

// 从加速度计数据计算角度
float getAccelAngle(float ax, float ay, float az) {
    // 使用atan2计算pitch角
    // 公式: pitch = atan2(-ax, sqrt(ay*ay + az*az))
    // 这里注意：根据MPU6050的安装方向，可能需要调整符号
    return -atan2(ax, sqrt(ay*ay + az*az)) * 180.0f / 3.14159;
}

/*
atan2(ax, sqrt(ay*ay + az*az)) * 180.0f / 3.14159相比于atan2(ax,az)可以解除x轴与y轴加速度的耦合
解耦机制：通过投影到Y-Z平面，将Roll角的影响从Pitch角计算中分离
数学保证：结果只依赖于Pitch角，与Roll角无关
物理正确：测量的是相对于真实水平面的角度，而非设备内部角度
*/

// 主函数：计算Pitch角
/**
 * @brief 使用卡尔曼滤波计算Pitch角度
 * @param ax MPU6050的X轴加速度原始数据
 * @param ay MPU6050的Y轴加速度原始数据（可选，如果不需要roll角可不使用）
 * @param az MPU6050的Z轴加速度原始数据
 * @param gy MPU6050的Y轴陀螺仪原始数据（pitch角速度）
 * @param dt 采样时间间隔（秒）
 * @param kf 卡尔曼滤波器指针
 * @return 滤波后的Pitch角度（度）
 */
float calculatePitchAngle(float ax, float ay, float az, float gy, float dt, KalmanFilter* kf) {
    
    float gyro_y = gy /32768.f*2000;   // 度/秒
    // 2. 从加速度计计算角度（测量值）
    // 3. 使用卡尔曼滤波融合数据
    return  Kalman_Calculate(kf, getAccelAngle(ax, ay, az), gyro_y, dt);
}



void GetOffset(float *Offset,float Angle,uint8_t Flag)
{
	static float AngleSum=0;
	AngleSum+=Angle;
	if(Flag==0){
		*Offset=AngleSum/100;
	}
}



/**
 * @brief 封装后的姿态解算函数
 * @param 关键数据定义在主函数
 * @return 无
 */
extern float gyro_yaw , gyro_pitch , gyro_roll ;
extern float acc_yaw , acc_pitch , acc_roll;
extern int16 AX, AY, AZ;
extern float Offset;
extern float yaw, pitch, roll;
extern KalmanFilter KF;
void Get_Angle(void)
{
	
		mpu6050_get_gyro();
		mpu6050_get_acc();
		
		//姿态解算，使用卡尔曼滤波算法
		
		//yaw角解算（无加速度计校准）
		gyro_yaw += (mpu6050_gyro_transition(mpu6050_gyro_z / 100 * 100) * 0.01);
		yaw = gyro_yaw;
		
		//pitch角解算（加速度计校准）
		//加速度计简陋滤波
		AX = mpu6050_acc_x / 100 * 100;
		AY = mpu6050_acc_y / 100 * 100;
		AZ = mpu6050_acc_z / 100 * 100;
		pitch=calculatePitchAngle(AX, AY, AZ, (mpu6050_gyro_y / 100 * 100) , 0.01, &KF)-Offset;
}



