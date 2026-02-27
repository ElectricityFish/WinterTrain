/*********************************************************************************************************************
* 四元数姿态解算驱动
* 基于MPU6050陀螺仪和加速度计数据进行四元数更新（Mahony互补滤波）
* 适用于 MPU6050 等 IMU 传感器
********************************************************************************************************************/

#include "zf_common_typedef.h"
#include "zf_device_mpu6050.h"
#include "quaternion.h"
#include <math.h>

// 滤波器参数
#define SAMPLE_TIME_MS    1.0f
#define DEG_TO_RAD        0.017453292519943f

static Quaternion attitude = {1.0f, 0.0f, 0.0f, 0.0f};
static uint8_t initialized = 0;

/**
 * @brief 初始化四元数模块
 */
void quaternion_init(void) {
    attitude.q0 = 1.0f;
    attitude.q1 = 0.0f;
    attitude.q2 = 0.0f;
    attitude.q3 = 0.0f;
    initialized = 1;
}

/**
 * @brief 更新四元数姿态
 */
void quaternion_update(void)
{
    if(!initialized) return;

    //获取原始陀螺仪数据并转换为弧度/秒（°/s）
    float gx = mpu6050_gyro_transition(mpu6050_gyro_x / 100 * 100) * DEG_TO_RAD;
    float gy = mpu6050_gyro_transition(mpu6050_gyro_y / 100 * 100) * DEG_TO_RAD;
    float gz = mpu6050_gyro_transition(mpu6050_gyro_z / 100 * 100) * DEG_TO_RAD;

    //一阶龙格-库塔法积分
    float dt = SAMPLE_TIME_MS / 1000.0f;
    float q0 = attitude.q0;
    float q1 = attitude.q1;
    float q2 = attitude.q2;
    float q3 = attitude.q3;

    // 四元数微分方程
    attitude.q0 += (-q1*gx - q2*gy - q3*gz) * (0.5f * dt);
    attitude.q1 += ( q0*gx - q3*gy + q2*gz) * (0.5f * dt);
    attitude.q2 += ( q3*gx + q0*gy - q1*gz) * (0.5f * dt);
    attitude.q3 += (-q2*gx + q1*gy + q0*gz) * (0.5f * dt);

    //四元数归一化
    float norm = sqrtf(attitude.q0 * attitude.q0 + attitude.q1 * attitude.q1 +
                       attitude.q2 * attitude.q2 + attitude.q3 * attitude.q3);

    norm = 1.0f / norm;
    attitude.q0 *= norm;
    attitude.q1 *= norm;
    attitude.q2 *= norm;
    attitude.q3 *= norm;

    //转换为欧拉角（弧度）
    attitude.roll = atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * (1 / DEG_TO_RAD);
    attitude.pitch = asinf(2*(q0*q2 - q3*q1)) * (1 / DEG_TO_RAD);
    attitude.yaw = atan2f(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * (1 / DEG_TO_RAD);
}

Quaternion* get_eular_angles(void) 
{
    return &attitude;
}
