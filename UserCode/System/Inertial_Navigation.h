#ifndef __INERTIAL_NAVIGATION_H
#define __INERTIAL_NAVIGATION_H

#include "zf_common_headfile.h"
#include <stdbool.h>

extern uint8_t road_memery_finish_plus_flag;     // 路径记忆完成标志位
extern uint8_t road_memery_start_plus_flag;      // 路径记忆开始标志位
extern uint8_t road_recurrent_plus_flag;         // 路径复现标志位

extern uint16_t NUM_L_Plus, NUM_R_Plus;
extern volatile float x, y;
extern float X_Memery_Plus[FLASH_DATA_BUFFER_SIZE * 6];
extern float Y_Memery_Plus[FLASH_DATA_BUFFER_SIZE * 6];
extern float X_Memery_Store_Plus[FLASH_DATA_BUFFER_SIZE * 6];
extern float Y_Memery_Store_Plus[FLASH_DATA_BUFFER_SIZE * 6];

typedef struct
{
    volatile int32_t total_pulses;             // 累计脉冲数（可能为负，表示反向转动）
    volatile int32_t last_total_pulses;        // 上一次的累计脉冲数，用于检测跨阈值
    volatile int16_t encoder_last;             // 上一次编码器值（用于Update_Wheel_Pulses）
    volatile int16_t encoder_prev_speed;       // 速度计算基准值（用于Get_Wheel_Speed）
    const int32_t target_pulses;               // 虚拟清零阈值（常量，初始化后不可修改）
} WheelData;

extern WheelData wheel_left, wheel_right;
extern volatile int32_t speed_left;
extern volatile int32_t speed_right;
extern volatile int16_t err_Nav_plus;
extern volatile float e_lat;

/*打滑检测配置参数*/
#define WHEEL_BASE_CM  15.0f                    // 左右轮间距（单位：厘米）
#define ENCODER_PULSES_PER_REVOLUTION    366    // 每1cm编码器脉冲数
#define SAMPLING_INTERVAL_MS    5               // 数据采样间隔（单位：毫秒）与电机中断周期有关

/* 横向打滑检测阈值 */
#define LATERAL_SLIP_YAW_RATE_THRESHOLD 10.0f // 偏航角速度偏差阈值（单位：弧度/秒）
#define MIN_LATERAL_DETECT_SPEED_CMPS 200.0f  // 最低横向检测速度（单位：厘米/秒）

/* 纵向打滑检测阈值 */
#define LONGITUDINAL_ACCEL_DIFF_THRESHOLD 1.0f  // 加速度差异阈值（单位：米/秒?）
#define MIN_LONGITUDINAL_DETECT_SPEED_CMPS 1.0f // 最低纵向检测速度（单位：米/秒）

/* 滑移标志位掩码 */
typedef enum
{
    SLIP_FLAG_NONE = 0x00,               // 无滑移
    SLIP_FLAG_LATERAL_LEFT = 0x01,       // 左侧横向滑移（左轮打滑）
    SLIP_FLAG_LATERAL_RIGHT = 0x02,      // 右侧横向滑移（右轮打滑）
    SLIP_FLAG_LONGITUDINAL_ACCEL = 0x04, // 纵向加速滑移（驱动轮空转）
    SLIP_FLAG_LONGITUDINAL_DECEL = 0x08  // 纵向减速滑移（轮胎抱死）
} SlipFlags;

/* 外部变量声明 */
extern volatile SlipFlags wheelSlipFlags; // 滑移状态标志位

extern volatile float leftWheelSpeedCmps;  // 左轮线速度（单位：厘米/秒）
extern volatile float rightWheelSpeedCmps; // 右轮线速度（单位：厘米/秒）
extern volatile float currentAvgSpeedCmps; // 当前平均轮速（单位：米 /秒）
extern volatile int locate_index;          // 小车定位点
extern uint16_t road_destination;

void Distance_Get_Plus(void);
int32_t Get_Wheel_Speed(WheelData *wheel, int16_t current_encoder);
int32_t Update_Wheel_Pulses(WheelData *wheel, int16_t current_encoder);
void Slip_Check(void);
int16_t pure_pursuit_control(void);

#endif
