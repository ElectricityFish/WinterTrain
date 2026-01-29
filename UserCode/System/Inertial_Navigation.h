#ifndef __INERTIAL_NAVIGATION_H
#define __INERTIAL_NAVIGATION_H

#include "zf_common_headfile.h"

// ******************************** 常量定义 ********************************
#define PATH_FLASH_SECTOR    125    // 使用扇区125存储路径数据
#define MAX_PATH_POINTS      50     // 最多记录50个关键点

// 记录阈值（可根据实际情况调整）
#define DISTANCE_THRESHOLD   50     // 距离变化超过50个脉冲记录新点
#define ANGLE_THRESHOLD      5.0f   // 角度变化超过5度记录新点

// 执行容差
#define DISTANCE_TOLERANCE   10     // 距离容差±10个脉冲
#define ANGLE_TOLERANCE      2.0f   // 角度容差±2度

// ******************************** 类型定义 ********************************

// 路径点结构体
typedef struct {
    int32_t distance_pulse;   // 累计距离（编码器脉冲数）
    float   angle_deg;        // 偏航角（度）
} PathPoint;

// 导航状态枚举
typedef enum {
    NAV_IDLE = 0,            // 空闲状态
    NAV_RECORDING,           // 记录路径中
    NAV_REPLAYING,           // 回放路径中
    NAV_ERROR                // 错误状态
} NavigationState;

// ******************************** 外部变量声明 ********************************
extern NavigationState nav_state;  // 当前导航状态

// ******************************** 函数声明 ********************************

// 初始化函数
void InertialNav_Init(void);

// 控制函数
void InertialNav_StartRecording(void);  // 开始记录路径
void InertialNav_StopRecording(void);   // 停止记录并保存
void InertialNav_StartReplay(void);     // 开始回放路径
void InertialNav_StopReplay(void);      // 停止回放
void InertialNav_ClearPath(void);       // 清除存储的路径

// 状态查询函数
NavigationState InertialNav_GetState(void);     // 获取当前状态
uint16_t InertialNav_GetPointCount(void);       // 获取存储的点数
bool InertialNav_HasStoredPath(void);           // 检查是否有存储的路径

// 处理函数（需要在中断中调用）
void InertialNav_Process(void);                 // 每10ms调用一次
void InertialNav_FastProcess(void);             // 每1ms调用一次（可选）

#endif
