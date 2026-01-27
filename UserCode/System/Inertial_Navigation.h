#ifndef __INERTIAL_NAVIGATION_H
#define __INERTIAL_NAVIGATION_H

#include "zf_common_headfile.h"

// 路径关键点结构体 - 用最小的数据记录路径
typedef struct {
    int16_t distance_cm;      // 累计行驶距离（厘米） - 使用有符号，前进为正
    int16_t angle_deg;        // 累计旋转角度（度）   - 使用有符号，右转为正
} PathPoint;

// 路径数据头结构体
typedef struct {
    uint16_t point_count;     // 关键点数量
    uint16_t checksum;        // 校验和
} PathHeader;

// 主状态枚举
typedef enum {
    NAV_IDLE = 0,            // 空闲
    NAV_RECORDING,           // 记录中
    NAV_REPLAYING,           // 回放中
    NAV_ERROR                // 错误
} NavigationState;

// 复现子状态枚举
typedef enum {
    REPLAY_TURNING,          // 正在转向
    REPLAY_MOVING,           // 正在直行
    REPLAY_COMPLETE,         // 完成
    REPLAY_ERROR             // 出错
} ReplaySubState;

// Flash配置 - 使用第100扇区存储路径数据
#define PATH_FLASH_SECTOR    100    // 存入第100扇区
#define MAX_PATH_POINTS      100    // 最多记录100个关键点
#define DISTANCE_THRESHOLD   5.0f   // 5厘米阈值 - 超过才记录点
#define ANGLE_THRESHOLD      5.0f   // 5度阈值   - 超过才记录点
#define ANGLE_TOLERANCE      2.0f   // 角度执行容差 ±2度
#define DISTANCE_TOLERANCE   1.0f   // 距离执行容差 ±1厘米

// 外部函数声明
void InertialNav_Init(void);
void InertialNav_StartRecording(void);
void InertialNav_StopRecording(void);
void InertialNav_StartReplay(void);
void InertialNav_StopReplay(void);
NavigationState InertialNav_GetState(void);
uint16_t InertialNav_GetPointCount(void);
bool InertialNav_HasStoredPath(void);
void InertialNav_ClearPath(void);
void InertialNav_Process(void);
void InertialNav_DisplayInfo(void);  // 新增：显示路径信息

#endif
