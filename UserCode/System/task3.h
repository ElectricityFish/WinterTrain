#ifndef __TASK3_H
#define __TASK3_H

#include "zf_common_headfile.h"
#include "menu.h"
#include "PID.h"
#include "Sensor.h"
#include "turn_control.h"

/* ==============================================================================================
                                        宏定义
   ============================================================================================== */
#define TASK3_UPDATE_PERIOD_MS     15     // 任务更新周期（ms）
#define TASK3_TURN_ANGLE           35     // 转向角度（度）
#define TASK3_FORWARD_SPEED        1.5f   // 前进速度
#define TASK3_TURN_SPEED           0.0f   // 转向时速度
#define TASK3_LOST_DELAY_COUNT     10     // 断线延迟计数
#define TASK3_FORWARD_TIMEOUT      200    // 前进超时计数
#define TASK3_COMPLETE_DELAY       100    // 完成延迟计数

/* ==============================================================================================
                                        枚举类型定义
   ============================================================================================== */

// 绕8字状态机状态
typedef enum {
    TASK3_STATE_IDLE = 0,          // 空闲状态
    TASK3_STATE_TRACKING,          // 循迹状态
    TASK3_STATE_LOST_JUST,         // 刚检测到断线
    TASK3_STATE_TURNING,           // 正在转向
    TASK3_STATE_FORWARD_AFTER_TURN,// 转向后前进
    TASK3_STATE_COMPLETE           // 完成一圈
} task3_state_t;

/* ==============================================================================================
                                        结构体定义
   ============================================================================================== */

// 任务3控制结构体
typedef struct {
    task3_state_t state;           // 当前状态
    uint8_t lap_count;             // 已完成圈数
    uint8_t stop_flag;             // 断线计数
    int8_t turn_direction;         // 当前转向方向：1表示左转，-1表示右转
    uint16_t lost_delay_counter;   // 断线延迟计数器
    uint16_t forward_counter;      // 前进计数器
    uint16_t complete_delay;       // 完成延迟计数器
} task3_ctrl_t;

/* ==============================================================================================
                                        全局变量声明
   ============================================================================================== */

extern task3_ctrl_t task3_ctrl;

/* ==============================================================================================
                                        函数声明
   ============================================================================================== */

// 初始化任务3
void Task3_Init(void);

// 重置任务3
void Task3_Reset(void);

// 任务3状态机更新（在中断中调用）
// 参数：cur_track_state - 当前循迹状态（0正常，1刚断线，2持续断线）
//       CarMode - 当前小车模式
void Task3_Update(int cur_track_state, boot_mode CarMode);

// 获取任务3状态
task3_state_t Task3_GetState(void);

// 获取已完成的圈数
uint8_t Task3_GetLapCount(void);

// 获取断线计数
uint8_t Task3_GetStopFlag(void);

// 检查任务是否完成（完成4圈）
uint8_t Task3_IsComplete(void);

#endif
