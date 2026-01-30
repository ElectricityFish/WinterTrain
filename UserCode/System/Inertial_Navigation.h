#ifndef __INERTIAL_NAVIGATION_H
#define __INERTIAL_NAVIGATION_H

#include "zf_common_headfile.h"

/* ==============================================================================================
                                        宏定义和枚举类型定义
   ============================================================================================== */

// Flash存储配置
#define INERTIAL_NAV_FLASH_SECTOR         100     // 惯导数据存储扇区
#define INERTIAL_NAV_FLASH_PAGE           0       // 惯导数据存储页
#define INERTIAL_NAV_MAX_POINTS           512     // 最大记录点数（每点8字节，一页512个点）
#define INERTIAL_NAV_RECORD_INTERVAL_MS   20      // 记录间隔20ms（50Hz采样）

// 记录数据类型枚举
typedef enum {
    INERTIAL_NAV_IDLE = 0,       // 空闲状态
    INERTIAL_NAV_RECORDING,      // 正在记录
    INERTIAL_NAV_REPLAYING,      // 正在回放
    INERTIAL_NAV_FINISHED        // 回放完成
} inertial_nav_state_t;

// 路径点数据结构
#pragma pack(push, 1)  // 1字节对齐，节省Flash空间
typedef struct {
    int16_t left_speed;          // 左轮速度（编码器计数）
    int16_t right_speed;         // 右轮速度（编码器计数）
    float   pitch;               // 俯仰角
    float   yaw;                 // 偏航角
    uint8_t flags;               // 标志位
} path_point_t;
#pragma pack(pop)

// 路径头信息结构
typedef struct {
    uint16_t    point_count;     // 路径点数
    uint16_t    record_time_ms;  // 记录总时间（毫秒）
    uint32_t    checksum;        // 校验和
    uint8_t     version;         // 版本号
    uint8_t     reserved[3];     // 保留字节
} path_header_t;

/* ==============================================================================================
                                        全局变量声明
   ============================================================================================== */

extern inertial_nav_state_t inertial_nav_state;
extern uint16_t inertial_nav_current_index;
extern uint16_t inertial_nav_total_points;
extern path_header_t inertial_nav_header;
extern path_point_t inertial_nav_current_point;

/* ==============================================================================================
                                        函数声明
   ============================================================================================== */

// 初始化函数
void Inertial_Nav_Init(void);

// 记录相关函数
void Inertial_Nav_StartRecord(void);
void Inertial_Nav_StopRecord(void);
void Inertial_Nav_SaveToFlash(void);
void Inertial_Nav_RecordPoint(void);

// 回放相关函数
void Inertial_Nav_StartReplay(void);
void Inertial_Nav_StopReplay(void);
void Inertial_Nav_LoadFromFlash(void);
uint8_t Inertial_Nav_GetNextPoint(void);

// 状态检查函数
uint8_t Inertial_Nav_IsRecording(void);
uint8_t Inertial_Nav_IsReplaying(void);
uint8_t Inertial_Nav_IsFinished(void);

// 数据获取函数
uint16_t Inertial_Nav_GetCurrentIndex(void);
uint16_t Inertial_Nav_GetTotalPoints(void);
uint16_t Inertial_Nav_GetRemainingTime(void);

// Flash操作函数
void Inertial_Nav_FlashErase(void);
uint8_t Inertial_Nav_FlashWriteData(uint32_t addr, const void* data, uint16_t size);
uint8_t Inertial_Nav_FlashReadData(uint32_t addr, void* data, uint16_t size);
uint32_t Inertial_Nav_CalculateChecksum(const void* data, uint16_t size);

#endif
