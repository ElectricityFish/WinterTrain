/************************************************************************************************
* @name   惯性导航系统（改进版）
* @note   结合navigation.c的思路，简化实现记录和复现功能
* @author 
*************************************************************************************************/
#include "zf_common_headfile.h"
#include "turn_control.h"
#include "distance_control.h"
#include "Inertial_Navigation.h"
#include "zf_driver_flash.h"
#include "zf_driver_delay.h"
#include "Encoder.h"
#include "Motor.h"
#include "menu.h"
#include <math.h>

extern float yaw;  // MPU6050的偏航角

// ******************************** 全局变量 ********************************
static NavigationState nav_state = NAV_IDLE;
static PathPoint path_points[MAX_PATH_POINTS];
static uint16_t point_count = 0;       // 实际存储的点数
static uint16_t replay_index = 0;      // 回放时当前执行的点索引

// 记录模式变量
static float last_yaw = 0.0f;          // 上次记录的偏航角
static int32_t last_distance = 0;      // 上次记录的累计距离（脉冲数）

// 回放模式变量
static float replay_start_yaw = 0.0f;  // 回放起始偏航角
static int32_t replay_start_distance = 0;  // 回放起始距离

// ******************************** 辅助函数 ********************************

/**
 * @brief 获取当前累计距离（左右编码器平均值）
 * @return 累计脉冲数
 */
static int32_t GetCurrentDistance(void)
{
    int32_t left_count = (int32_t)encoder_get_count(ENCODER_QUADDEC1);
    int32_t right_count = (int32_t)encoder_get_count(ENCODER_QUADDEC2);
    return (left_count + right_count) / 2;
}

/**
 * @brief 计算两点之间的距离差（欧几里得距离）
 */
static float CalculateDistanceDiff(PathPoint* p1, PathPoint* p2)
{
    float delta_dist = (float)(p2->distance_pulse - p1->distance_pulse);
    return delta_dist;
}

/**
 * @brief 计算两点之间的角度差
 */
static float CalculateAngleDiff(PathPoint* p1, PathPoint* p2)
{
    float delta_angle = p2->angle_deg - p1->angle_deg;
    // 角度规范化到[-180, 180]
    while(delta_angle > 180.0f) delta_angle -= 360.0f;
    while(delta_angle < -180.0f) delta_angle += 360.0f;
    return delta_angle;
}

// ******************************** Flash操作函数 ********************************

/**
 * @brief 保存路径到Flash
 * @return true-成功, false-失败
 */
static bool SavePathToFlash(void)
{
    if(point_count == 0) return false;
    
    // 使用扇区125存储路径数据
    flash_buffer_clear();
    
    // 第0个数据存储点数
    flash_union_buffer[0].uint16_type = point_count;
    
    // 存储每个路径点（距离和角度打包存储）
    for(uint16_t i = 0; i < point_count && i < FLASH_DATA_BUFFER_SIZE/2; i++)
    {
        // 距离（32位）和角度（16位）打包存储
        flash_union_buffer[2*i + 1].int32_type = path_points[i].distance_pulse;
        flash_union_buffer[2*i + 2].int16_type[0] = (int16_t)(path_points[i].angle_deg * 100); // 放大100倍存储
    }
    
    // 擦除并写入第0页
    if(flash_check(PATH_FLASH_SECTOR, 0))
    {
        flash_erase_page(PATH_FLASH_SECTOR, 0);
    }
    
    return !flash_write_page_from_buffer(PATH_FLASH_SECTOR, 0);
}

/**
 * @brief 从Flash读取路径
 * @return true-成功, false-失败
 */
static bool LoadPathFromFlash(void)
{
    flash_read_page_to_buffer(PATH_FLASH_SECTOR, 0);
    
    // 读取点数
    point_count = flash_union_buffer[0].uint16_type;
    if(point_count == 0 || point_count > MAX_PATH_POINTS) 
    {
        point_count = 0;
        return false;
    }
    
    // 读取路径点数据
    for(uint16_t i = 0; i < point_count && i < FLASH_DATA_BUFFER_SIZE/2; i++)
    {
        path_points[i].distance_pulse = flash_union_buffer[2*i + 1].int32_type;
        path_points[i].angle_deg = flash_union_buffer[2*i + 2].int16_type[0] / 100.0f;
    }
    
    return true;
}

/**
 * @brief 擦除路径数据
 */
static void ErasePathData(void)
{
    if(flash_check(PATH_FLASH_SECTOR, 0))
    {
        flash_erase_page(PATH_FLASH_SECTOR, 0);
    }
    point_count = 0;
}

// ******************************** 公共接口函数 ********************************

/**
 * @brief 初始化惯性导航模块
 */
void InertialNav_Init(void)
{
    nav_state = NAV_IDLE;
    point_count = 0;
    replay_index = 0;
    
    // 尝试加载已存储的路径
    LoadPathFromFlash();
    
    // 重置编码器计数
    Encoder_Clear();
}

/**
 * @brief 开始记录路径
 * @note 按下按键开始记录
 */
void InertialNav_StartRecording(void)
{
    if(nav_state != NAV_IDLE) return;
    
    nav_state = NAV_RECORDING;
    
    // 重置记录变量
    point_count = 0;
    last_yaw = yaw;
    last_distance = GetCurrentDistance();
    
    // 记录起始点
    if(point_count < MAX_PATH_POINTS)
    {
        path_points[point_count].distance_pulse = 0;
        path_points[point_count].angle_deg = 0;
        point_count++;
    }
    
    // 蜂鸣器提示
    // Buzzer_Beep(100); // 根据你的蜂鸣器函数调整
}

/**
 * @brief 停止记录并保存路径
 * @note 按下按键停止记录
 */
void InertialNav_StopRecording(void)
{
    if(nav_state != NAV_RECORDING) return;
    
    nav_state = NAV_IDLE;
    
    // 保存到Flash
    SavePathToFlash();
    
    // 蜂鸣器提示
    // Buzzer_Beep(200); // 根据你的蜂鸣器函数调整
}

/**
 * @brief 开始回放路径
 * @note 按下按键开始回放
 */
void InertialNav_StartReplay(void)
{
    if(nav_state != NAV_IDLE) return;
    
    // 确保有路径数据
    if(point_count == 0)
    {
        if(!LoadPathFromFlash() || point_count == 0)
        {
            // 没有路径数据
            // Buzzer_Beep(500); // 错误提示
            return;
        }
    }
    
    nav_state = NAV_REPLAYING;
    replay_index = 0;
    
    // 记录回放起始状态
    replay_start_yaw = yaw;
    replay_start_distance = GetCurrentDistance();
    
    // 蜂鸣器提示
    // Buzzer_Beep(100);
    // system_delay_ms(100);
    // Buzzer_Beep(100);
}

/**
 * @brief 停止回放
 */
void InertialNav_StopReplay(void)
{
    if(nav_state != NAV_REPLAYING) return;
    
    nav_state = NAV_IDLE;
    
    // 停止所有运动
    Stop_Angle_Turn();
    Stop_Run_Distance();
    
    // 蜂鸣器提示
    // Buzzer_Beep(300);
}

/**
 * @brief 清除存储的路径
 */
void InertialNav_ClearPath(void)
{
    if(nav_state == NAV_IDLE)
    {
        ErasePathData();
        point_count = 0;
        // Buzzer_Beep(1000); // 长鸣提示清除完成
    }
}

/**
 * @brief 获取当前导航状态
 */
NavigationState InertialNav_GetState(void)
{
    return nav_state;
}

/**
 * @brief 获取存储的点数
 */
uint16_t InertialNav_GetPointCount(void)
{
    return point_count;
}

// ******************************** 核心处理函数 ********************************

/**
 * @brief 记录路径处理函数（每10ms调用一次）
 */
static void ProcessRecording(void)
{
    // 获取当前状态
    int32_t current_distance = GetCurrentDistance();
    float current_yaw = yaw;
    
    // 计算变化量
    int32_t distance_diff = current_distance - last_distance;
    float angle_diff = current_yaw - last_yaw;
    
    // 角度规范化
    while(angle_diff > 180.0f) angle_diff -= 360.0f;
    while(angle_diff < -180.0f) angle_diff += 360.0f;
    
    // 检查是否需要记录新点
    if(fabs(angle_diff) > ANGLE_THRESHOLD || abs(distance_diff) > DISTANCE_THRESHOLD)
    {
        // 记录新点
        if(point_count < MAX_PATH_POINTS)
        {
            path_points[point_count].distance_pulse = current_distance;
            path_points[point_count].angle_deg = current_yaw;
            point_count++;
            
            // 更新上次记录值
            last_distance = current_distance;
            last_yaw = current_yaw;
        }
        else
        {
            // 达到最大点数，自动停止
            InertialNav_StopRecording();
        }
    }
}

/**
 * @brief 回放路径处理函数（每10ms调用一次）
 */
static void ProcessReplaying(void)
{
    if(replay_index >= point_count)
    {
        // 所有点都执行完成
        InertialNav_StopReplay();
        return;
    }
    
    // 获取当前目标点
    PathPoint target_point = path_points[replay_index];
    
    // 计算当前位置（相对于回放起始点）
    int32_t current_distance = GetCurrentDistance() - replay_start_distance;
    float current_yaw = yaw - replay_start_yaw;
    
    // 角度规范化
    while(current_yaw > 180.0f) current_yaw -= 360.0f;
    while(current_yaw < -180.0f) current_yaw += 360.0f;
    
    // 检查是否到达目标点（使用容差判断）
    float distance_error = target_point.distance_pulse - current_distance;
    float angle_error = target_point.angle_deg - current_yaw;
    
    // 角度误差规范化
    while(angle_error > 180.0f) angle_error -= 360.0f;
    while(angle_error < -180.0f) angle_error += 360.0f;
    
    // 先转向到目标角度
    if(fabs(angle_error) > ANGLE_TOLERANCE)
    {
        // 使用转向环控制转向
        Start_Angle_Turn(angle_error);
    }
    else
    {
        // 角度已达到，停止转向
        Stop_Angle_Turn();
        
        // 然后直行到目标距离
        if(fabs(distance_error) > DISTANCE_TOLERANCE)
        {
            // 使用距离环控制直行
            Start_Run_Distance(distance_error);
        }
        else
        {
            // 距离也已达到，移动到下一个点
            Stop_Run_Distance();
            replay_index++;
        }
    }
}

/**
 * @brief 惯性导航主处理函数（在pit_handler中每10ms调用一次）
 */
void InertialNav_Process(void)
{
    // 只在平衡模式下工作
    if(CarMode != BALANCE_MODE)  // 需要根据你的实际模式定义调整
    {
        if(nav_state != NAV_IDLE)
        {
            InertialNav_StopRecording();
            InertialNav_StopReplay();
        }
        return;
    }
    
    switch(nav_state)
    {
        case NAV_RECORDING:
            ProcessRecording();
            break;
            
        case NAV_REPLAYING:
            ProcessReplaying();
            break;
            
        case NAV_IDLE:
        case NAV_ERROR:
        default:
            // 空闲状态，不做处理
            break;
    }
}

/**
 * @brief 在中断中快速处理（每1ms调用一次）
 */
void InertialNav_FastProcess(void)
{
    // 这里可以处理一些需要快速响应的逻辑
    // 比如急停检测等
}
