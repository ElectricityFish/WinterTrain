/************************************************************************************************
* @name   惯性导航系统
* @note   实现路径记录和回放功能
* @author 
*************************************************************************************************/
#include "zf_common_headfile.h"
#include "Inertial_Navigation.h"
#include "nav_flash.h"
#include <stdbool.h>

/**
 * ======= 纯跟踪算法可调参数区域 =======
 * 以下参数影响Pure Pursuit算法的性能和效率，可根据实际场景调整
 */
// 最近点查找范围（前后各多少个点）
// - 增大此值可以提高在大弯道或车辆偏离路径时的鲁棒性
// - 减小此值可以提高计算效率，适合路径简单或车辆偏离较小的场景
// - 推荐范围：10~50
#define LOOKAHEAD_SEARCH_RANGE 30

// 预瞄点最大搜索步数（从最近点向前搜索的最大点数）
// - 增大此值可以处理较大的预瞄距离，但会增加计算量
// - 减小此值可以提高效率，适合预瞄距离较小或路径点密集的场景
// - 推荐范围：10~30
#define LOOKAHEAD_MAX_STEPS 20

// 跳跃步长估算的平均点间距（单位：cm）
// - 此值应等于或接近路径点的实际间距
// - 若路径点每1cm记录一次，则设为1.0f
// - 若路径点间距不确定，建议保守设置为1.0f
// - 此值直接影响跳跃搜索的准确性和效率
#define LOOKAHEAD_AVG_DIST 1.0f

// 输出差速的经验比例系数
#define SCALE_FACTOR 80

/**
 * ===================================
 */

// 小车物理参数
#define distance (1.0) // 每间隔1cm记录一次点位
#define WHEELBASE 17   // 轮距（左右轮距离，单位cm）

float X_Memery_Plus[FLASH_DATA_BUFFER_SIZE * 6] = {0};
float Y_Memery_Plus[FLASH_DATA_BUFFER_SIZE * 6] = {0};
float X_Memery_Store_Plus[FLASH_DATA_BUFFER_SIZE * 6] = {0}; // 小车记忆路径（东天北坐标系）单位cm
float Y_Memery_Store_Plus[FLASH_DATA_BUFFER_SIZE * 6] = {0}; // 小车每走1cm记录一个路径点
volatile float X = 0, Y = 0;                                 // 小车实时位置（东天北坐标系）单位cm

uint8_t road_memery_finish_Plus_flag = 0; // 路径记忆完成标志位
uint8_t road_memery_start_Plus_flag = 0;  // 路径记忆开始标志位
uint8_t road_recurrent_Plus_flag = 0;     // 路径复现标志位
uint16_t NUM_L_Plus = 0, NUM_R_Plus = 0;  // 每走过1cm,NUM+1
uint16_t road_destination = 0;            // 路径终点

volatile int16 err_Nav_plus = 0;     // 惯导差速
volatile float e_lat = 0;            // 惯导横向误差
volatile int32_t speed_left = 0;     // 左轮速度（脉冲/5ms）
volatile int32_t speed_right = 0;    // 右轮速度（脉冲/5ms）
// 小车实时状态
volatile int locate_index = 0;           // 小车定位点
static int target_index = 0;             // 小车目标点
const int32_t MAX_PHYSICAL_DELTA = 1000; // 最大允许物理增量，用于防御性检查

volatile SlipFlags wheelSlipFlags = SLIP_FLAG_NONE;
volatile float leftWheelSpeedCmps = 0;
volatile float rightWheelSpeedCmps = 0;
volatile float currentAvgSpeedCmps = 0;

/**
 * @brief 轮子数据结构体，封装单个轮子的编码器相关数据
 *
 * @成员 total_pulses     累计脉冲数，用于虚拟清零功能
 * @成员 encoder_last     上一次读取的编码器值，用于计算增量
 * @成员 encoder_prev_speed 速度计算基准值，保存上一次中断时的编码器值
 * @成员 finish_flag;     达到设定阈值标志位
 * @成员 target_pulses    目标脉冲数，达到此值后触发虚拟清零（应设为正数）
 */

// 初始化左轮数据结构体
// - total_pulses: 初始累计脉冲数为0
// - last_total_pulses: 初始上一次脉冲数为0
// - encoder_last: 初始编码器值为0
// - encoder_prev_speed: 初始速度基准值为0
// - target_pulses: 虚拟清零阈值为366脉冲（需根据实际调整）

WheelData wheel_left = {
    .total_pulses = 0,
    .last_total_pulses = 0,
    .encoder_last = 0,
    .encoder_prev_speed = 0,
    .target_pulses = 366};

// 初始化右轮数据结构体（配置同左轮）
WheelData wheel_right = {
    .total_pulses = 0,
    .last_total_pulses = 0,
    .encoder_last = 0,
    .encoder_prev_speed = 0,
    .target_pulses = 366};

