/************************************************************************************************
* @name   惯性导航系统
* @note   对于惯性导航系统目前都是脑测写代码，相信肯定是有问题存在的
		  这里用到我写的转向环PID和位置环PID
		  要把记录的点位数据存入魏子尧的菜单flash里，复现时在依次把值赋值到对应环的target里
		  先转向环在位置环
		  两次的发车位置和朝向必须一致
* @author 
*************************************************************************************************/
#include "zf_common_headfile.h"
#include "turn_control.h"
#include "distance_control.h"
#include "Inertial_Navigation.h"
#include "zf_driver_flash.h"
#include "Encoder.h"
#include "Motor.h"
#include "menu.h"

extern float yaw;  // MPU6050的偏航角

// 静态全局变量
static NavigationState nav_state = NAV_IDLE;
static PathPoint path_points[MAX_PATH_POINTS];
static PathHeader path_header = {0};
static uint16_t current_point_index = 0;

// 记录时的累计值
static float accumulated_distance = 0.0f;
static float accumulated_angle = 0.0f;
static float last_record_distance = 0.0f;
static float last_record_angle = 0.0f;

// 回放时的目标值
static float target_distance = 0.0f;
static float target_angle = 0.0f;
static uint16_t replay_step = 0;

// 编码器原始值记录（用于计算变化量）
static int16_t last_left = 0;
static int16_t last_right = 0;

uint8_t record_flag = 0;

uint16_t previous_angle = 0;
uint16_t present_angle = 0;

uint16_t previous_distance = 0;
uint16_t present_distance = 0;

/**
 * @brief 读取编码器并计算距离变化（厘米）
 */
static float GetDistanceChange(void)
{
	float current_left = Left_Distance();
	float current_right = Right_Distance();
	
	//计算增量
	float delta_left = current_left - last_left;
	float delta_right = current_right - last_right;
	
	last_left = current_left;
	last_right = current_right;
	
	float ave_delta = (delta_left + delta_right) / 2.0f;
	
	return ave_delta;
}

