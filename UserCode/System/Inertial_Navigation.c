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
#include "zf_driver_delay.h"
#include "Encoder.h"
#include "Motor.h"
#include "menu.h"
#include <math.h>

extern float yaw;  // MPU6050的偏航角

extern Distance_PID distance_pid;
extern Turn_PID turn_pid;

// 静态全局变量
// 主状态和子状态
static NavigationState nav_state = NAV_IDLE;
static ReplaySubState replay_substate = REPLAY_COMPLETE;
static PathPoint path_points[MAX_PATH_POINTS];
static PathHeader path_header = {0};
static uint16_t current_point_index = 0;

// 记录模式变量
static float accumulated_distance = 0.0f;      // 累计行驶距离
static float accumulated_angle = 0.0f;         // 累计旋转角度
static float last_record_distance = 0.0f;      // 上次记录的距离
static float last_record_angle = 0.0f;         // 上次记录的角度

// 复现模式变量
static uint16_t current_replay_index = 0;      // 当前执行的点索引
static PathPoint current_target_point;         // 当前目标点
static float replay_accumulated_distance = 0.0f;  // 复现累计距离
static float replay_accumulated_angle = 0.0f;     // 复现累计角度

// 编码器原始值记录（用于计算变化量）
static int16_t last_left = 0;
static int16_t last_right = 0;

/**
 * @brief 计算校验和  //用于检测数据的读出和记录是否一致，防止数据突变
 */
static uint16_t CalculateChecksum(void) {
    uint16_t sum = 0;
    uint8_t *data = (uint8_t*)path_points;
    uint32_t size = path_header.point_count * sizeof(PathPoint);
    
    for(uint32_t i = 0; i < size; i++) {
        sum += data[i];
    }
    return sum;
}

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

/**
 * @brief 保存路径到Flash
 * @note 使用扇区125的第0-3页存储路径数据
 */

static bool SavePathToFlash(void)
{
	// 计算校验和
    path_header.checksum = CalculateChecksum();
    
	/*=== 第一页：存储头部信息 ===*/
	flash_buffer_clear();
	
	//将点位和校验和存入
	//0：point_count, 1：checksum
	flash_union_buffer[0].uint16_type = path_header.point_count;
	flash_union_buffer[1].uint16_type = path_header.checksum;
	
	//擦除并写入第一页
	if(flash_check(PATH_FLASH_SECTOR,0))
	{
		flash_erase_page(PATH_FLASH_SECTOR,0);
	}
	if(flash_write_page_from_buffer(PATH_FLASH_SECTOR,0))  //写入扇区并返回判断值
	{
		return false;   //写入失败
	}
	
	/*=== 第2-4页：存储路径点数据 === */
	uint32_t total_points = path_header.point_count;
	uint32_t page = 1;   //从第一页开始存储
	
	//每个页可以存储 FLASH_DATA_BUFFER_SIZE 个路径点
	for(uint32_t i = 0; i < total_points; )
	{
		flash_buffer_clear();
		uint32_t points_in_this_page = 0;
		
		//填充当前页的缓冲区
		while(points_in_this_page < FLASH_DATA_BUFFER_SIZE && i < total_points)
		{
			//将距离和角度存入到32位数据中
			//低位：distance， 高位：angle
			uint32_t packed_data = ((uint32_t)(path_points[i].distance_cm) & 0xFFFF) | (((uint32_t)(path_points[i].angle_deg) & 0xFFFF) << 16);
			
			flash_union_buffer[points_in_this_page].int32_type = packed_data;
			points_in_this_page ++;
			i ++;
		}
	
		//擦除并写入当页
		if(flash_check(PATH_FLASH_SECTOR, page))
		{
			flash_erase_page(PATH_FLASH_SECTOR, page);
		}
		if(flash_write_page_from_buffer(PATH_FLASH_SECTOR,page))
		{
			return false;
		}
		
		page ++;
		
		//检查是否超过可用页数
		if(page >= FLASH_MAX_PAGE_INDEX)
		{
			break;
		}
	}
	
	return true;
}

/**
 * @brief 从Flash读取路径
 * @return true-成功, false-失败
 */
static bool LoadPathFromFlash(void) 
{
    // === 第一页：读取头部信息 ===
    flash_read_page_to_buffer(PATH_FLASH_SECTOR, 0);
    
    // 解析头部信息
    path_header.point_count = flash_union_buffer[0].uint16_type;
    path_header.checksum = flash_union_buffer[1].uint16_type;
    
    // 检查点数是否合理
    if(path_header.point_count == 0 || path_header.point_count > MAX_PATH_POINTS) 
    {
        return false;
    }
    
    // === 第2-4页：读取路径点数据 ===
    uint32_t total_points = path_header.point_count;
    uint32_t page = 1;  // 从第1页开始读取
    uint32_t point_index = 0;
    
    for(uint32_t i = 0; i < total_points; ) 
    {
        flash_read_page_to_buffer(PATH_FLASH_SECTOR, page);
        
        // 解析当前页的路径点
        uint32_t points_in_this_page = 0;
        while(points_in_this_page < FLASH_DATA_BUFFER_SIZE && i < total_points) 
        {
            uint32_t packed_data = flash_union_buffer[points_in_this_page].uint32_type;
            
            // 解包数据
            path_points[point_index].distance_cm = (int16_t)(packed_data & 0xFFFF);
            path_points[point_index].angle_deg = (int16_t)((packed_data >> 16) & 0xFFFF);
            
            points_in_this_page++;
            i++;
            point_index++;
        }
        
        page++;
        
        // 检查是否超过可用页数
        if(page >= FLASH_MAX_PAGE_INDEX) 
        {
            break;
        }
    }
    
    // 验证校验和
    uint16_t calculated_checksum = CalculateChecksum();
    if(calculated_checksum != path_header.checksum) {
        path_header.point_count = 0;  // 校验失败，清空点数
        return false;
    }
    
    return true;
}

/**
 * @brief 擦除整个路径扇区
 */
static void ErasePathSector(void) 
{
    for(uint8_t page = 0; page < FLASH_MAX_PAGE_INDEX; page++)
    {
        if(flash_check(PATH_FLASH_SECTOR, page)) 
        {
            flash_erase_page(PATH_FLASH_SECTOR, page);
        }
    }
}

/**
 * @brief 初始化惯性导航模块
 */
void InertialNav_Init(void) 
{
    nav_state = NAV_IDLE;
    replay_substate = REPLAY_COMPLETE;
    path_header.point_count = 0;
    current_replay_index = 0;
    
    // 重置所有累计值
    accumulated_distance = 0.0f;
    accumulated_angle = 0.0f;
    last_record_distance = 0.0f;
    last_record_angle = 0.0f;
    replay_accumulated_distance = 0.0f;
    replay_accumulated_angle = 0.0f;
    
    // 尝试加载已存储的路径
    LoadPathFromFlash();  // 不检查返回值，只是加载到内存
}

/**
 * @brief 开始记录路径
 */
void InertialNav_StartRecording(void) 
{
    if(nav_state != NAV_IDLE){return;}
    
    nav_state = NAV_RECORDING;
    replay_substate = REPLAY_COMPLETE;
    
    // 重置所有记录
    path_header.point_count = 0;
    accumulated_distance = 0.0f;
    accumulated_angle = 0.0f;
    last_record_distance = 0.0f;
    last_record_angle = 0.0f;
    
    // 擦除之前的路径数据
    ErasePathSector();
}

/**
 * @brief 停止记录并保存
 */
void InertialNav_StopRecording(void) {
    if(nav_state != NAV_RECORDING){return;}
    
    // 如果最后一点有未保存的数据，保存它
    if(fabs(accumulated_distance - last_record_distance) > 0.1f || fabs(accumulated_angle - last_record_angle) > 0.1f)
    {
        if(path_header.point_count < MAX_PATH_POINTS) 
		{
            path_points[path_header.point_count].distance_cm = (int16_t)accumulated_distance;
            path_points[path_header.point_count].angle_deg = (int16_t)accumulated_angle;
            path_header.point_count++;
        }
    }
    
    // 保存到Flash
    bool save_success = false;
    if(path_header.point_count > 0) 
	{
        save_success = SavePathToFlash();
    }
    
    nav_state = NAV_IDLE;
    system_delay_ms(1000);
}

/**
 * @brief 开始回放路径
 */
void InertialNav_StartReplay(void) 
{
    if(nav_state != NAV_IDLE) {return; }
    
    // 从Flash加载路径
    if(!LoadPathFromFlash()) 
	{
        nav_state = NAV_ERROR;
        system_delay_ms(1000);
        return;
    }
    
    if(path_header.point_count == 0) 
	{
        nav_state = NAV_ERROR;
        system_delay_ms(1000);
        return;
    }
    
    nav_state = NAV_REPLAYING;
    replay_substate = REPLAY_TURNING;
    current_replay_index = 0;
    
    // 重置复现累计值
    replay_accumulated_distance = 0.0f;
    replay_accumulated_angle = 0.0f;
    
    // 获取第一个目标点
    current_target_point = path_points[0];    
}

/**
 * @brief 停止回放
 */
void InertialNav_StopReplay(void) 
{
    if(nav_state != NAV_REPLAYING) 
	{
        return;
    }
    
    // 停止电机
    Moto_SetPWM(1, 0);
    Moto_SetPWM(2, 0);
    
    nav_state = NAV_IDLE;
    replay_substate = REPLAY_COMPLETE;
   
    system_delay_ms(500);
}

/**
 * @brief 获取当前状态
 */
NavigationState InertialNav_GetState(void) 
{
    return nav_state;
}

/**
 * @brief 获取路径点数
 */
uint16_t InertialNav_GetPointCount(void) 
{
    return path_header.point_count;
}

/**
 * @brief 检查是否有存储的路径
 */
bool InertialNav_HasStoredPath(void) 
{
    return flash_check(PATH_FLASH_SECTOR, 0);
}
/**
 * @brief 清除存储的路径数据
 */
void InertialNav_ClearPath(void) 
{
    if(nav_state == NAV_IDLE) 
	{
        ErasePathSector();
        path_header.point_count = 0;        
        system_delay_ms(1000);
    }
}

/**
 * @brief 记录模式处理
 */
static void ProcessRecording(void) 
{
    // 获取传感器数据变化
    float distance_change = GetDistanceChange();
    float angle_change = yaw;  // yaw是角度变化
    
    // 更新累计值
    accumulated_distance += distance_change;
    accumulated_angle += angle_change;
    
    // 检查是否超过阈值需要记录
    if(fabs(accumulated_distance - last_record_distance) > DISTANCE_THRESHOLD || fabs(accumulated_angle - last_record_angle) > ANGLE_THRESHOLD)    
	{
        // 保存关键点
        if(path_header.point_count < MAX_PATH_POINTS) 
		{
            path_points[path_header.point_count].distance_cm = (int16_t)accumulated_distance;
            path_points[path_header.point_count].angle_deg = (int16_t)accumulated_angle;
            path_header.point_count++;
            
            // 更新最后记录点
            last_record_distance = accumulated_distance;
            last_record_angle = accumulated_angle;
            
        } 
		else 
		{
            // 达到最大点数，自动停止
            InertialNav_StopRecording();
        }
    }
}

/**
 * @brief 复现模式处理
 */
static void ProcessReplaying(void) 
{
    replay_accumulated_distance = 0.0f;  // 重置距离累计
    replay_accumulated_angle = 0.0f;  // 重置角度累计    

    switch(replay_substate) 
	{
        case REPLAY_TURNING: {
            // 1. 先旋转到目标角度
            float angle_error = current_target_point.angle_deg - replay_accumulated_angle;
            
            // 使用转向环PID
            turn_pid.Target = current_target_point.angle_deg;
            turn_pid.Actual = replay_accumulated_angle;
            turn_angle_start(angle_error);
			turn_angle_pid();
            
            // 更新当前角度（从MPU6050获取）
            replay_accumulated_angle = yaw;
            
            // 检查是否达到目标角度
            if(fabs(angle_error) < ANGLE_TOLERANCE) 
            {
                // 角度达到，开始直行
                replay_substate = REPLAY_MOVING;               
                //replay_accumulated_distance = 0.0f;  // 重置距离累计
            }
            break;
        }
        
        case REPLAY_MOVING: {
            // 2. 直行到目标距离
            float distance_error = current_target_point.distance_cm - replay_accumulated_distance;
            
            // 使用位置环PID
            distance_pid.Target = current_target_point.distance_cm;
            distance_pid.Actual = replay_accumulated_distance;
            run_distance_start(distance_error);
			run_distance_pid();
            
            // 更新当前距离
            replay_accumulated_distance += GetDistanceChange();
            
            // 检查是否达到目标距离
            if(fabs(distance_error) < DISTANCE_TOLERANCE) 
			{
                // 当前关键点完成！
                current_replay_index++;
                
                // 检查是否所有点都完成
                if(current_replay_index >= path_header.point_count) 
				{
                    // 所有点完成
                    replay_substate = REPLAY_COMPLETE;                                  
                    // 停止电机
                    Moto_SetPWM(1, 0);
                    Moto_SetPWM(2, 0);
                    
                    // 自动回到空闲状态
                    nav_state = NAV_IDLE;                   
                } 
				else 
				{
                    // 准备下一个点
                    current_target_point = path_points[current_replay_index];
                    replay_substate = REPLAY_TURNING;
                    //replay_accumulated_angle = 0.0f;  // 重置角度累计                  
                }
            }
            break;
        }
        
        case REPLAY_COMPLETE:
            // 已完成，无需处理
            break;
            
        case REPLAY_ERROR:
            // 出错状态
            break;
    }

}

/**
 * @brief 惯性导航主处理函数 - 需要在主循环中定期调用
 */
void InertialNav_Process(void) 
{
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
            // 空闲或错误状态，无需处理
            break;
    }
}
