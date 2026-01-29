/************************************************************************************************
* @name   惯性导航系统
* @note   实现路径记录和回放功能
* @author 
*************************************************************************************************/
#include "zf_common_headfile.h"
#include "Inertial_Navigation.h"
#include "Menu.h"
#include "Encoder.h"
#include "PID.h"
#include "Motor.h"

/* ==============================================================================================
                                        全局变量定义
   ============================================================================================== */

inertial_nav_state_t inertial_nav_state = INERTIAL_NAV_IDLE;
uint16_t inertial_nav_current_index = 0;
uint16_t inertial_nav_total_points = 0;
path_header_t inertial_nav_header = {0};
path_point_t inertial_nav_current_point = {0};

extern float yaw;
extern float pitch;

extern uint32_t system_time_ms;

static uint32_t record_start_time = 0;
static uint32_t last_record_time = 0;
static path_point_t point_buffer[INERTIAL_NAV_MAX_POINTS];

/* ==============================================================================================
                                        内部函数定义
   ============================================================================================== */
//获取系统时间
uint32_t get_system_time_ms(void)
{
    return system_time_ms;
}

/**
 * @brief 计算数据校验和
 * @param data 数据指针
 * @param size 数据大小
 * @return 校验和
 */
uint32_t Inertial_Nav_CalculateChecksum(const void* data, uint16_t size)
{
    const uint8_t* ptr = (const uint8_t*)data;
    uint32_t checksum = 0;
    
    for(uint16_t i = 0; i < size; i++) {
        checksum += ptr[i];
    }
    
    return checksum;
}

/**
 * @brief 擦除Flash数据
 */
void Inertial_Nav_FlashErase(void)
{
    if(flash_check(INERTIAL_NAV_FLASH_SECTOR, INERTIAL_NAV_FLASH_PAGE)) {
        flash_erase_page(INERTIAL_NAV_FLASH_SECTOR, INERTIAL_NAV_FLASH_PAGE);
    }
}

/**
 * @brief 写入Flash数据
 * @param addr 相对地址（从扇区起始地址偏移）
 * @param data 数据指针
 * @param size 数据大小
 * @return 0-成功，1-失败
 */
uint8_t Inertial_Nav_FlashWriteData(uint32_t addr, const void* data, uint16_t size)
{
    // 计算Flash绝对地址
    uint32_t flash_addr = FLASH_BASE_ADDR + 
                         (INERTIAL_NAV_FLASH_SECTOR * FLASH_SECTION_SIZE) +
                         (INERTIAL_NAV_FLASH_PAGE * FLASH_PAGE_SIZE) +
                         addr;
    
    // 临时缓冲区
    static uint32_t temp_buffer[FLASH_DATA_BUFFER_SIZE];
    
    // 读取当前页数据到缓冲区
    flash_read_page_to_buffer(INERTIAL_NAV_FLASH_SECTOR, INERTIAL_NAV_FLASH_PAGE);
    
    // 复制到临时缓冲区
    memcpy(temp_buffer, flash_union_buffer, FLASH_PAGE_SIZE);
    
    // 修改数据
    memcpy((uint8_t*)temp_buffer + addr, data, size);
    
    // 擦除页
    flash_erase_page(INERTIAL_NAV_FLASH_SECTOR, INERTIAL_NAV_FLASH_PAGE);
    
    // 写入新数据
    return flash_write_page(INERTIAL_NAV_FLASH_SECTOR, INERTIAL_NAV_FLASH_PAGE, 
                           temp_buffer, FLASH_DATA_BUFFER_SIZE);
}

/**
 * @brief 读取Flash数据
 * @param addr 相对地址
 * @param data 数据指针
 * @param size 数据大小
 * @return 0-成功，1-失败
 */
uint8_t Inertial_Nav_FlashReadData(uint32_t addr, void* data, uint16_t size)
{
    // 计算Flash绝对地址
    uint32_t flash_addr = FLASH_BASE_ADDR + 
                         (INERTIAL_NAV_FLASH_SECTOR * FLASH_SECTION_SIZE) +
                         (INERTIAL_NAV_FLASH_PAGE * FLASH_PAGE_SIZE) +
                         addr;
    
    // 读取数据
    flash_read_page_to_buffer(INERTIAL_NAV_FLASH_SECTOR, INERTIAL_NAV_FLASH_PAGE);
    memcpy(data, (uint8_t*)flash_union_buffer + addr, size);
    
    return 0;
}

/* ==============================================================================================
                                        外部函数定义
   ============================================================================================== */

/**
 * @brief 惯导系统初始化
 */
void Inertial_Nav_Init(void)
{
    inertial_nav_state = INERTIAL_NAV_IDLE;
    inertial_nav_current_index = 0;
    inertial_nav_total_points = 0;
    memset(&inertial_nav_header, 0, sizeof(path_header_t));
    memset(&inertial_nav_current_point, 0, sizeof(path_point_t));
    memset(point_buffer, 0, sizeof(point_buffer));
}

/**
 * @brief 开始记录路径
 */
void Inertial_Nav_StartRecord(void)
{
    if(inertial_nav_state != INERTIAL_NAV_IDLE) {
        return;
    }
    
    // 初始化变量
    inertial_nav_state = INERTIAL_NAV_RECORDING;
    inertial_nav_current_index = 0;
    record_start_time = get_system_time_ms();
    last_record_time = record_start_time;
    
    // 清空点缓冲区
    memset(point_buffer, 0, sizeof(point_buffer));
    
    // 记录第一个点（初始状态）
    Inertial_Nav_RecordPoint();
    
    // 蜂鸣器提示开始记录
    // Buzzer_Beep(100);
}

/**
 * @brief 记录路径点
 */
void Inertial_Nav_RecordPoint(void)
{
    if(inertial_nav_state != INERTIAL_NAV_RECORDING) {
        return;
    }
    
    uint32_t current_time = get_system_time_ms();
    
    // 检查记录间隔
    if((current_time - last_record_time) < INERTIAL_NAV_RECORD_INTERVAL_MS) {
        return;
    }
    
    // 检查缓冲区是否已满
    if(inertial_nav_current_index >= INERTIAL_NAV_MAX_POINTS) {
        Inertial_Nav_StopRecord();
        return;
    }
    
    // 获取当前状态
    path_point_t point;
    point.left_speed = Get_Count1();      // 左编码器
    point.right_speed = Get_Count2();     // 右编码器
    point.pitch = pitch;                  // 俯仰角
    point.yaw = yaw;                      // 偏航角
    point.flags = 0;
    
    // 编码器清零（在中断中已清零，这里只读取）
    // 注意：编码器在主循环中每10ms清零一次
    
    // 保存到缓冲区
    point_buffer[inertial_nav_current_index] = point;
    inertial_nav_current_index++;
    
    last_record_time = current_time;
}

/**
 * @brief 停止记录路径
 */
void Inertial_Nav_StopRecord(void)
{
    if(inertial_nav_state != INERTIAL_NAV_RECORDING) {
        return;
    }
    
    inertial_nav_state = INERTIAL_NAV_IDLE;
    
    // 保存到Flash
    Inertial_Nav_SaveToFlash();
    
    // 蜂鸣器提示记录完成
    // Buzzer_Beep(200);
    // systick_delay_ms(100);
    // Buzzer_Beep(200);
}

/**
 * @brief 保存路径到Flash
 */
void Inertial_Nav_SaveToFlash(void)
{
    if(inertial_nav_current_index == 0) {
        return;
    }
    
    // 准备头信息
    inertial_nav_header.point_count = inertial_nav_current_index;
    inertial_nav_header.record_time_ms = get_system_time_ms() - record_start_time;
    inertial_nav_header.version = 1;
    inertial_nav_header.checksum = 0;
    
    // 计算校验和（不包括头信息）
    inertial_nav_header.checksum = Inertial_Nav_CalculateChecksum(
        point_buffer, 
        inertial_nav_current_index * sizeof(path_point_t)
    );
    
    // 擦除Flash
    Inertial_Nav_FlashErase();
    
    // 写入头信息
    Inertial_Nav_FlashWriteData(0, &inertial_nav_header, sizeof(path_header_t));
    
    // 写入路径点数据
    Inertial_Nav_FlashWriteData(
        sizeof(path_header_t),
        point_buffer,
        inertial_nav_current_index * sizeof(path_point_t)
    );
    
    // 保存总点数
    inertial_nav_total_points = inertial_nav_current_index;
}

/**
 * @brief 从Flash加载路径
 */
void Inertial_Nav_LoadFromFlash(void)
{
    // 读取头信息
    Inertial_Nav_FlashReadData(0, &inertial_nav_header, sizeof(path_header_t));
    
    // 检查版本
    if(inertial_nav_header.version != 1) {
        // 无效数据
        inertial_nav_total_points = 0;
        return;
    }
    
    // 读取路径点数据
    inertial_nav_total_points = inertial_nav_header.point_count;
    if(inertial_nav_total_points > INERTIAL_NAV_MAX_POINTS) {
        inertial_nav_total_points = INERTIAL_NAV_MAX_POINTS;
    }
    
    Inertial_Nav_FlashReadData(
        sizeof(path_header_t),
        point_buffer,
        inertial_nav_total_points * sizeof(path_point_t)
    );
    
    // 验证校验和
    uint32_t calculated_checksum = Inertial_Nav_CalculateChecksum(
        point_buffer,
        inertial_nav_total_points * sizeof(path_point_t)
    );
    
    if(calculated_checksum != inertial_nav_header.checksum) {
        // 校验和错误
        inertial_nav_total_points = 0;
        return;
    }
}

/**
 * @brief 开始回放路径
 */
void Inertial_Nav_StartReplay(void)
{
    if(inertial_nav_state != INERTIAL_NAV_IDLE) {
        return;
    }
    
    // 从Flash加载路径
    Inertial_Nav_LoadFromFlash();
    
    if(inertial_nav_total_points == 0) {
        // 没有可回放的路径
        return;
    }
    
    // 初始化回放状态
    inertial_nav_state = INERTIAL_NAV_REPLAYING;
    inertial_nav_current_index = 0;
    
    // 获取第一个点
    Inertial_Nav_GetNextPoint();
    
    // 蜂鸣器提示开始回放
    // Buzzer_Beep(300);
}

/**
 * @brief 停止回放路径
 */
void Inertial_Nav_StopReplay(void)
{
    if(inertial_nav_state != INERTIAL_NAV_REPLAYING) {
        return;
    }
    
    inertial_nav_state = INERTIAL_NAV_FINISHED;
    
    // 停止电机
    Motor_SetPWM(1, 0);
    Motor_SetPWM(2, 0);
    
    // 蜂鸣器提示回放完成
    // Buzzer_Beep(400);
    // systick_delay_ms(100);
    // Buzzer_Beep(400);
}

/**
 * @brief 获取下一个路径点
 * @return 1-成功，0-失败（路径结束）
 */
uint8_t Inertial_Nav_GetNextPoint(void)
{
    if(inertial_nav_state != INERTIAL_NAV_REPLAYING) {
        return 0;
    }
    
    if(inertial_nav_current_index >= inertial_nav_total_points) {
        Inertial_Nav_StopReplay();
        return 0;
    }
    
    // 获取当前路径点
    inertial_nav_current_point = point_buffer[inertial_nav_current_index];
    inertial_nav_current_index++;
    
    // 设置PID目标值
    // 注意：这里需要根据实际情况调整控制策略
    
    return 1;
}

/**
 * @brief 检查是否正在记录
 * @return 1-正在记录，0-否
 */
uint8_t Inertial_Nav_IsRecording(void)
{
    return (inertial_nav_state == INERTIAL_NAV_RECORDING);
}

/**
 * @brief 检查是否正在回放
 * @return 1-正在回放，0-否
 */
uint8_t Inertial_Nav_IsReplaying(void)
{
    return (inertial_nav_state == INERTIAL_NAV_REPLAYING);
}

/**
 * @brief 检查是否完成回放
 * @return 1-已完成，0-否
 */
uint8_t Inertial_Nav_IsFinished(void)
{
    return (inertial_nav_state == INERTIAL_NAV_FINISHED);
}

/**
 * @brief 获取当前索引
 * @return 当前索引
 */
uint16_t Inertial_Nav_GetCurrentIndex(void)
{
    return inertial_nav_current_index;
}

/**
 * @brief 获取总点数
 * @return 总点数
 */
uint16_t Inertial_Nav_GetTotalPoints(void)
{
    return inertial_nav_total_points;
}

/**
 * @brief 获取剩余时间
 * @return 剩余时间（毫秒）
 */
uint16_t Inertial_Nav_GetRemainingTime(void)
{
    if(inertial_nav_state != INERTIAL_NAV_REPLAYING) {
        return 0;
    }
    
    uint16_t remaining_points = inertial_nav_total_points - inertial_nav_current_index;
    return remaining_points * INERTIAL_NAV_RECORD_INTERVAL_MS;
}
