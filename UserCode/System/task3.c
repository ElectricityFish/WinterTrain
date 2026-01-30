#include "task3.h"
#include "Motor.h"
#include "Encoder.h"
#include "turn_control.h"
#include <math.h>

/* ==============================================================================================
                                        全局变量定义
   ============================================================================================== */

// 外部变量声明
extern PID_t SpeedPID;
extern PID_t TurnPID;
extern PID_t SensorPID;
extern float gyro_yaw;
extern double speed;

// 任务3控制结构体
task3_ctrl_t task3_ctrl;

/* ==============================================================================================
                                        内部函数定义
   ============================================================================================== */

// 初始化任务3状态
static void Task3_InitState(void)
{
    task3_ctrl.state = TASK3_STATE_IDLE;
    task3_ctrl.lap_count = 0;
    task3_ctrl.stop_flag = 0;
    task3_ctrl.turn_direction = -1;  // 初始方向设为左转
    task3_ctrl.lost_delay_counter = 0;
    task3_ctrl.forward_counter = 0;
    task3_ctrl.complete_delay = 0;
}

// 开始循迹模式
static void Task3_StartTracking(void)
{
    task3_ctrl.state = TASK3_STATE_TRACKING;
    SpeedPID.Target = TASK3_FORWARD_SPEED;
    speed = TASK3_FORWARD_SPEED;
    SensorPID.Ki = 0.0f;
}

// 处理断线检测
static void Task3_HandleLostTrack(void)
{
    task3_ctrl.state = TASK3_STATE_LOST_JUST;
    task3_ctrl.stop_flag++;
    
    // 根据当前圈数和方向决定转向角度
    if(task3_ctrl.lap_count % 2 == 0) // 偶数圈
    {
        task3_ctrl.turn_direction = (task3_ctrl.stop_flag % 2 == 1) ? 1 : -1;
    }
    else // 奇数圈
    {
        task3_ctrl.turn_direction = (task3_ctrl.stop_flag % 2 == 1) ? -1 : 1;
    }
}

// 开始转向
static void Task3_StartTurning(void)
{
    gyro_yaw = 0.0f;
//    TurnPID.Target = 0.0f;
    SpeedPID.Target = TASK3_TURN_SPEED;
    speed = TASK3_TURN_SPEED;
    
    // 开始转向
    if(task3_ctrl.turn_direction > 0)
    {
        Start_Angle_Turn(TASK3_TURN_ANGLE); // 左转
    }
    else
    {
        Start_Angle_Turn(-TASK3_TURN_ANGLE); // 右转
    }
    
    task3_ctrl.state = TASK3_STATE_TURNING;
}

// 转向后前进
static void Task3_ForwardAfterTurn(void)
{
    SpeedPID.Target = TASK3_FORWARD_SPEED;
    speed = TASK3_FORWARD_SPEED;
    task3_ctrl.state = TASK3_STATE_FORWARD_AFTER_TURN;
    task3_ctrl.forward_counter = 0;
}

// 完成一圈处理
static void Task3_HandleLapComplete(void)
{
    task3_ctrl.lap_count++;
    
    // 检查是否完成4圈
    if(task3_ctrl.lap_count >= 4)
    {
        task3_ctrl.state = TASK3_STATE_COMPLETE;
    }
    else
    {
        task3_ctrl.state = TASK3_STATE_TRACKING;
    }
}

// 任务完成处理
static void Task3_HandleComplete(void)
{
    SpeedPID.Target = 0.0f;
    TurnPID.Target = 0.0f;
    speed = 0.0f;
	
    task3_ctrl.complete_delay++;
    if(task3_ctrl.complete_delay > TASK3_COMPLETE_DELAY)
    {
        Menu_SetRunningMode(MODE_1);
        Task3_Reset();
    }
}

/* ==============================================================================================
                                        外部函数定义
   ============================================================================================== */

// 初始化任务3
void Task3_Init(void)
{
    Task3_InitState();
}

// 重置任务3
void Task3_Reset(void)
{
    Task3_InitState();
}

// 任务3状态机更新
void Task3_Update(int cur_track_state, boot_mode CarMode)
{
    // 只在MODE_3下运行
    if(CarMode != MODE_3) return;
    
    // 状态机处理
    switch(task3_ctrl.state)
    {
        case TASK3_STATE_IDLE:
            // 初始化绕8字任务
            if(cur_track_state == 0) // 开始时有线
            {
                Task3_StartTracking();
            }
            break;
            
        case TASK3_STATE_TRACKING:
            // 正常循迹状态
            TurnPID.Target = SensorPID.Out;
            
            // 检测到断线
            if(cur_track_state == 1) 
            {
                Task3_HandleLostTrack();
            }
            break;
            
        case TASK3_STATE_LOST_JUST:
            // 刚检测到断线，准备转向
            TurnPID.Target = 0.0f;
            
            task3_ctrl.lost_delay_counter++;
            if(task3_ctrl.lost_delay_counter >= TASK3_LOST_DELAY_COUNT)
            {
                task3_ctrl.lost_delay_counter = 0;
                Task3_StartTurning();
            }
            break;
            
        case TASK3_STATE_TURNING:
            // 正在转向
            TurnPID.Target = 0.0f;
            
            // 检查转向是否完成
            if(Update_Angle_Turn() == 1) // 转向完成
            {
                Task3_ForwardAfterTurn();
            }
            break;
            
        case TASK3_STATE_FORWARD_AFTER_TURN:
            // 转向后前进，寻找下一段线
            TurnPID.Target = 0.0f; // 直行
            
            task3_ctrl.forward_counter++;
            
            // 检测是否重新找到线，或者前进足够距离
            if(cur_track_state == 0) // 重新检测到线
            {
                Task3_HandleLapComplete();
            }
            else if(task3_ctrl.forward_counter > TASK3_FORWARD_TIMEOUT) // 前进超时（安全保护）
            {
                task3_ctrl.state = TASK3_STATE_TRACKING;
                task3_ctrl.forward_counter = 0;
            }
            break;
            
        case TASK3_STATE_COMPLETE:
            // 完成4圈，停止
            Task3_HandleComplete();
            break;
    }
    
    // 根据状态设置SensorPID.Ki
    if(task3_ctrl.state == TASK3_STATE_TRACKING && cur_track_state == 0)
    {
        // 正常循迹时使用积分
        SensorPID.Ki = 0.0f; // 根据实际需要设置
    }
    else
    {
        // 其他状态禁用积分
        SensorPID.Ki = 0.0f;
    }
}

// 获取任务3状态
task3_state_t Task3_GetState(void)
{
    return task3_ctrl.state;
}

// 获取已完成的圈数
uint8_t Task3_GetLapCount(void)
{
    return task3_ctrl.lap_count;
}

// 获取断线计数
uint8_t Task3_GetStopFlag(void)
{
    return task3_ctrl.stop_flag;
}

// 检查任务是否完成（完成4圈）
uint8_t Task3_IsComplete(void)
{
    return (task3_ctrl.lap_count >= 4);
}
