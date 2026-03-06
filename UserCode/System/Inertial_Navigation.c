#include "zf_common_headfile.h"
#include "Inertial_Navigation.h"
#include "nav_flash.h"
#include "Encoder.h"
#include <math.h>

// 定义大数组 (放在 RAM 里)
Pathpoint Nav_Record_Buffer[MaxSize]; 
Nag N;
extern PID_t SpeedPID;

// 初始化
void Init_Nag(void)
{
    memset(&N, 0, sizeof(N));
    memset(Nav_Record_Buffer, 0, sizeof(Nav_Record_Buffer));
}

// 路径记忆
void Run_Nag_Save(void)
{
    //1. 获取小车的位移
    float step_pulses = (L_Mileage + R_Mileage) / 2.0f;
    float current_step = step_pulses / (float)Nag_Set_mileage; 
    float yaw_rad = Nag_Yaw * PI / 180.0f;
    
    //2. 航位推算，算出小车的坐标
    N.Current_X += current_step * (-sinf(yaw_rad));
//	N.Current_X += current_step * sinf(yaw_rad);
    N.Current_Y += current_step * cosf(yaw_rad);

    //3. 累积里程用于存点判断 (维持原有逻辑，保证点距均匀)
    N.Mileage_All += step_pulses; 
    
    if(N.Mileage_All >= Nag_Set_mileage) 
    {
        if(N.Save_index >= MaxSize - 1) {
            N.End_f = 1; 
            return;
        }
        // 将当前的坐标存入 Flash 数组
        Nav_Record_Buffer[N.Save_index].x = N.Current_X;
        Nav_Record_Buffer[N.Save_index].y = N.Current_Y;
        N.Save_index++;

        N.Mileage_All -= Nag_Set_mileage; 
    }
}

// ========================================================
// 复现逻辑 (Repeat) - 纯跟踪算法 (2ms 高频丝滑版)
// ========================================================
void Run_Nag_GPS(void)
{
    // === 1. 实时位置积分 (每2ms执行，保持最高精度) ===
    float step_pulses = (L_Mileage + R_Mileage) / 2.0f;
    float current_step = step_pulses / (float)Nag_Set_mileage; // 1 step = 1 cm
    float yaw_rad = Nag_Yaw * PI / 180.0f;
    
    N.Current_X += current_step * (-sinf(yaw_rad));
//	N.Current_X += current_step * sinf(yaw_rad);
    N.Current_Y += current_step * cosf(yaw_rad);

    // === 2. 终点停车判定 ===
    if(N.Save_index <= 5 || N.Run_index >= N.Save_index - 10) {
        N.Nag_Stop_f = 1; 
        N.Final_Out = 0;
        SpeedPID.Target = 0.0f;
		Menu_SetRunningMode(MODE_1);		
        return;
    }

    float current_speed = fabs(SpeedPID.Target);    //我们的target为2.0
    
    // ★ 核心修复 1：既然 1个点=1cm，预瞄距离必须拉长！
    // 设基础预瞄为 25cm，速度越快看得越远
//	float L_distance = 3.0f + current_speed * 0.13f;  //这个可以
    float L_distance = 2.0f + current_speed * 0.13f; 
    if(L_distance > 50.0f) L_distance = 50.0f; // 最多看前方 50cm
    
    // 算力优化：提前算好平方，下面比对时就不需要开根号了
    float L_dist_sq = L_distance * L_distance; 

    // === 4. 寻找定位点 (最近点) ===
    float min_dist_sq = 99999999.0f;
    int closest_index = N.Run_index;
    
    int search_start = N.Run_index - 40;
    if(search_start < 0) search_start = 0;
    int search_end = N.Run_index + 40;
    if(search_end >= N.Save_index) search_end = N.Save_index - 1;
    
    for(int i = search_start; i <= search_end; i++) {
        float dx = Nav_Record_Buffer[i].x - N.Current_X;
        float dy = Nav_Record_Buffer[i].y - N.Current_Y;
        
        // 算力优化：直接用平方值对比
        float dist_sq = dx*dx + dy*dy; 
        if(dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_index = i;
        }
    }
    N.Run_index = closest_index; 

    // === 5. 寻找预瞄点 (目标点) ===
    int target_idx = closest_index;
    
    // 搜索窗口必须大于最大预瞄距离 (100)，否则找不到点
    int max_search_idx = closest_index + 100;
    if(max_search_idx > N.Save_index) max_search_idx = N.Save_index;

    for(int i = closest_index; i < max_search_idx; i++) {
        float dx = Nav_Record_Buffer[i].x - N.Current_X;
        float dy = Nav_Record_Buffer[i].y - N.Current_Y;
        float dist_sq = dx*dx + dy*dy;
        
        if(dist_sq >= L_dist_sq) { 
            target_idx = i;
            break;
        }
        target_idx = i;
    }

    // === 6. 纯跟踪曲率解算 ===
    float dx = Nav_Record_Buffer[target_idx].x - N.Current_X;
    float dy = Nav_Record_Buffer[target_idx].y - N.Current_Y;
    
	//曲率计算
//    float e_lat = -dx * cosf(yaw_rad) - dy * sinf(yaw_rad);
	float e_lat = dx * cosf(yaw_rad) + dy * sinf(yaw_rad);
    
    // 整个中断 2ms 只在这里开 1 次根号，对 CPU 毫无压力
    float L_actual = sqrtf(dx*dx + dy*dy);
    if(L_actual < 10.0f) L_actual = 10.0f; // 防除0
	
    float curvature = 2.0f * e_lat / (L_actual * L_actual);
    
    // ★ 核心修复 2：因为 L 放大了，L的平方放大了几十倍，这里的系数也要相应增加，保证转向力
    N.Final_Out = curvature * 1500.0f;   //这里可能是一个要根据实际进行调整的参数

    float max_turn = 50.0f; 
    if(N.Final_Out > max_turn)  N.Final_Out = max_turn;
    if(N.Final_Out < -max_turn) N.Final_Out = -max_turn;
    
    // === 7. 纵向速度规划 ===
    if (N.Save_index > N.Run_index) 
    {
        // 1. 核心参数设定
        float max_straight_speed = 20.0f; // 【参数】直道极限速度 (大胆往上提)
        float min_corner_speed   = 3.0f;  // 【参数】最急的弯允许的最低速度
        float finish_min_speed   = 2.0f;  // 【参数】终点冲线速度
        
        // 2. 弯道动态限速 (根据曲率计算)
        // 曲率越大(弯越急)，减速越多。50000.0f 是灵敏度系数，需根据实际曲率大小微调
        float abs_curv = fabsf(curvature);
        float corner_target_v = max_straight_speed - abs_curv * 50000.0f; 
//        float corner_target_v = max_straight_speed - abs_curv * 5000.0f; 
        
        // 限制弯道最低速度，防止在弯心停下
        if(corner_target_v < min_corner_speed) {
            corner_target_v = min_corner_speed;
        }

        // 默认期望速度等于弯道限速
        float expected_v = corner_target_v;

        // 3. 终点提前减速逻辑 (优先级最高)
        uint16_t decel_distance = 10;     // 离终点还剩多少个点开始刹车
        uint16_t remain_points = N.Save_index - N.Run_index;
        
        if (remain_points <= decel_distance) {
            float finish_v = ((float)remain_points / decel_distance) * (max_straight_speed - finish_min_speed) + finish_min_speed;
            
            // 如果终点要求的速度比弯道要求的还低，听终点的
            if(finish_v < expected_v) {
                expected_v = finish_v;
            }
        } 

        // 4. 执行速度平滑过渡 (斜率限制)
        float accel_step = 0.06f; // 【参数】直道加速的猛烈程度 (推背感)
        float decel_step = 0.20f; // 【参数】入弯刹车的猛烈程度 (刹车一定要比加速快，防止撞墙)

        if (SpeedPID.Target < expected_v) {
            SpeedPID.Target += accel_step; 
            if(SpeedPID.Target > expected_v) SpeedPID.Target = expected_v;
        } 
        else if (SpeedPID.Target > expected_v) {
            SpeedPID.Target -= decel_step; 
            if(SpeedPID.Target < expected_v) SpeedPID.Target = expected_v;
        }
    }
}

// 主任务 (被 isr.c 调用)
void Nag_System(void)
{
    if(N.Nav_System_Run_Index == 0 || N.Nag_Stop_f == 1) {
        N.Final_Out = 0;
        return;
    }

    if(N.Nav_System_Run_Index == 1) {
        if(N.End_f) {
            N.Nag_Stop_f = 1; 
            N.Nav_System_Run_Index = 0; 
            N.End_f = 0;
        } else {
            Run_Nag_Save();
        }
    }
    else if(N.Nav_System_Run_Index == 2) {
        // 在这里，复现模式不用再做 Angle_Run - Nag_Yaw 的差值了
        // 因为 N.Final_Out 已经在 Run_Nag_GPS 的纯跟踪里算出来了
        Run_Nag_GPS(); 
    }
}
