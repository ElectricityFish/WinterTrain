#ifndef __INERTIAL_NAVIGATION_H
#define __INERTIAL_NAVIGATION_H

#include "zf_common_headfile.h"
#include <stdbool.h>

extern float Total_Encoder_L;
extern float Total_Encoder_R;
extern float yaw;

//*********************用户设置区域****************************//
#define MaxSize 10000           //1cm记录一次，可以记录100m

// 2. 里程计阈值
#define Nag_Set_mileage 12     //小车行走1cm的编码器脉冲总数

//参数范围 <1 - 80>
//#define Nag_End_Page 30      //flash中止页面
//#define Nag_Start_Page 90   //flah起始页面

#define Nag_Yaw yaw          //陀螺仪读取出来的偏航角

#define L_Mileage Total_Encoder_L
#define R_Mileage Total_Encoder_R  
//********************************************************//
//将小车的轨迹转化为虚拟坐标值（x，y）存入
typedef struct{
	float x;
	float y;
}Pathpoint;

typedef struct{
        float Final_Out;
	    float Mileage_All;
	    //小车实际坐标值
	    float Current_X;
	    float Current_Y;
		uint8 Nag_Stop_f;                   //结束标志位
		uint16 Run_index;                   //进程标志位
		uint16 Save_index;                  //记录打的点的个数
		uint8 End_f;                        //停止记录
	    uint8 Nav_System_Run_Index;         //引用状态机：0=空闲，1=记忆，2=复现
}Nag;

extern Nag N;   //整个变量的结构体，方便开发和移植

extern Pathpoint Nav_Record_Buffer[MaxSize];

void Run_Nag_GPS();
void Run_Nag_Save(); 
//****************************//
void Init_Nag();    //这个是参数初始化与flash的缓冲区初始化，请放到函数开始。
void Nag_System();  //这个是惯性导航最后的包装函数，请放到中断中。

#endif
