
/************************************************************************************************
* @name   红外循迹与误差获取模块
* @note   PID计算不在这里！
* @author MxBz_
*************************************************************************************************/
#include "zf_common_headfile.h"
#include "Sensor.h"
#include "Menu.h"
#include "PID.h"
#include <math.h>

/* ==============================================================================================
                                        全局变量定义
   ============================================================================================== */
// 为之后和菜单打配合准备，本来是要直接设成宏定义的，默认为 8 5 3 1
double L4_WEIGHT = 8.0f;           // 外面灯的权重
double L3_WEIGHT = 5.0f;           // 次外面灯的权重
double L2_WEIGHT = 3.0f;           // 次内部灯的权重
double L1_WEIGHT = 1.0f;           // 内部灯的权重

int track_lost_counter = 0;
double yaw_offset;


/* ==============================================================================================
                                        函数定义
   ============================================================================================== */

/** 
 * @brief 传感器初始化
 * @note 一些简单的gpio_init堆砌
 * @return
 */
void Sensor_Init(void)
{
    gpio_init(E8,  GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(E9,  GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(E10, GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(E11, GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(E12, GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(E13, GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(E14, GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(E15, GPI, GPIO_LOW, GPI_PULL_DOWN);
    
}

/** 
 * @brief 获取 传感器Error值
 * @note 给每个灯一个权重，然后加和，等下传到sensor_pid去 (离散部分)
 * @return 一个 double 值，传到互补滤波去
 */
double Sensor_GetSensorError(void)
{
    // 更新权重
    int16_t L4_WEIGHT = Menu_GetValue(SENSOR_MENU, 4);
    int16_t L3_WEIGHT = Menu_GetValue(SENSOR_MENU, 3);
    int16_t L2_WEIGHT = Menu_GetValue(SENSOR_MENU, 2);
    int16_t L1_WEIGHT = Menu_GetValue(SENSOR_MENU, 1);

    double Error = 0;
    // LEFT
    Error -= ( L4_WEIGHT * L4
             + L3_WEIGHT * L3
             + L2_WEIGHT * L2
             + L1_WEIGHT * L1);
    // RIGHT
    Error += ( L4_WEIGHT * R4
             + L3_WEIGHT * R3
             + L2_WEIGHT * R2
             + L1_WEIGHT * R1);
    
    return Error;
}

extern float acc_yaw;
const float dt = 0.01f;         // 10ms 的采样频率
/** 
 * @brief 获取 线性Error值
 * @note 详情见我的 1.26学习日志，拟合函数值见Sensor.h
 * @return 一个 double 值，传到互补滤波去
 */
double Sensor_GetYawError(void)
{
    double delta_yaw = acc_yaw * dt;
    yaw_offset += delta_yaw;            // 这样记录的就是相对于目标值的偏差了
    double mapped_integral_error = yaw_offset * MAPPING_FACTOR;
    return mapped_integral_error; 
}

/** 
 * @brief 为离散Error和线性Error做互补滤波
 * @note 一个简单的互补滤波，ALPHA请在Sensor.h中调整
 * @return 返回一个 double, 用于PID
 */
double Sensor_ComplementaryFilteredError(float ALPHA)
{
    static double Filtered_Error;

    double Sensor_Error = Sensor_GetSensorError();
    double Yaw_Error = Sensor_GetYawError();

    Filtered_Error = Sensor_Error * ALPHA 
                   + Yaw_Error * (1 - ALPHA);

    return Filtered_Error;
}

extern float yaw;
/** 
 * @brief 检查是不是 >TRACK LOST<
 * @note 卡车丢失
 * @return 0表示正常，1表示卡车丢失
 */
int Sensor_CheckTrack(void)      // 0->黑线  1->白线
{   
//    if (Left1 && Left2 && Left3 && Left4 && Right1 && Right2 && Right3 && Right4) {
//        return 1;
//    } 
//	else 
//	{
//		return 0;
//	}
	
	uint8_t Sensor_LED[8] = {L1,L2,L3,L4,R1,R2,R3,R4};
	int Count = 0;
	for (int i = 0; i < 8; i++) {
		if (Sensor_LED[i] == 0) { // 检测到黑线
			Count++;
		}
    }
	
	if(Count == 0){return 1;}
	else{return 0;}
}
