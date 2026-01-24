/************************************************************************************************
* @name   红外循迹与误差获取模块
* @note   PID计算不在这里！
* @author MxBz_
*************************************************************************************************/

#include "zf_common_headfile.h"

//宏定义传感器返回值
#define Right3 gpio_get_level (E8);
#define Right2 gpio_get_level (E9);
#define Right1 gpio_get_level (E10);
#define Right0 gpio_get_level (E11);

#define Left3 gpio_get_level (E12);
#define Left2 gpio_get_level (E13);
#define Left1 gpio_get_level (E14);
#define Left0 gpio_get_level (E15);


void Sensor_Init(void)
{
	gpio_init(E8, GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(E9, GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(E10, GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(E11, GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(E12, GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(E13, GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(E14, GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(E15, GPI, GPIO_LOW, GPI_PULL_DOWN);
    
}

/**
 * @brief 更新权重
 * @note 过渡
 * @return 
 */
void Sensor_UpdateWeight(void)
{
    int16_t L4_WEIGHT = Menu_GetValue(SENSOR_MENU, 4);
    int16_t L3_WEIGHT = Menu_GetValue(SENSOR_MENU, 3);
    int16_t L2_WEIGHT = Menu_GetValue(SENSOR_MENU, 2);
    int16_t L1_WEIGHT = Menu_GetValue(SENSOR_MENU, 1);
}

/** 
 * @brief 获取 Error 值
 * @note 给每个灯一个权重，然后加和，等下传到sensor_pid去
 * @return 一个 `int16_t` 值，代表权重后四舍五入的Error
 */
int16_t Sensor_GetError(void)
{
    Sensor_UpdateWeight(); // 更新一下
    double Error = 0;
    // LEFT
    Error -= ( L4_WEIGHT * Left4
             + L3_WEIGHT * Left3
             + L2_WEIGHT * Left2
             + L1_WEIGHT * Left1);
    // RIGHT
    Error += ( L4_WEIGHT * Right4
             + L3_WEIGHT * Right3
             + L2_WEIGHT * Right2
             + L1_WEIGHT * Right1);
    
    return (int16_t)Error;
}


