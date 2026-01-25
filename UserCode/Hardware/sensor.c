/************************************************************************************************
* @name   红外循迹与误差获取模块
* @note   PID计算不在这里！
* @author MxBz_
*************************************************************************************************/
#include "zf_common_headfile.h"
#include "sensor.h"
#include "menu.h"
#include "PID.h"

/* ==============================================================================================
                                        全局变量定义
   ============================================================================================== */
// 为之后和菜单打配合准备，本来是要直接设成宏定义的，默认为 8 5 3 1
double L4_WEIGHT = 8.0f;           // 外面灯的权重
double L3_WEIGHT = 5.0f;           // 次外面灯的权重
double L2_WEIGHT = 3.0f;           // 次内部灯的权重
double L1_WEIGHT = 1.0f;           // 内部灯的权重

PID_t Sensor_PID = {
    .Target     = 0.0f,
    .Actual     = 0.0f,
    .Actual1    = 0.0f,
    .Out        = 0.0f,

    .Kp         = 1.0f,
    .Ki         = 1.0f,
    .Kd         = 1.0f,

    .Error0     = 0.0f,
    .Error1     = 0.0f,
    .ErrorInt   = 0.0f,

    .OutMax     = 10.0f,
    .OutMin     = -10.0f,
    .OutOffset  = 0.0f,
};

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
double Sensor_GetError(void)
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
    
    return Error;
}

/** 
 * @brief 更新Sensor_pid
 * @note 你应该在每一个计算Sensor的PID之前用一次这个函数
 * @return 
 */
void Sensor_UpdateSensorPID(void)
{
    Sensor_PID.Kp = Menu_GetValue(SENSOR_PID_MENU, 0);
    Sensor_PID.Ki = Menu_GetValue(SENSOR_PID_MENU, 1);
    Sensor_PID.Kd = Menu_GetValue(SENSOR_PID_MENU, 2);
    Sensor_PID.Error0 = Sensor_GetError();
}


