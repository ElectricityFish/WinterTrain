#include "zf_common_headfile.h"
#ifndef __SENSOR_H
#define __SENSOR_H

//宏定义传感器返回值
#define Right3 gpio_get_level (E8);
#define Right2 gpio_get_level (E9);
#define Right1 gpio_get_level (E10);
#define Right0 gpio_get_level (E11);

#define Left3 gpio_get_level (E12);
#define Left2 gpio_get_level (E13);
#define Left1 gpio_get_level (E14);
#define Left0 gpio_get_level (E15);

void Sensor_Init(void);
int16_t Sensor_Get_WeightError(void);
#endif
