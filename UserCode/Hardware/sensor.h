
#ifndef __SENSOR_H
#define __SENSOR_H

#include "PID.h"

/* ==============================================================================================
                                        宏定义和枚举类型
   ============================================================================================== */
#define SENSOR_COUNT            (8)
#define MAPPING_FACTOR          (1.7f)

#define R4                  (gpio_get_level   (E8))
#define R3                  (gpio_get_level   (E9))
#define R2                  (gpio_get_level   (E10))
#define R1                  (gpio_get_level   (E11))

#define L4                   (gpio_get_level   (E12))
#define L3                   (gpio_get_level   (E13))
#define L2                   (gpio_get_level   (E14))
#define L1                   (gpio_get_level   (E15))

/* ==============================================================================================
                                        函数申明
   ============================================================================================== */

void Sensor_Init            (void);
double Sensor_GetSensorError      (void);
double Sensor_GetYawError      (void);
double Sensor_ComplementaryFilteredError (float ALPHA);
int Sensor_CheckTrack(void);  // 返回值: 0-正常, 1-刚断线, 2-持续断线 

#endif
