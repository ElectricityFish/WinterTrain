#ifndef __SENSOR_H
#define __SENSOR_H

#include "PID.h"

/* ==============================================================================================
                                        宏定义和枚举类型
   ============================================================================================== */
#define SENSOR_COUNT            (8)
#define MAPPING_FACTOR          (1.7f)

#define Right4                  (gpio_get_level   (E8))
#define Right3                  (gpio_get_level   (E9))
#define Right2                  (gpio_get_level   (E10))
#define Right1                  (gpio_get_level   (E11))

#define Left4                   (gpio_get_level   (E12))
#define Left3                   (gpio_get_level   (E13))
#define Left2                   (gpio_get_level   (E14))
#define Left1                   (gpio_get_level   (E15))

/* ==============================================================================================
                                        函数申明
   ============================================================================================== */

void Sensor_Init            (void);
double Sensor_GetSensorError      (void);
double Sensor_GetYawError      (void);
double Sensor_ComplementaryFilteredError (float ALPHA);
int Sensor_CheckTrack       (void); 

#endif
