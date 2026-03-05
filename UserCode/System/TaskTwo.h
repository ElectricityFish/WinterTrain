#include "zf_common_headfile.h"
#ifndef __TASKTWO_H
#define __TASKTWO_H

/**
 * @brief 封装后的任务二函数
 * @note 功能：执行任务二
 * @return 无
 */
void TaskTwoRun(void);

extern uint8_t onLinePromoptFlag;			//用于任务二的声光提示

/**
 * @brief 任务二标志位清零函数
 * @note 功能：执行任务二标志位的 清理防止出错
 * @return 无
 */
void TaskTwoFlagClear(void);
#endif


