#ifndef NAV_FLASH_H_
#define NAV_FLASH_H_

#include "zf_common_headfile.h"

void flash_road_memery_store_Plus(void);
void flash_road_memery_get_Plus(void);

void flash_road_memery_store(void);        // 存储路径数据
void flash_road_memery_get(void);          // 读取路径数据

extern uint8_t flash_flag;                 // 0为初始标志，1为开始存，2为开始取，3为存完标志，4为取完标志；
extern uint8_t flash_flag_Plus;            // 0为初始标志，1为开始存，2为开始取，3为存完标志，4为取完标志；

// X点存储扇区
#define X_memery_page_INDEX_7   (7)
#define X_memery_page_INDEX_9   (9)
#define X_memery_page_INDEX_13  (13)

// Y点存储扇区
#define Y_memery_page_INDEX_6   (6)
#define Y_memery_page_INDEX_8   (8)
#define Y_memery_page_INDEX_10  (10)
#define Y_memery_page_INDEX_12  (12)

// Yaw角存储扇区
extern float yaw_memery[FLASH_DATA_BUFFER_SIZE];     // 记录的航偏角
extern float yaw_store[FLASH_DATA_BUFFER_SIZE];      // 打点储存的历史航偏角

#endif
