#include "zf_common_headfile.h"
#include "nav_flash.h"
#include "Inertial_Navigation.h"

#define FLASH_SECTION_INDEX    (0)   // 存储数据用的扇区
#define road_memery_page_INDEX (11)  // 路径存点专用扇区

#define X_memery_page_INDEX_1   (1)
#define X_memery_page_INDEX_3   (3)
#define X_memery_page_INDEX_5   (5)
#define X_memery_page_INDEX_7   (7)
#define X_memery_page_INDEX_9   (9)
#define X_memery_page_INDEX_13  (13)


#define Y_memery_page_INDEX_2   (2)
#define Y_memery_page_INDEX_4   (4)
#define Y_memery_page_INDEX_6   (6)
#define Y_memery_page_INDEX_8   (8)
#define Y_memery_page_INDEX_10  (10)
#define Y_memery_page_INDEX_12  (12)

uint8_t flash_flag = 0;
uint8_t flash_flag_Plus = 0;

float yaw_memery[FLASH_DATA_BUFFER_SIZE] = {0};      // 记录的航偏角
float yaw_store[FLASH_DATA_BUFFER_SIZE] = {0};       // 打点储存的历史航偏角

void flash_road_memery_store(void)
{
	flash_buffer_clear();
	
	for(size_t i = 0; i < FLASH_DATA_BUFFER_SIZE - 1; i ++)
	{
		flash_union_buffer[i].float_type = yaw_memery[i];
	}
	
	if(flash_check(FLASH_SECTION_INDEX, road_memery_page_INDEX))
	{
		flash_erase_page(FLASH_SECTION_INDEX, road_memery_page_INDEX);
	}
	
	flash_write_page_from_buffer(FLASH_SECTION_INDEX, road_memery_page_INDEX);
	
	flash_flag = 3;
}

void flash_road_memery_get(void)
{
	if(flash_check(FLASH_SECTION_INDEX, road_memery_page_INDEX))
	{
		flash_read_page_to_buffer(FLASH_SECTION_INDEX, road_memery_page_INDEX);
		for(size_t i = 0; i < FLASH_DATA_BUFFER_SIZE - 1; i ++)
		{
			yaw_store[i] = flash_union_buffer[i].float_type;
		}
	}
	
	flash_flag = 4;
}

void flash_road_memery_store_Plus(void)
{
	flash_buffer_clear();
	
	for(size_t i = 0; i < FLASH_DATA_BUFFER_SIZE - 1; i ++)
	{
		flash_union_buffer[i].float_type = X_Memery_Plus[i];
	}
	
	if(flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_1))
	{
		flash_erase_page(FLASH_SECTION_INDEX, X_memery_page_INDEX_1);
	}
	
	flash_write_page_from_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_1);
	
	flash_buffer_clear();
	
	for(size_t i = FLASH_DATA_BUFFER_SIZE, j = 0; i < 2 * FLASH_DATA_BUFFER_SIZE; i ++, j ++)
	{
		flash_union_buffer[j].float_type = X_Memery_Plus[i];
	}
	
	if(flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_3))
	{
		flash_erase_page(FLASH_SECTION_INDEX, X_memery_page_INDEX_3);
	}
	
	flash_write_page_from_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_3);
	
	flash_buffer_clear();
	
	for (size_t i = 2 * FLASH_DATA_BUFFER_SIZE, j = 0; i < 3 * FLASH_DATA_BUFFER_SIZE; i++, j++)
    {
        flash_union_buffer[j].float_type = X_Memery_Plus[i];
    }
	
	if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_5))
    { // 判断是否有数据
        flash_erase_page(FLASH_SECTION_INDEX, X_memery_page_INDEX_5);
    } // 擦除这一页
    // 第四步存到FLASH指定扇区
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_5);

    // 新增X轴数据存储，扇区 7
    flash_buffer_clear();
    for (size_t i = 3 * FLASH_DATA_BUFFER_SIZE, j = 0; i < 4 * FLASH_DATA_BUFFER_SIZE; i++, j++)
    {
        flash_union_buffer[j].float_type = X_Memery_Plus[i];
    }
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_7))
    {
        flash_erase_page(FLASH_SECTION_INDEX, X_memery_page_INDEX_7);
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_7);

    // 新增X轴数据存储，扇区 9
    flash_buffer_clear();
    for (size_t i = 4 * FLASH_DATA_BUFFER_SIZE, j = 0; i < 5 * FLASH_DATA_BUFFER_SIZE; i++, j++)
    {
        flash_union_buffer[j].float_type = X_Memery_Plus[i];
    }
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_9))
    {
        flash_erase_page(FLASH_SECTION_INDEX, X_memery_page_INDEX_9);
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_9);

    // 新增X轴数据存储，扇区 13
    flash_buffer_clear();
    for (size_t i = 5 * FLASH_DATA_BUFFER_SIZE, j = 0; i < 6 * FLASH_DATA_BUFFER_SIZE; i++, j++)
    {
        flash_union_buffer[j].float_type = X_Memery_Plus[i];
    }
    // 在最后一页的最后一个位置存储NUM_L_Plus，用于记录实际数据长度即路径终点
    flash_union_buffer[FLASH_DATA_BUFFER_SIZE - 1].uint16_type = NUM_L_Plus;
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_13))
    {
        flash_erase_page(FLASH_SECTION_INDEX, X_memery_page_INDEX_13);
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_13);

    /*存y轴数据  分三个扇区依次存*/

    // 第一步清除缓冲区
    flash_buffer_clear();
    // 第二步把数据存到缓冲区
    for (size_t i = 0; i < FLASH_DATA_BUFFER_SIZE; i++)
    {
        flash_union_buffer[i].float_type = Y_Memery_Plus[i];
    }

    // 第三步判断FLASH里有没有数据，有就把FLASH数据擦除/把数据存到缓冲区
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_2))
    { // 判断是否有数据
        flash_erase_page(FLASH_SECTION_INDEX, Y_memery_page_INDEX_2);
    } // 擦除这一页
    // 第四步存到FLASH指定扇区
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_2);

    // 第一步清除缓冲区
    flash_buffer_clear();
    // 第二步把数据存到缓冲区
    for (size_t i = FLASH_DATA_BUFFER_SIZE, j = 0; i < 2 * FLASH_DATA_BUFFER_SIZE; i++, j++)
    {
        flash_union_buffer[j].float_type = Y_Memery_Plus[i];
    }

    // 第三步判断FLASH里有没有数据，有就把FLASH数据擦除/把数据存到缓冲区
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_4))
    { // 判断是否有数据
        flash_erase_page(FLASH_SECTION_INDEX, Y_memery_page_INDEX_4);
    } // 擦除这一页
    // 第四步存到FLASH指定扇区
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_4);

    // 第一步清除缓冲区
    flash_buffer_clear();
    // 第二步把数据存到缓冲区
    for (size_t i = 2 * FLASH_DATA_BUFFER_SIZE, j = 0; i < 3 * FLASH_DATA_BUFFER_SIZE; i++, j++)
    {
        flash_union_buffer[j].float_type = Y_Memery_Plus[i];
    }

    // 第三步判断FLASH里有没有数据，有就把FLASH数据擦除/把数据存到缓冲区
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_6))
    { // 判断是否有数据
        flash_erase_page(FLASH_SECTION_INDEX, Y_memery_page_INDEX_6);
    } // 擦除这一页
    // 第四步存到FLASH指定扇区
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_6);

    // 新增Y轴数据存储，扇区 8
    flash_buffer_clear();
    for (size_t i = 3 * FLASH_DATA_BUFFER_SIZE, j = 0; i < 4 * FLASH_DATA_BUFFER_SIZE; i++, j++)
    {
        flash_union_buffer[j].float_type = Y_Memery_Plus[i];
    }
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_8))
    {
        flash_erase_page(FLASH_SECTION_INDEX, Y_memery_page_INDEX_8);
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_8);

    // 新增Y轴数据存储，扇区 10
    flash_buffer_clear();
    for (size_t i = 4 * FLASH_DATA_BUFFER_SIZE, j = 0; i < 5 * FLASH_DATA_BUFFER_SIZE; i++, j++)
    {
        flash_union_buffer[j].float_type = Y_Memery_Plus[i];
    }
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_10))
    {
        flash_erase_page(FLASH_SECTION_INDEX, Y_memery_page_INDEX_10);
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_10);

    // 新增Y轴数据存储，扇区 12
    flash_buffer_clear();
    for (size_t i = 5 * FLASH_DATA_BUFFER_SIZE, j = 0; i < 6 * FLASH_DATA_BUFFER_SIZE; i++, j++)
    {
        flash_union_buffer[j].float_type = Y_Memery_Plus[i];
    }
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_12))
    {
        flash_erase_page(FLASH_SECTION_INDEX, Y_memery_page_INDEX_12);
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_12);

    // 存取过程结束 —— 成功存取
    flash_flag_Plus = 3;
}

void flash_road_memery_get_Plus(void)
{

    /*取x轴数据,分三个扇区取*/
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_1)) // 检查flash里面是否存了数据
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_1);
        for (size_t i = 0; i < FLASH_DATA_BUFFER_SIZE; i++)
        {
            X_Memery_Store_Plus[i] = flash_union_buffer[i].float_type; //
        }
    }

    /*取x轴数据*/
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_3)) // 检查flash里面是否存了数据
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_3);
        for (size_t i = 0, j = FLASH_DATA_BUFFER_SIZE; i < FLASH_DATA_BUFFER_SIZE; i++, j++)
        {
            X_Memery_Store_Plus[j] = flash_union_buffer[i].float_type; //
        }
    }

    /*取x轴数据*/
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_5)) // 检查flash里面是否存了数据
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_5);
        for (size_t i = 0, j = 2 * FLASH_DATA_BUFFER_SIZE; i < FLASH_DATA_BUFFER_SIZE; i++, j++)
        {
            X_Memery_Store_Plus[j] = flash_union_buffer[i].float_type; //
        }
    }

    // 新增X轴数据读取，扇区 7
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_7))
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_7);
        for (size_t i = 0, j = 3 * FLASH_DATA_BUFFER_SIZE; i < FLASH_DATA_BUFFER_SIZE; i++, j++)
        {
            X_Memery_Store_Plus[j] = flash_union_buffer[i].float_type;
        }
    }

    // 新增X轴数据读取，扇区 9
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_9))
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_9);
        for (size_t i = 0, j = 4 * FLASH_DATA_BUFFER_SIZE; i < FLASH_DATA_BUFFER_SIZE; i++, j++)
        {
            X_Memery_Store_Plus[j] = flash_union_buffer[i].float_type;
        }
    }

    // 新增X轴数据读取，扇区 13
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_13))
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_13);
        for (size_t i = 0, j = 5 * FLASH_DATA_BUFFER_SIZE; i < FLASH_DATA_BUFFER_SIZE; i++, j++)
        {
            X_Memery_Store_Plus[j] = flash_union_buffer[i].float_type;
        }
        // 从最后一页读取路径终点值
        road_destination = flash_union_buffer[FLASH_DATA_BUFFER_SIZE - 1].uint16_type;
    }

    /*取y轴数据,分三个扇区取*/
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_2)) // 检查flash里面是否存了数据
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_2);
        for (size_t i = 0; i < FLASH_DATA_BUFFER_SIZE; i++)
        {
            Y_Memery_Store_Plus[i] = flash_union_buffer[i].float_type; //
        }
    }

    /*取y轴数据*/
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_4)) // 检查flash里面是否存了数据
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_4);
        for (size_t i = 0, j = FLASH_DATA_BUFFER_SIZE; i < FLASH_DATA_BUFFER_SIZE; i++, j++)
        {
            Y_Memery_Store_Plus[j] = flash_union_buffer[i].float_type; //
        }
    }

    /*取y轴数据*/
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_6)) // 检查flash里面是否存了数据
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_6);
        for (size_t i = 0, j = 2 * FLASH_DATA_BUFFER_SIZE; i < FLASH_DATA_BUFFER_SIZE; i++, j++)
        {
            Y_Memery_Store_Plus[j] = flash_union_buffer[i].float_type; //
        }
    }

    // 新增Y轴数据读取，扇区 8
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_8))
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_8);
        for (size_t i = 0, j = 3 * FLASH_DATA_BUFFER_SIZE; i < FLASH_DATA_BUFFER_SIZE; i++, j++)
        {
            Y_Memery_Store_Plus[j] = flash_union_buffer[i].float_type;
        }
    }

    // 新增Y轴数据读取，扇区 10
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_10))
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_10);
        for (size_t i = 0, j = 4 * FLASH_DATA_BUFFER_SIZE; i < FLASH_DATA_BUFFER_SIZE; i++, j++)
        {
            Y_Memery_Store_Plus[j] = flash_union_buffer[i].float_type;
        }
    }

    // 新增Y轴数据读取，扇区 12
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_12))
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_12);
        for (size_t i = 0, j = 5 * FLASH_DATA_BUFFER_SIZE; i < FLASH_DATA_BUFFER_SIZE; i++, j++)
        {
            Y_Memery_Store_Plus[j] = flash_union_buffer[i].float_type;
        }
    }

    flash_flag_Plus = 4;
}
