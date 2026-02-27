#include "zf_common_headfile.h"
#include "nav_flash.h"
#include "Inertial_Navigation.h"

// 扇区写入   30 - 90 
#define NAG_FLASH_SECTOR_START    (30) 
#define NAG_MAGIC_WORD            (0xA5A5A5A5) // 用于标记"这里有有效数据"
#define SECTOR_COUNT              (60) // 占用扇区数

// 保存路径到 Flash (跨页写入)
void flash_save_nag(void)
{		
    uint32_t start_sector = NAG_FLASH_SECTOR_START;

	for(int s = 0; s < SECTOR_COUNT; s++) {
        for(int p = 0; p < 4; p++) {
            flash_erase_page(NAG_FLASH_SECTOR_START + s, p);
        }
    }
	
    uint32_t sector = NAG_FLASH_SECTOR_START;
    uint32_t page = 0;
	
    // === 1. 准备第一页 (包含文件头) ===
    flash_buffer_clear();
    
    // 写入文件头
    flash_union_buffer[0].uint32_type = NAG_MAGIC_WORD; // 标记:有效数据
    flash_union_buffer[1].uint32_type = (uint32_t)N.Save_index; // 标记:有多少个点
    
    // 写入第一批数据
    int count = 0;
    //int items_per_page = FLASH_DATA_BUFFER_SIZE; // 256个uint32
    // 第一页前面占用了2个位置，所以还能存 254 个数据
    int first_page_capacity = 127;
    
	int offset = 2; // 从第2个格开始填
	
    for(int i=0; i < first_page_capacity && i < N.Save_index; i++)
    {
        // 巧妙拆分：把结构体里的 x 和 y 分别存入
        flash_union_buffer[offset].float_type = Nav_Record_Buffer[count].x;
        flash_union_buffer[offset+1].float_type = Nav_Record_Buffer[count].y;
        offset += 2;
        count++;
    }
    
	
    // 写入第一页 (注意: flash_write_page_from_buffer 内部会自动擦除)
    flash_write_page_from_buffer(sector, page);
    page++; // 准备写下一页
    
    // === 2. 循环写入剩余数据 ===
    while(count < N.Save_index)
    {
        // 处理页码/扇区递增 (MM32F327 每个扇区4页)
        if(page > 3) {
            sector++;
            page = 0;
        }
        
        flash_buffer_clear();
        offset = 0;
		
        // 填满这一页缓冲
		// 满页容量写死 128
        int normal_page_capacity = 128;
        for(int i=0; i < normal_page_capacity && count < N.Save_index; i++)
        {
            flash_union_buffer[offset].float_type = Nav_Record_Buffer[count].x;
            flash_union_buffer[offset+1].float_type = Nav_Record_Buffer[count].y;
            offset += 2;
            count++;
        }
        
        // 写入 Flash
        flash_write_page_from_buffer(sector, page);
        page++;
    }
	
	// ==========================================================
    // 阶段三：立即自检 (Verify)
    // 存完马上读回来检查，看看是不是真的存进去了
    // ==========================================================
    flash_read_page_to_buffer(NAG_FLASH_SECTOR_START, 0);
	
}

// 从 Flash 读取路径 (跨页读取)
uint8_t flash_load_nag(void)
{
    uint32_t sector = NAG_FLASH_SECTOR_START;
    uint32_t page = 0;
    
    // === 1. 读取第一页 (检查文件头) ===
    flash_read_page_to_buffer(sector, page);
    
    // 检查魔术字 (如果不是 A5A5A5A5，说明还没存过数据)
    if(flash_union_buffer[0].uint32_type != NAG_MAGIC_WORD) {
        return 0; // 失败
    }
    
    // 恢复点数
    N.Save_index = (uint16_t)flash_union_buffer[1].uint32_type;
    
    // 安全保护：防止读出来的长度撑爆 RAM
    if(N.Save_index > MaxSize) N.Save_index = MaxSize;
    
    // 读取第一批数据
    int count = 0;
    //int items_per_page = FLASH_DATA_BUFFER_SIZE;
    int first_page_capacity = 127;
    int offset = 2;
	
    for(int i=0; i < first_page_capacity && count < N.Save_index; i++)
    {
        Nav_Record_Buffer[count].x = flash_union_buffer[offset].float_type;
        Nav_Record_Buffer[count].y = flash_union_buffer[offset+1].float_type;
        offset += 2;
        count++;
    }
    
    page++;
    
    // === 2. 循环读取剩余数据 ===
    while(count < N.Save_index)
    {
        if(page > 3) {
            sector++;
            page = 0;
        }
        
        flash_read_page_to_buffer(sector, page);
        offset = 0;
		
		int normal_page_capacity = 128;
        for(int i=0; i < normal_page_capacity && count < N.Save_index; i++)
        {
            Nav_Record_Buffer[count].x = flash_union_buffer[offset].float_type;
            Nav_Record_Buffer[count].y = flash_union_buffer[offset+1].float_type;
            offset += 2;
            count++;
        }
        page++;
    }
    
    return 1; // 成功
}
