#include "zf_common_headfile.h"
#include "nav_flash.h"
#include "Inertial_Navigation.h"

void flash_Nag_Save(void)
{
	if(flash_check(0, N.Flash_page_index))flash_erase_page(0, N.Flash_page_index);                  
                       
    flash_write_page_from_buffer(0,N.Flash_page_index);
    if(N.End_f == 1)
    {    
     flash_union_buffer[MaxSize+2].uint32_type = N.Save_index;
     flash_write_page_from_buffer(0,Nag_End_Page);
    }
    flash_buffer_clear();
}

void flash_Nag_Read(){
    flash_buffer_clear();
    static uint8 Index_R_f=0;

    if( 0 == Index_R_f)
    {
       flash_read_page_to_buffer(0,Nag_End_Page);
        N.Save_index = flash_union_buffer[MaxSize+2].uint32_type;       
        Index_R_f=1;
        flash_buffer_clear();
    }
    if(flash_check(0, N.Flash_page_index))
    {        
        flash_read_page_to_buffer(0, N.Flash_page_index);
    }
}
