/*********************************************************************************************************************
* MM32F327X-G8P Opensourec Library 即（MM32F327X-G8P 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MM32F327X-G8P 开源库的一部分
* 
* MM32F327X-G8P 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          zf_driver_delay
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.37
* 适用平台          MM32F327X_G8P
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-08-10        Teternal            first version
********************************************************************************************************************/

#include "hal_device_registers.h"
#include "zf_common_clock.h"
#include "zf_common_debug.h"

#include "zf_driver_delay.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     system 延时函数 ms 级别
// 参数说明     time        需要延时的时间 ms 级别
// 返回参数     void
// 使用示例     system_delay_ms(100);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void system_delay_ms (uint32 time)
{
    zf_assert(36000000 >= time);

    SysTick->LOAD   =   system_clock / 1000;
    SysTick->VAL    =   0UL; 
    SysTick->CTRL   =   SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while(time --)
    {
        while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
        SysTick->CTRL   |=   SysTick_CTRL_ENABLE_Msk;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     system 延时函数 us 级别
// 参数说明     time        需要延时的时间 us 级别
// 返回参数     void
// 使用示例     system_delay_us(100);
// 备注信息     受限于程序运行跳转 此延时会比输入值高出一些
//-------------------------------------------------------------------------------------------------------------------
void system_delay_us (uint32 time)
{
    zf_assert(5000 >= time);

    SysTick->LOAD   =   system_clock / 1000000 * time;
    SysTick->VAL    =   0UL; 
    SysTick->CTRL   =   SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
}

