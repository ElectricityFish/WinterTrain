#include "zf_common_headfile.h"
#include "Inertial_Navigation.h"
#include "nav_flash.h"
#include "Encoder.h"

int32 Nav_read[Read_MaxSize];      
Nag N;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取偏航角的线程函数
// 参数说明     读取偏航角的线程函数，通过切换N.End_f来切换线程
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_Read()
{
        switch(N.End_f)
        {
            case 0:Run_Nag_Save();  //默认执行函数
                break;
            case 1:;
                    flash_Nag_Save();  //写入最后一页，保证falsh存储满
                    N.End_f++;
                    break;
            case 2://Buzzer_check(500);   //蜂鸣器确认执行
                    N.End_f++;  //结束线程
                    break;
        }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     用于生成偏差计算
// 参数说明     N.Final_Out为最终生成的偏差大小
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_Run()
{
    Run_Nag_GPS();  //偏航角读取复现
 if(N.Nag_Stop_f) //防止旋转
 {
    N.Final_Out=0;
    return;
  }
    N.Final_Out=Nag_Yaw - N.Angle_Run;
    
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     偏航角存入
// 参数说明     将读取的YAW存储到flash中存储
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Run_Nag_Save()
{
    N.Mileage_All+=(R_Mileage+L_Mileage)/2.0;//历程计读取，左右编码器，使用浮点数的话误差能保留下来
    if(N.size > MaxSize)//当大于这页有的flash大小的时候，写入一次，防止重复写入
    {
        flash_Nag_Save();
        N.size=0;   //索引重置为0从下一个缓冲区开始读取
        N.Flash_page_index--;   //flash页面索引减小
        zf_assert(N.Flash_page_index > Nag_End_Page);//防止越界报错
    }

    if(N.Mileage_All >= Nag_Set_mileage)    //大于你的设定值的时候
    {
       int32 Save=(int32)(yaw*100); //读取的偏航角放大100倍，避免使用Float类型来存储
       flash_union_buffer[N.size++].int32_type = Save;  //将偏航角写入缓冲区

       N.Save_index++;


       if(N.Mileage_All > 0) N.Mileage_All -= Nag_Set_mileage;//重置历程计数字//保存到flash
       else N.Mileage_All += Nag_Set_mileage;//倒车
    }

}
// 偏航角复现
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     偏航角复现
// 参数说明     读取flash中存储的YAW
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Run_Nag_GPS()
{
    N.Mileage_All+=(int)((R_Mileage+L_Mileage)/2.0);//历程计读取，左右编码器，使用浮点数的话误差能保留下来
    uint16 prospect=0;
    if(N.Mileage_All >= Nag_Set_mileage)
    {
    if(N.Run_index> N.Save_index-2)
    {
        N.Nag_Stop_f++;
        return;
    }
       N.Run_index++;//如果需要跑两圈可以直接把这个赋值为0.
    
       prospect=N.Run_index ;//前瞻
       if(prospect >N.Save_index-2)  prospect=N.Save_index-2;//越界保护
       N.Angle_Run = (Nav_read[prospect]/100.0f);  
       if(N.Mileage_All > 0) N.Mileage_All -= Nag_Set_mileage;//重置历程计数字//保存到flash
       else N.Mileage_All += Nag_Set_mileage;   //倒车
    }


}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯导参数初始化
// 返回参数     void
// 使用示例     放入程序执行开始
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Init_Nag()
{
    memset(&N, 0, sizeof(N));
    N.Flash_page_index=Nag_Start_Page;
    flash_buffer_clear();
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯性导航执行函数
// 参数说明     index           索引
// 参数说明     type            类型值
// 返回参数     void
// 使用示例     放入中断中
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_System(){
    //卫保护
    if(!N.Nag_SystemRun_Index || N.Nag_Stop_f )  return;

    switch(N.Nag_SystemRun_Index)
    {
       case 1 : Nag_Read();    //1是读取
            break;
      case 3: Nag_Run();
            break;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     一次性读取程序，只读取一次！
// 参数说明     index           索引
// 参数说明     type            类型值
// 返回参数     void
// 使用示例     放入主函数直接调用，demo中有示例。
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void NagFlashRead(){
  if(N.Save_state) return;
  flash_Nag_Read();
  uint8 page_trun=0;
  
  for(int index=0;index <= N.Save_index;index++)
  {
    if(index >= N.Save_index)
    {
        N.Save_state=1;
        break;
    }
    int temp_index=index-(500*page_trun);
    if(temp_index >MaxSize)    //当大于设定的flsh大小的时候
    {
        N.Flash_page_index--;   //页面减少
        page_trun++;
        flash_Nag_Read(); //重新读取
    }
     Nav_read[index]= flash_union_buffer[index-(500*page_trun)].int32_type;
  }
  N.Nag_SystemRun_Index++;
}
