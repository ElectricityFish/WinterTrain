#include "zf_common_headfile.h"

//宏定义传感器返回值
#define Right3 gpio_get_level (E8);
#define Right2 gpio_get_level (E9);
#define Right1 gpio_get_level (E10);
#define Right0 gpio_get_level (E11);

#define Left3 gpio_get_level (E12);
#define Left2 gpio_get_level (E13);
#define Left1 gpio_get_level (E14);
#define Left0 gpio_get_level (E15);


void Sensor_Init(void)
{
	gpio_init(E8, GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(E9, GPI, GPIO_LOW, GPI_PULL_DOWN);
	gpio_init(E10, GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(E11, GPI, GPIO_LOW, GPI_PULL_DOWN);
	gpio_init(E12, GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(E13, GPI, GPIO_LOW, GPI_PULL_DOWN);
	gpio_init(E14, GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(E15, GPI, GPIO_LOW, GPI_PULL_DOWN);
	
}


int16_t Sensor_Get_WeightError(void)//获取加权偏移误差
{
    int16_t Error=0;
    Error+=(-1)*Left0;
	Error+=(-3)*Left1; 
	Error+=(-5)*Left2 ;
	Error+=(-8)*Left3 ;
	
	Error+=Right0;
	Error+=3*Right1; 
	Error+=5*Right2;
	Error+=8*Right3;	
	
	return Error;
}

