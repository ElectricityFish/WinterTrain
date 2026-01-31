#include "zf_common_headfile.h"
#include <string.h>
#include <stdlib.h>
#include "BlueSerial.h"

char Serial_RxPacket[100];
uint8_t Serial_RxFlag;


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙串口串口初始化
// 无参数
// 返回参数     无         
// 备注信息     使用的是UART6的RX和TX，波特率9600
//-------------------------------------------------------------------------------------------------------------------
void BludeSerial_Init(void)
{
	uart_init(UART_6, 9600, UART6_TX_C6, UART6_RX_C7);
	uart_rx_interrupt (UART_6, 1);
	 Serial_RxFlag = 0;  // 初始化标志位
}


void BlueSerial_SendByte(uint8_t Byte)
{
	uart_write_byte (UART_6, Byte);
}

void BlueSerial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		BlueSerial_SendByte(Array[i]);
	}
}

void BlueSerial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		BlueSerial_SendByte(String[i]);
	}
}

uint32_t BlueSerial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void BlueSerial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		BlueSerial_SendByte(Number / BlueSerial_Pow(10, Length - i - 1) % 10 + '0');
	}
}



void BlueSerial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	BlueSerial_SendString(String);
}






//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口接收中断函数
// 参数说明     无
// 返回参数     void        
// 备注信息     完成接收文本数据包
//-------------------------------------------------------------------------------------------------------------------

void UART6_IRQHandler (void)
{	
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;
	
    if(UART6->ISR & 0x00000002)                                                 // 串口接收缓冲中断
    {
        wireless_module_uart_handler();
        
		//uint8_t RxData =uart_read_byte (UART_6);
		uint8_t RxData = (uint8_t)UART6->RDR;//D老师让我直接读取寄存器
		
		if (RxState == 0)
		{
			if (RxData == '[' && Serial_RxFlag == 0)
			{
				RxState = 1;
				pRxPacket = 0;
			}
		}
		else if (RxState == 1)
		{
			if (RxData == ']')
			{
				RxState = 0;
				Serial_RxPacket[pRxPacket] = '\0';
				Serial_RxFlag = 1;
			}
			else
			{
				Serial_RxPacket[pRxPacket] = RxData;
				pRxPacket ++;
			}
		}
		
        UART6->ICR |= 0x00000002;                                               // 清除中断标志位
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙串口控制函数
// 参数说明     SpeedPID结构体和TurnPID结构体里的Target成员
// 返回参数     void        
//-------------------------------------------------------------------------------------------------------------------
void BlueSerial_Control(float *SpeedTarget,float *TurnTarget)
{
	
	int8_t LH;
	int8_t LV;
	int8_t RH;
	int8_t RV;
	
	if (Serial_RxFlag == 1)
	{           
		//strtok strcmp atoi/atof
		
		char *Tag = strtok(Serial_RxPacket, ",");
	   
		if (strcmp(Tag, "joystick") == 0)
		{
			LH = atoi(strtok(NULL, ","));//左摇杆横向值
			LV = atoi(strtok(NULL, ","));//左摇杆纵向值
			RH = atoi(strtok(NULL, ","));
			RV = atoi(strtok(NULL, ","));
		}
		
		*SpeedTarget=LV/15;
		*TurnTarget=RH/10;
		
		Serial_RxFlag = 0;//记得标志位归零
		
		
    }	
}


