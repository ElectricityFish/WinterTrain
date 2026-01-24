#ifndef __PID_H
#define __PID_H

typedef struct {
	float Target;
	float Actual;
	float Actual1;
	float Out;
	
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;
	float Error1;
	float ErrorInt;
	
	float OutMax;
	float OutMin;
	float OutOffset;	//输出偏移值
} PID_t;	//PID结构体


void PID_Update(PID_t *p);					//一般PID函数
float TracePID_Update(void);				//循迹PID函数

#endif
