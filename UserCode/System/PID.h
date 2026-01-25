#ifndef __PID_H
#define __PID_H

/*****************************************************
 * @brief PID结构体
 * @note  Angle、Speed、Turn、SensorPID都基于这个结构体
 * @param Kp_Ki_Kd 三个参数
 * @param Error0 相当于Current_Error
 * @param Error1 相当于Previous_Error
 * @param Out    PID输出
 * @param Actual 实际值，相当于Current_Actual
 * @param Actual1 相当于Previous_Actual
 *****************************************************/
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
	float OutOffset;	// 输出偏移值，微调时候用
} PID_t;

void PID_Update(PID_t *p);					// 一般PID函数
void Sensor_PIDControl(void);				// 循迹PID函数
void Angle_PIDControl(void);				// 角度环PID函数
void SpeedAndTurn_PIDControl(void);			// 速度环和转向环PID函数

#endif
