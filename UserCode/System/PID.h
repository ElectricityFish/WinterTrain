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
	float OutOffset;	//输出偏移值
} PID_t;	//PID结构体

extern int prev_track_state;
extern int cur_track_state;    // 0-正常, 1-刚断线, 2-持续断线

void PID_Update(PID_t *p);					//一般PID函数
void Balance_PIDControl(void);				//角度环PID函数
void Sensor_PIDControl(void);				//循迹PID函数，至于为啥不叫Trace，这是个历史遗留问题（哭）

#endif
