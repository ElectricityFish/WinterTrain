#ifndef __DISTANCE_CONTROL_H
#define __DISTANCE_CONTROL_H

typedef struct {
	float Target;
	float Actual;
	float Out;
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;
	float Error1;
	float ErrorInt;
	
	float OutMax;
	float OutMin;
} Distance_PID;

#define wheel_r = 3.4   //单位：cm

void Right_EncodeGet(void);
void Left_EncodeGet(void);
float Right_Distance(void);
float Left_Distance(void);
void distance_pid_UpdatePID(void);
void run_distance(uint16_t distance);

#endif
