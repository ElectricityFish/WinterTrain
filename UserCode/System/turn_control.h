#ifndef __TURN_CONTROL_H
#define __TURN_CONTROL_H

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
} Turn_PID;

void turn_pid_UpdatePID(void);
void turn_angle_start(uint16 angle);
void turn_angle_pid(void);

#endif
