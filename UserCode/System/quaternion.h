#ifndef _QUATERNION_H_
#define _QUATERNION_H_

#include "zf_common_typedef.h"

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
    float roll;
    float pitch;
    float yaw;
} Quaternion;

void quaternion_init(void);
void quaternion_update(void);
Quaternion* get_eular_angles(void);

#endif
