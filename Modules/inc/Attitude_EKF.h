#ifndef __ATTI_EKF
#define __ATTI_EKF

#include "main.h"

extern float	angles[3];
extern float roll_z,pitch_z,yaw_z;
void AttitudeEKF_Init(float ax,float ay,float az,float mx,float my,float mz);
void AttitudeEKF(float gx,float gy,float gz,float ax,float ay,float az,float mx,float my,float mz,float dt);
#endif
