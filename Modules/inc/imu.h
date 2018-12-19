#ifndef __IMU_H
#define __IMU_H

#include "main.h"
extern float Pitch,Roll,Yaw;
extern float imu_update_dt;
extern float q[4];
extern float rot[9];
void GetAttitude(void);
void GetRotationMatrix(float q[4],float rot[9]);
void GetHorizontalAcceleration(float angles[3]);
void GetNEDAcceleration(void);
void RotateVelToBody(float body_vel[],float ned_vel_x,float ned_vel_y);
void RotateVectorToBody(float v_n[3],float v_b[3]);
void RotateVectorToNED(float v_n[3],float v_b[3]);
#endif 
