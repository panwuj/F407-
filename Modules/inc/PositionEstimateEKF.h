#ifndef __POSESTEKF_H
#define __POSESTEKF_H

#include "main.h"
extern float body_vel[3];
extern float EST_X[3];//s,v,da
extern float EST_Y[3];//s,v,da
void position_estimate_EKF(float ax,float ay,float pos_x,float pos_y,float dt);
void position_estimate_reset(void);
#endif
