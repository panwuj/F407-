#ifndef __ALTEST_H
#define __ALTEST_H

#include "main.h"

extern float VerVelEst;
extern float Vel_baro;

float baro_altitude_kalman_filter(float acceleration,float baro_altitude,float dt);
void baro_filter_init(float altitude);
void VerticalVelocityEstimator(float acc_z,float baroAlt,float dt);
#endif
