#ifndef __ATTITUDE_CF_H
#define __ATTITUDE_CF_H

#include "main.h"

extern float Roll_cf;
extern float Pitch_cf;
extern float Yaw_cf;

void AttitudeCF(float dt);
void GetObservationByCF(float Z[4],float dt);
#endif
