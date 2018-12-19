#ifndef __FLIGHT_H
#define __FLIGHT_H

#include "main.h"

/*******for debug******/
typedef struct
{
	float q0;
	float q1;
	float q2;
	float q3;
	float dt;
	float roll;
	float pitch;
	float yaw;
	int16_t mpu6050[6];
	int16_t hmc5883l[3];
}_att_info_;

/*******for debug******/

void FlightCtrlSysInit(void);

void flight_task_1(void);
void flight_task_2(void);
void flight_task_3(void);
void flight_task_4(void);

void BEEP_Init(void);

#endif
