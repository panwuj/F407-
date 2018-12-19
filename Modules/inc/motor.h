#ifndef __MOTOR_H
#define __MOTOR_H
#include "stdint.h"

void Motor_PwmRefresh(int16_t MOTORPWM_1,int16_t MOTORPWM_2,int16_t MOTORPWM_3,int16_t MOTORPWM_4);
void Motor_AUX_PwmRefresh(int16_t MOTORPWM_5,int16_t MOTORPWM_6,int16_t MOTORPWM_7,int16_t MOTORPWM_8,int16_t MOTORPWM_9,int16_t MOTORPWM_10);
void Motor_Init(void);
void Motor_AUX_Init(void);
#endif
