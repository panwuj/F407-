#ifndef __PWM_H
#define __PWM_H
#include "stm32f4xx.h"

void TIM1_PWM_Init(uint32_t psc,uint16_t prd); //PWM8
void TIM2_PWM_Init(uint32_t psc,uint16_t prd); //PWM2,PWM3
void TIM3_PWM_Init(uint32_t psc,uint16_t prd); //PWM4,PWM5,PWM6,PWM7
void TIM4_PWM_Init(uint32_t psc,uint16_t prd); //PWM1

#endif
