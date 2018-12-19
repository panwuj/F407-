#include "motor.h"
#include "pwm.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

void Motor_PwmRefresh(int16_t MOTORPWM_1,int16_t MOTORPWM_2,int16_t MOTORPWM_3,int16_t MOTORPWM_4)
{		
	taskENTER_CRITICAL();
	if(MOTORPWM_1>4500)MOTORPWM_1=4500;
	if(MOTORPWM_2>4500)MOTORPWM_2=4500;
	if(MOTORPWM_3>4500)MOTORPWM_3=4500;
	if(MOTORPWM_4>4500)MOTORPWM_4=4500;
	TIM_SetCompare1(TIM4,MOTORPWM_1);
	TIM_SetCompare2(TIM4,MOTORPWM_2);
	TIM_SetCompare3(TIM4,MOTORPWM_3);
	TIM_SetCompare4(TIM4,MOTORPWM_4);
	taskEXIT_CRITICAL();
//  	TIM4->CCR1 = MOTOPWM_1;
//  	TIM4->CCR2 = MOTOPWM_2;
//  	TIM4->CCR3 = MOTOPWM_3;
//  	TIM4->CCR4 = MOTOPWM_4;
}
void Motor_AUX_PwmRefresh(int16_t MOTORPWM_5,int16_t MOTORPWM_6,int16_t MOTORPWM_7,int16_t MOTORPWM_8,int16_t MOTORPWM_9,int16_t MOTORPWM_10)
{		
//	TIM_SetCompare1(TIM4,MOTORPWM_5);
//	TIM_SetCompare2(TIM4,MOTORPWM_6);
//	TIM_SetCompare3(TIM4,MOTORPWM_7);
//	TIM_SetCompare4(TIM4,MOTORPWM_8);
//	
//	TIM_SetCompare3(TIM1,MOTORPWM_9);
//	TIM_SetCompare4(TIM1,MOTORPWM_10);	 
}

void Motor_Init(void)
{
	TIM4_PWM_Init(42-1,5000);
//	delay_ms(50);
	Motor_PwmRefresh(4500,4500,4500,4500);
	delay_ms(2000);
	Motor_PwmRefresh(2000,2000,2000,2000);
}

void Motor_AUX_Init(void)
{ 
	TIM3_PWM_Init(42-1,5000);
	delay_ms(50);
	Motor_AUX_PwmRefresh(0,0,0,0,0,0);
//	delay_ms(2000);
}
