#include "pwm.h"

//TIM1 channel_1 for PWM8
void TIM1_PWM_Init(uint32_t psc,uint16_t prd)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef	TIM1_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	/* Time1 base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler         = psc;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period            = prd;          
	TIM_TimeBaseStructure.TIM_ClockDivision     = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	/* Channel 1 Configuration in PWM mode */
	TIM1_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
	TIM1_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
	TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM1_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;         
	TIM1_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_Low;
	TIM1_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
	TIM1_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
//	TIM1_OCInitStructure.TIM_Pulse = TIM1_CCR.wCCR1;
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM1, &TIM1_OCInitStructure);
		TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM1, &TIM1_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);                                 
	TIM_Cmd(TIM1, ENABLE);
}
//TIM2 for PWM 2~3 init
void TIM2_PWM_Init(uint32_t psc,uint16_t prd)
{
	GPIO_InitTypeDef         GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef	TIM2_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler         = psc;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period            = prd;               
	TIM_TimeBaseStructure.TIM_ClockDivision     = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM2_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
	TIM2_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
	TIM2_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM2_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;         
	TIM2_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_Low;
	TIM2_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
	TIM2_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC1Init(TIM2, &TIM2_OCInitStructure);
	
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2, &TIM2_OCInitStructure);
	
	TIM_CtrlPWMOutputs(TIM2, ENABLE);                                    
	TIM_Cmd(TIM2, ENABLE);
}
//TIM3 for PWM 4-7 init
void TIM3_PWM_Init(uint32_t psc,uint16_t prd)
{
	GPIO_InitTypeDef         GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef				 TIM3_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	//PWM5~8
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //??
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
	TIM_TimeBaseStructure.TIM_Prescaler         = psc;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period            = prd;               //??10ms
	TIM_TimeBaseStructure.TIM_ClockDivision     = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM3_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
	TIM3_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
	TIM3_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM3_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;         //TIM???????
	TIM3_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_Low;
	TIM3_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
	TIM3_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC1Init(TIM3, &TIM3_OCInitStructure);
	
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM3, &TIM3_OCInitStructure);
	
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM3, &TIM3_OCInitStructure);
	
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM3, &TIM3_OCInitStructure);
	
	//TIM_CtrlPWMOutputs(TIM8, ENABLE);                             
	TIM_Cmd(TIM3, ENABLE);
}

void TIM4_PWM_Init(uint32_t psc,uint16_t prd)
{
	GPIO_InitTypeDef         GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef				 TIM4_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	//PWM1
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler         = psc;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period            = prd;               
	TIM_TimeBaseStructure.TIM_ClockDivision     = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM4_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
	TIM4_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
	TIM4_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM4_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;         
	TIM4_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_Low;
	TIM4_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
	TIM4_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
		TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC1Init(TIM4, &TIM4_OCInitStructure);  
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM4, &TIM4_OCInitStructure);        
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM4, &TIM4_OCInitStructure);  
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM4, &TIM4_OCInitStructure);  	
	TIM_Cmd(TIM4, ENABLE);
}
