#ifndef __LED_H
#define __LED_H
#include "stm32f4xx.h"
#include "sys.h"
#define LED0		PEout(8)	
#define LED1		PEout(9)	
#define LED2		PEout(10)	
#define LED3		PEout(11)	
void LED_Init(void);
#endif
