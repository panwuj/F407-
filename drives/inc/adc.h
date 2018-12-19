#ifndef __ADC_H
#define __ADC_H
#include "sys.h"

void adcInit(void);
u16 getAdcValue(u8 ch);
u16 getAdcAverage(u8 ch,u8 times);
#endif
