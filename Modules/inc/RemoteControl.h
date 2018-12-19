#ifndef __REMOTE_H
#define __REMOTE_H
#include "stdint.h"
#include "main.h"

#define ROLL 			0  //channel 1
#define PITCH     1  //channel 2
#define YAW				3  //channel 4
#define THROTTLE  2  //channel 3
#define MODE			4  //channel 5

#define MANUL			0x00//self-stability,non altitude hold,non hover
#define ALT_HOLD	0x01//self-stability,altitude hold,non hover
#define HOVER			0x02//self-stability,altitude hold,hover
#define AUTO			0x03//self-stability,altitude hold,hover,auto flight

extern uint8_t		SBUS_DMA_buffer[35];
extern uint16_t    SBUS_data[16];
extern uint8_t SBUS_update;
extern u8 flight_function;
void RemoteControl_Init(void);
void ReadRemoteControl(void);

#endif
