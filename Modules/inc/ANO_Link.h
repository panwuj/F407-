#ifndef __ANO_H
#define __ANO_H

#include "main.h"

typedef struct
{
	u8 send_status;
	u8 send_motor;
	u8 send_speed;
	u8 send_gps;
}_dt_send_;

extern u32 pid_setting;

void ANO_Send_Status(void);
void ANO_Send_Sensors(void);
void ANO_Send_Speed(void);
void ANO_Send_GPS(void);
void ANO_Send_Motor(void);
void ANO_Send_Altitude(void);
void ANO_Send_To_GroundStation(void);
void ANO_Send_UserData(void);
void ANO_Send_UserData_2(void);
u8 ANO_Data_Analysis(u8 *buf,u8 length);
void ANO_Send_To_Car(int16_t data);

#endif
