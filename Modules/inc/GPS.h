#ifndef __GPS_H
#define __GPS_H	 

#include "main.h"

typedef struct
{
	double longitude;
	double latitude;
	float altitude;
	float speed;
	float pos_x;// in meters
	float pos_y;// in meters
	char NShemisphere;
	char EWhemisphere;
	uint8_t svnum;
	uint8_t available;
	uint8_t update;
	uint8_t HomeSet;
}gps_struct;

extern gps_struct gps;
void GPS_Analysis(u8 *buf);
void GPS_Update(void);
void GPS_HomeReset(void);
void Location_Init(void);
#endif  

 