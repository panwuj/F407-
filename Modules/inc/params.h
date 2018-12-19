#ifndef __PARAMS_H
#define __PARAMS_H
#include "stm32f4xx.h"
#include "stdint.h"
#include "main.h"


typedef struct
{
	float x;
	float y;
	float z;
}velocity;

typedef struct
{
  float roll;
  float pitch;
  float yaw;
}AttitudeObject;

typedef struct 
{
  float longitude;
  float latitude;
  float  altitude;
	uint16_t hover_time;
	uint8_t	speed;
	uint8_t  photo;
}GPS_Point;

typedef struct
{
	int16_t max;
	int16_t min;
	int16_t rang;
	int16_t mid;
	int16_t value;
	int8_t  reverse;
}remote_control;

typedef struct
{
  unsigned char ControlMode;
  unsigned char	TaskLaunch;
  unsigned char	TaskPauseOrResume;
  unsigned char	EmergenciesDispose;
  unsigned char	GoHome;
  unsigned char GPSPointCounter;
  unsigned char GPSPointsMax;
  unsigned char GPSUploaded;
  unsigned char MotorLocked; 
	
	float AltitudeActual;
  float AltitudeDesired;
  
  AttitudeObject AttitudeDesired;
  AttitudeObject AttitudeActual;
  AttitudeObject CameraAttitudeDesired;
	
	velocity VelocityDesired;
	velocity VelocityActual;
	
	remote_control rc[8];
	GPS_Point HomePoint;
  GPS_Point GPSTargetPoints[20];
  
}Params; 

extern Params global_params;

void global_params_Init(void);
#endif
