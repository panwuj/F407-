#ifndef __FLIGHTRCD_H
#define __FLIGHTRCD_H

#include "main.h"

#define REC_MSG_NUM	 10

#define RECORD_TASK_PRIO	2
#define RECORD_STK_SIZE	1024*2

typedef struct recoder_struct
{
	int16_t acc_raw[3];
	int16_t gyro_raw[3];		
	int16_t magn_raw[3];  //9
	float imu_update_dt;  //10
	float alt_ctrl_dt;		//11	
	float vel_ctrl_dt;    //12
	float q[4];					  //13-16
	float att_actual[3];	//17-19  *
	float att_desired[3]; //20-22	 *
	float yaw_rate_desired;//23    *
	double longitude;			 //24
	double latitude;			 //25
	float pos_est[2];			 //26-27
	float pos_desired[2];	 //28-29
	float altitude_baro;   //30
	float altitude_est;    //31
	float altitude_desired;	//32
	float vel_actual[3];		//33-35
	uint16_t motor[4];			//36-39
	uint8_t ctrl_mode;			//40
	int16_t rc_value[5];		//41-45
}_flight_recoder_;


//extern _flight_recoder_ recoder_data[REC_MSG_NUM];

//void RecorderDataAssign(_flight_recoder_ *rec_data);
//void FlightDataEnQueue(void);
//void FlightDataRecord_task(void *pvParameters);
#endif
