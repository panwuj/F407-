#include "sensor.h"
#include "math.h"


void GetSensorsData(void)
{
	static uint32_t time_now=0,time_last=0;
	static uint8_t fun_first_run=1;
	float dt;
//	float velocity=0;
	get_dt_in_seconds(&time_now,&time_last,&dt);
	if(fun_first_run)
	{
		fun_first_run=0;
		dt=0.01;
	}
	KS103_height=KS103GetHeigth();
	//get altitude from barometer.
	GetNEDAcceleration();
	GetHorizontalAcceleration(angles);
	
	/******get altitude_baro in ReadBaroTask*******/
//	altitude_baro=MS5611_get_height()-altitude_baro_ground;
	
	global_params.AltitudeActual=baro_altitude_kalman_filter(ned_acc.z,altitude_baro,dt);
	if(KS103_height<4500)
	{
		global_params.AltitudeActual=KS103_height;//in mm
	}
	else
	{
		pidAltitude.kp=0;
	}
	
	//update GPS meassage
	
//	GPS_Update();
//	position_estimate_EKF(ned_acc.x,ned_acc.y,gps.pos_x,gps.pos_y,dt);
//	RotateVelToBody(body_vel,EST_X[1],EST_Y[1]);
	
	pixy_Update();
	pixy_Velocity_estimateEKF(hor_acc.x,hor_acc.y,dt);
}
