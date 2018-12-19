#include "params.h"
#include "pid.h"
#include "controller.h"
Params global_params;

void global_params_Init(void)
{
	global_params.MotorLocked=1;
	global_params.AttitudeDesired.roll=0;
	global_params.AttitudeDesired.pitch=0;
	global_params.AttitudeDesired.yaw=0;
	global_params.AltitudeActual=0;
	global_params.AltitudeDesired=0;
////	global_params.ControlMode=ALT_HOLD;
	
	global_params.HomePoint.longitude=0;
	global_params.HomePoint.latitude=0;
	global_params.HomePoint.altitude=0;
	global_params.HomePoint.hover_time=0;
	global_params.HomePoint.photo=0;
	global_params.HomePoint.speed=0;
}