#include "RemoteControl.h"

uint8_t		SBUS_DMA_buffer[35];
uint16_t  SBUS_data[16];
uint8_t SBUS_update=0;

u8 flight_function=0;
void RemoteControl_Init(void)
{
	global_params.rc[ROLL].max=1690;
	global_params.rc[ROLL].min=309;
	global_params.rc[ROLL].rang=1381;
	global_params.rc[ROLL].mid=1006;
	global_params.rc[ROLL].reverse=1;
	
	global_params.rc[PITCH].max=1690;
	global_params.rc[PITCH].min=309;
	global_params.rc[PITCH].rang=1381;
	global_params.rc[PITCH].mid=1000;
	global_params.rc[PITCH].reverse=1;
	
	global_params.rc[YAW].max=1690;
	global_params.rc[YAW].min=309;
	global_params.rc[YAW].rang=1381;
	global_params.rc[YAW].mid=981;
	global_params.rc[YAW].reverse=1;
	
	global_params.rc[THROTTLE].max=1690;
	global_params.rc[THROTTLE].min=312;
	global_params.rc[THROTTLE].rang=1370;
	global_params.rc[THROTTLE].mid=1000;
	global_params.rc[THROTTLE].reverse = 1;
	
	global_params.rc[MODE].max=1925;
	global_params.rc[MODE].min=74;
	global_params.rc[MODE].rang=1851;
	global_params.rc[MODE].mid=1000;
	global_params.rc[MODE].reverse = 1;
}

void RemoteControlCalibration(void)
{
	
}

void function_select(void)
{
	static uint16_t channel_9_last=0;
	static uint16_t channel_8_last=0;
	static uint16_t channel_6_last=0; 
	if((SBUS_data[9]-channel_9_last)>1000)
	{
		flight_function=3;
		channel_9_last=SBUS_data[9];
		channel_8_last=SBUS_data[8];
		channel_6_last=SBUS_data[6];
		return;
	}
	if((SBUS_data[8]-channel_8_last)>1000)
	{
		flight_function=4;
		channel_9_last=SBUS_data[9];
		channel_8_last=SBUS_data[8];
		channel_6_last=SBUS_data[6];
		return;
	}
	if(SBUS_data[6]-channel_6_last>700)
	{
		flight_function=2;
		channel_9_last=SBUS_data[9];
		channel_8_last=SBUS_data[8];
		channel_6_last=SBUS_data[6];
		return;
	}
	if((SBUS_data[6]-channel_6_last)>1200)
	{
		flight_function=1;
		channel_9_last=SBUS_data[9];
		channel_8_last=SBUS_data[8];
		channel_6_last=SBUS_data[6];
		return;
	}
}

void ReadRemoteControl(void)
{
	uint8_t i;
	int16_t rc_range;
	int16_t rc_mid;
	int8_t rc_reverse;
	int16_t tempCtrl;
	remote_control *rc;
	static float AltitudeDesiredTemp=0;
	static int rc_lock_time=0;
	static int rc_unlock_time=0;
	static uint8_t YawLock=1;
	static uint8_t AltLock=1;
	static uint8_t pitch_pos=0,roll_pos=0;
	static uint8_t start_hover=1;
	rc=&global_params.rc[0];
	
	if(!SBUS_update)
		return;
	else
		SBUS_update=0;
	
	for(i=0;i<5;i++)		//normalize the remote control input,so it's ranging from -500 to 500. 
	{
		rc_range=rc[i].rang;
		rc_mid  =rc[i].mid;
		rc_reverse=rc[i].reverse;
		//check data range.
//		if(SBUS_data[i]<(rc[i].max+10)&&SBUS_data[i]>(rc[i].min-10)) 
//		{
			tempCtrl=rc_reverse*(SBUS_data[i]-rc_mid)*1000/rc_range;
			global_params.rc[i].value=rc_reverse*(SBUS_data[i]-rc_mid)*1000/rc_range;
//		}
//		else
//			return;
	}
	/****unlock the motor******/
	/*  THROTTLE min && ROLL max && PITCH max && YAW max****/
	if(rc[THROTTLE].value<-480&&rc[ROLL].value<-480&&rc[PITCH].value<-480&&rc[YAW].value>480)
	{
		rc_unlock_time++;
		if(rc_unlock_time>=100) 
		{
			global_params.MotorLocked=0;
			rc_unlock_time=0;
			LED3=1;
			GPS_HomeReset();
			controllerIntegReset();
//			DoIntegral();
			pidVerVelocity.wheInteg=true;
			velociyt_est_reset();
			global_params.AttitudeDesired.yaw =Yaw;
			throttle=2200;
			return;
		}
	}
	else
		rc_unlock_time=0;
	/****lock the motor******/
	/*  THROTTLE min && ROLL max && PITCH max && YAW max****/
	//if(rc[THROTTLE].value<50&&rc[ROLL].value<-450&&rc[PITCH].value<-450&&rc[YAW].value>450)
	if(rc[THROTTLE].value<-480&&rc[ROLL].value>480&&rc[PITCH].value<-480&&rc[YAW].value<-480)
	{
		global_params.MotorLocked=1;
		Motor_PwmRefresh(2000,2000,2000,2000);
		throttle=2000;
		LED3=0;
		VERVELCON=1;
		DoNotIntegral();
	}

	//control mode select.
	if(rc[MODE].value>200)
	{
//		if(gps.available&&gps.HomeSet)
		if(pixy_available&&KS103_height>200)  
		{
			global_params.ControlMode=HOVER;
			if(start_hover)
			{
				start_hover=0;
				posDesired[0]=EST_X[0];
				posDesired[1]=EST_Y[0];
				pidVel_X.integ=0;
				pidVel_Y.integ=0;
				pidAcc_X.integ=0;
				pidAcc_Y.integ=0;
			}
		}
		else
			global_params.ControlMode=ALT_HOLD;
		function_select();
	}
	else
	{
			flight_function=0;
			start_hover=1;
			global_params.ControlMode=ALT_HOLD;
	}
	
	if(rc[THROTTLE].value<-450)	return;
	
	if(global_params.ControlMode==ALT_HOLD)
	{
		if(rc[ROLL].value<-20||rc[ROLL].value>20)
		{
			if(rc[ROLL].value<-20)	
				tempCtrl=rc[ROLL].value+20;
			else	
				tempCtrl=rc[ROLL].value-20;
			global_params.AttitudeDesired.roll=tempCtrl*0.065;
		}
		else
			global_params.AttitudeDesired.roll=0;
		if(rc[PITCH].value<-20||rc[PITCH].value>20)
		{
			if(rc[PITCH].value<-20)	
				tempCtrl=rc[PITCH].value+20;
			else	
				tempCtrl=rc[PITCH].value-20;
			global_params.AttitudeDesired.pitch=tempCtrl*0.065;
		}
		else
			global_params.AttitudeDesired.pitch=0;
	}
	else if(global_params.ControlMode==HOVER)
	{
		pitch_pos=roll_pos=1;
		if(rc[PITCH].value<-30||rc[PITCH].value>30)
		{
			if(rc[PITCH].value<-30)	
				tempCtrl=rc[PITCH].value+30;
			else	
				tempCtrl=rc[PITCH].value-30;
			
			global_params.AttitudeDesired.pitch=tempCtrl*0.065;
			
//			posDesired[0]=EST_X[0];
//			posDesired[1]=EST_Y[0];
			posDesired[0]=105;
			posDesired[1]=178;
			
			POS_LOCK=0;
			pitch_pos=0;
			VEL_X_CTRL=0;
		}	
		else 
		{
			VEL_X_CTRL=1;
			global_params.VelocityDesired.x=0;
		}
		if(rc[ROLL].value<-30||rc[ROLL].value>30)
		{
			if(rc[ROLL].value<-30)	
				tempCtrl=rc[ROLL].value+30;
			else	
				tempCtrl=rc[ROLL].value-30;
			
			global_params.AttitudeDesired.roll=tempCtrl*0.065;
			
//			posDesired[0]=EST_X[0];
//			posDesired[1]=EST_Y[0];
			
			posDesired[0]=105;
			posDesired[1]=178;
			
			POS_LOCK=0;
			roll_pos=0;
			VEL_Y_CTRL=0;
		}
		else
		{
			VEL_Y_CTRL=1;
			global_params.VelocityDesired.y=0;
		}
		if(pitch_pos&&roll_pos)
		{
			if(POS_LOCK==0&&EST_X[1]<0.5&&EST_X[1]>-0.5&&EST_Y[1]<0.5&&EST_Y[1]>-0.5)
			{
				POS_LOCK=1;
//				posDesired[0]=EST_X[0];
//				posDesired[1]=EST_Y[0];
				posDesired[0]=105;
				posDesired[1]=178;
			}
		}
	}
	if(rc[YAW].value<-30||rc[YAW].value>30)
	{
		if(rc[YAW].value<-30)
			tempCtrl=rc[YAW].value + 30;
		else
			tempCtrl=rc[YAW].value - 30;
		YAWRATECON=1; // yaw axis enter rate control model.
		YawLock=0;
		yawRateDesired = -tempCtrl*0.012;
		global_params.AttitudeDesired.yaw =Yaw;
	}
	else
	{
//		yawRateDesired=0;
//		if(YawLock==0&&fabs(body_gyro.z)<0.08)
//		{
//			YawLock=1;
//			YAWRATECON=0;
//			global_params.AttitudeDesired.yaw =Yaw;
//		}
		YawLock=1;
		YAWRATECON=0;
	}
	
	if(flight_function)	return;
	
	//altitude control model
	if(rc[THROTTLE].value<-100||rc[THROTTLE].value>100)
	{
		if(rc[THROTTLE].value<-100)
		{
			tempCtrl=rc[THROTTLE].value + 100;
			VerVelDesired=tempCtrl*0.005; //max -1m/s
		}
		else
		{
			tempCtrl=rc[THROTTLE].value - 100;
			VerVelDesired=tempCtrl*0.01; //max 4m/s
		}
		AltLock=0;
		global_params.AltitudeDesired = global_params.AltitudeActual;
		VERVELCON=1;
	}
	else 
	{
		VerVelDesired=0;
		if(AltLock==0&&VerVelEst<0.1&&VerVelEst>-0.1)
		{
			AltLock=1;
			VERVELCON=0;
			global_params.AltitudeDesired = global_params.AltitudeActual;
		}
	}
}

