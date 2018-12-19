#include "controller.h"
#include "mpu6500.h"
#include "imu.h"
#include "delay.h"
#include "pid.h"
#include "params.h"
#include "motor.h"
#include <stdbool.h>
#include <math.h>

int16_t motor1,motor2,motor3,motor4;
int16_t throttle=2000;
int16_t throttleDesired=2000;
//angualr acceleration control
PidObject	pidRollAcc;
PidObject	pidPitchAcc;
PidObject pidYawAcc;
//rotate control
PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;
//attitude control
PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;
//altitude control
PidObject pidAltitude;	
PidObject pidVerVelocity;
//position control

//velocity control
PidObject pidVel_X;
PidObject pidVel_Y;

//flight acceleration control
PidObject pidAcc_X;
PidObject pidAcc_Y;

int16_t rollOutput;
int16_t pitchOutput;
int16_t yawOutput;
int16_t AltitudeOutPut=0;

float rollAccActual=0;
float pitchAccActual=0;
float yawAccActual=0;

float rollAccDesired;
float pitchAccDesired;
float yawAccDesired;

float rollRateDesired;
float pitchRateDesired;
float yawRateDesired;

float rollDesired=0;
float pitchDesired=0;
float yawDesired=0;

float AltitudeDesired=0;
float AltitudeActual=0;
float VerVelDesired=0;
float VerVelActual=0;

//int16_t collective_pitch=0;//总距
//int16_t longitude_pitch=0;//纵向变距
//int16_t horizont_pitch=0;//横向变距
//int16_t head_pitch=0;    //航向变距

float HorAccDesired[2];

float VelDesired[2];
float vel_ctrl_dt=0.04;
float alt_ctrl_dt=0.01;

uint8_t RATECON=1;
uint8_t YAWRATECON=0;
uint8_t VERVELCON=1;

uint8_t POS_LOCK=0;
uint8_t VEL_X_CTRL=1,VEL_Y_CTRL=1;
float posDesired[2];

int16_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}
void controllerInit()
{
  //TODO: get parameters from configuration manager instead
	
	pidInit(&pidRollAcc,0,PID_ROLL_ACC_KP,PID_ROLL_ACC_KI,PID_ROLL_ACC_KD,IMU_UPDATE_DT);
	pidInit(&pidPitchAcc,0,PID_PITCH_ACC_KP,PID_PITCH_ACC_KI,PID_PITCH_ACC_KD,IMU_UPDATE_DT);
	pidInit(&pidYawAcc,0,PID_YAW_ACC_KP,PID_YAW_ACC_KI,PID_YAW_ACC_KD,IMU_UPDATE_DT);
	
	pidSetIntegralLimit(&pidRollAcc, 200);
	pidSetIntegralLimitLow(&pidRollAcc,-200);
	
	pidSetIntegralLimit(&pidPitchAcc, 200);
	pidSetIntegralLimitLow(&pidPitchAcc,-200);
	
	pidSetIntegralLimit(&pidYawAcc, 200);
	pidSetIntegralLimitLow(&pidYawAcc,-200);
	
	pidRollAcc.wheInteg=false;
	pidPitchAcc.wheInteg=false;
	pidYawAcc.wheInteg=false;
	
  pidInit(&pidRollRate, 0, PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD, IMU_UPDATE_DT);
  pidInit(&pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD, IMU_UPDATE_DT);
  pidInit(&pidYawRate, 0, PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD, IMU_UPDATE_DT);

  pidSetIntegralLimit(&pidRollRate, PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYawRate, PID_YAW_RATE_INTEGRATION_LIMIT);
	 
	pidSetIntegralLimitLow(&pidRollRate, PID_ROLL_RATE_INTEGRATION_LIMIT_LOW);
  pidSetIntegralLimitLow(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT_LOW);
  pidSetIntegralLimitLow(&pidYawRate, PID_YAW_RATE_INTEGRATION_LIMIT_LOW);
	
  pidInit(&pidRoll, 0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, IMU_UPDATE_DT);
  pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, IMU_UPDATE_DT);
  pidInit(&pidYaw, 0, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, IMU_UPDATE_DT);
	pidInit(&pidAltitude,0,PID_ALTITUDE_KP,PID_ALTITUDE_KI,PID_ALTITUDE_KD,ATlTITUDE_UPDATE_DT);
	pidInit(&pidVerVelocity,0,VER_VEL_KP,VER_VEL_KI,VER_VEL_KD,VER_VEL_UPDATE_DT);
	pidInit(&pidVel_X,0,PID_VEL_X_KP,PID_VEL_X_KI,PID_VEL_X_KD,IMU_UPDATE_DT);
	pidInit(&pidVel_Y,0,PID_VEL_Y_KP,PID_VEL_Y_KI,PID_VEL_Y_KD,IMU_UPDATE_DT);
	
	pidInit(&pidAcc_X,0,PID_ACC_X_KP,PID_ACC_X_KI,PID_ACC_X_KD,0.01);
	pidInit(&pidAcc_Y,0,PID_ACC_Y_KP,PID_ACC_Y_KI,PID_ACC_Y_KD,0.01);
	
  pidSetIntegralLimit(&pidRoll, PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYaw, PID_YAW_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pidAltitude, PID_ALTITUDE_INTEGRATION_LIMIT);
	pidSetIntegralLimitLow(&pidAltitude,PID_ALTITUDE_INTEGRATION_LIMIT_LOW);
	pidSetIntegralLimit(&pidVerVelocity, VER_VEL_INTEGRATION_LIMIT);
	pidSetIntegralLimitLow(&pidVerVelocity,VER_VEL_INTEGRATION_LIMIT_LOW);
	
	pidSetIntegralLimit(&pidVel_X, PID_VEL_X_INTEGRATION_LIMIT);
	pidSetIntegralLimitLow(&pidVel_X,PID_VEL_X_INTEGRATION_LIMIT_LOW);
	pidSetIntegralLimit(&pidVel_Y, PID_VEL_Y_INTEGRATION_LIMIT);
	pidSetIntegralLimitLow(&pidVel_Y,PID_VEL_Y_INTEGRATION_LIMIT_LOW);
	
	pidSetIntegralLimit(&pidAcc_X, PID_ACC_X_INTEGRATION_LIMIT);
	pidSetIntegralLimitLow(&pidAcc_X,PID_ACC_X_INTEGRATION_LIMIT_LOW);
	pidSetIntegralLimit(&pidAcc_Y, PID_ACC_Y_INTEGRATION_LIMIT);
	pidSetIntegralLimitLow(&pidAcc_Y,PID_ACC_Y_INTEGRATION_LIMIT_LOW);
}

void PidControllerStopIntegral(void)
{
	pidSetWhetherIntegral(&pidRollRate,false);
	pidSetWhetherIntegral(&pidPitchRate,false);
	
	pidClearIntegral(&pidRollRate);
	pidClearIntegral(&pidPitchRate);

}
//	float yaw_Weight=1;
void controllerCorrectAttitudePID(float eulerRollActual, float eulerPitchActual, float eulerYawActual,
																	float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
																	float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{
	float yaw_Weight=1;
	float w_temp;
	float roll,pitch;
	float yawError;
	// Update PID for roll axis
  pidSetDesired(&pidRoll, eulerRollDesired);
	pidSetDt(&pidRoll,imu_update_dt);
  *rollRateDesired = pidUpdate(&pidRoll, eulerRollActual);
  // Update PID for pitch axis
  pidSetDesired(&pidPitch, eulerPitchDesired);
	pidSetDt(&pidPitch,imu_update_dt);
  *pitchRateDesired = pidUpdate(&pidPitch, eulerPitchActual);
  // Update PID for yaw axis
	roll=pidRoll.error*0.01745;//to radian
	pitch=pidPitch.error*0.01745;
	w_temp=arm_cos_f32(roll)*arm_cos_f32(pitch);
	yaw_Weight=w_temp*w_temp;
	
  yawError = eulerYawDesired - eulerYawActual;
  if (yawError > 180)
	{
    yawError -= 360;
	}
  else if (yawError < -180)
	{
		yawError += 360;
	}
	eulerYawDesired=eulerYawActual+yawError;
	pidSetDesired(&pidYaw,eulerYawDesired);
	pidSetDt(&pidYaw,imu_update_dt);
	if(!YAWRATECON)
	{
		*yawRateDesired = pidUpdate(&pidYaw, eulerYawActual);
	}
}

void controllerCorrectRatePID(float rollRateActual, float pitchRateActual, float yawRateActual,
															float rollRateDesired, float pitchRateDesired, float yawRateDesired)
{
	if(RATECON)
	{
		pidSetDesired(&pidRollRate, rollRateDesired);
		pidSetDt(&pidRollRate,imu_update_dt);
		//rollOutput = saturateSignedInt16(pidUpdate(&pidRollRate, rollRateActual));
		rollAccDesired=pidUpdate(&pidRollRate,rollRateActual);
		
		pidSetDesired(&pidPitchRate, pitchRateDesired);
		pidSetDt(&pidPitchRate,imu_update_dt);
		//pitchOutput = saturateSignedInt16(pidUpdate(&pidPitchRate, pitchRateActual));
		pitchAccDesired=pidUpdate(&pidPitchRate,pitchRateActual);
		
		pidSetDesired(&pidYawRate, yawRateDesired);
		pidSetDt(&pidYawRate,imu_update_dt);
		//yawOutput = saturateSignedInt16(pidUpdate(&pidYawRate, yawRateActual));
		yawAccDesired=pidUpdate(&pidYawRate,yawRateActual);
	}
	else
	{
		rollOutput=rollRateDesired;
		pitchOutput=pitchRateDesired;
		yawOutput=yawRateDesired;
	}
}

void controllerCorrectAngularAccPID(float rollAccActual,float pitchAccActual,float yawAccActual,
																		float rollAccDesired,float pitchAccDesired,float yawAccDesired)
{
	pidSetDesired(&pidRollAcc, rollAccDesired);
	pidSetDt(&pidRollAcc,imu_update_dt);	
	rollOutput = saturateSignedInt16(pidUpdate(&pidRollAcc, rollAccActual));
	
	pidSetDesired(&pidPitchAcc, pitchAccDesired);
	pidSetDt(&pidPitchAcc,imu_update_dt);	
	pitchOutput = saturateSignedInt16(pidUpdate(&pidPitchAcc, pitchAccActual));
	
	pidSetDesired(&pidYawAcc, yawAccDesired);
	pidSetDt(&pidYawAcc,imu_update_dt);	
	yawOutput = saturateSignedInt16(pidUpdate(&pidYawAcc, yawAccActual));
	
}

//高度调节
void controllerCorrectAltitudePID(float AltitudeActual,float AltitudeDesired,float *VerVelOutput)
{
	pidSetDesired(&pidAltitude,AltitudeDesired);
	pidSetDt(&pidAltitude,alt_ctrl_dt);
	if(!VERVELCON)
	{
		*VerVelOutput=pidUpdate(&pidAltitude,AltitudeActual);
		if(*VerVelOutput>10)	*VerVelOutput=10;
		else if(*VerVelOutput<-10) *VerVelOutput=-10;
	}
}
//速度调节
void controllerCorrectVerVelPID(float VelActual,float VelDesired)
{
	pidSetDesired(&pidVerVelocity,VelDesired);
	pidSetDt(&pidVerVelocity,alt_ctrl_dt);
	AltitudeOutPut=pidUpdate(&pidVerVelocity,VelActual);
	if(AltitudeOutPut>2000)AltitudeOutPut=2000;
	if(AltitudeOutPut<0)AltitudeOutPut=0;
}
void conrollerCorrectPositionPID(float pos_x,float pos_y,float posDesired[2],float velDesired[2])
{
	float vel_x,vel_y;
	
	vel_x=(posDesired[0]-pos_x)*0.01;
	if(vel_x>4)vel_x=4;
	else if(vel_x<-4)vel_x=-4;
		
	vel_y=(posDesired[1]-pos_y)*0.01;
	if(vel_y>4)vel_y=4;
	else if(vel_y<-4)vel_y=-4;
	
	velDesired[0]=-vel_x;
	velDesired[1]=-vel_y;
	if(velDesired[0]>1)velDesired[0]=1;
	if(velDesired[1]>1)velDesired[1]=1;
	
	if(velDesired[0]<-1)velDesired[0]=-1;
	if(velDesired[1]<-1)velDesired[0]=-1;
//	RotateVelToBody(velDesired,vel_x,vel_y);
	
}
void controllerCorrectVelocityPID(float VelActual[2],float VelDesired[2],float *PitchDesired,float *RollDesired)
{
	//calculate the desired pitch angle.
	if(VEL_X_CTRL)
	{
		pidSetDesired(&pidVel_X,VelDesired[0]); 
		pidSetDt(&pidVel_X,vel_ctrl_dt);
		*PitchDesired=pidUpdate(&pidVel_X,VelActual[0]);
		if(*PitchDesired>8)	*PitchDesired=8;
		else if(*PitchDesired<-8)	*PitchDesired=-8;
	}
	else
	{
		*PitchDesired=global_params.AttitudeDesired.pitch;
	}
	//calculate the desired roll angle.
	if(VEL_Y_CTRL)
	{
		pidSetDesired(&pidVel_Y,VelDesired[1]); 
		pidSetDt(&pidVel_Y,vel_ctrl_dt);
		*RollDesired=-pidUpdate(&pidVel_Y,VelActual[1]);
		if(*RollDesired>8)		*RollDesired=8;
		else if(*RollDesired<-8)		*RollDesired=-8;
	}
	else
	{
		*RollDesired=global_params.AttitudeDesired.roll;
	}
}


int16_t throttle_C;
void PIDcontrolOutput(void)
{
	float fac=GetCompensationFactor(angles);
	
	if(throttle+AltitudeOutPut<2700)
	{
		DoNotIntegral();
		pidVerVelocity.wheInteg=true;
	}
	else
	{
		DoIntegral();
	}
	throttle_C=(float)(throttle+AltitudeOutPut)/(fac);
//	motor1 = throttle + rollOutput - pitchOutput + yawOutput + AltitudeOutPut;
//	motor2 = throttle - rollOutput - pitchOutput - yawOutput + AltitudeOutPut;
//	motor3 = throttle - rollOutput + pitchOutput + yawOutput + AltitudeOutPut;
//	motor4 = throttle + rollOutput + pitchOutput - yawOutput + AltitudeOutPut;

	motor1 = throttle_C + rollOutput - pitchOutput + yawOutput;
	motor2 = throttle_C - rollOutput - pitchOutput - yawOutput;
	motor3 = throttle_C - rollOutput + pitchOutput + yawOutput;
	motor4 = throttle_C + rollOutput + pitchOutput - yawOutput;
	if(motor1<2000)motor1=2000;
	if(motor2<2000)motor2=2000;
	if(motor3<2000)motor3=2000;
	if(motor4<2000)motor4=2000;
//	motor1=motor2=motor3=motor4=throttle;
	if(!global_params.MotorLocked)
	{
		Motor_PwmRefresh(motor1,motor2,motor3,motor4);
	}
}

void GetAngularAcceleration_2(float rollRateActual,float pitchRateActual,float yawRateActual)
{
	static float rollRateLast=0;
	static float pitchRateLast=0;
	static float yawRateLast=0;
	
	rollAccActual=(rollRateActual-rollRateLast)/imu_update_dt;
	pitchAccActual=(pitchRateActual-pitchRateLast)/imu_update_dt;
	yawAccActual=(yawRateActual-yawRateLast)/imu_update_dt;
	
	rollRateLast=rollRateActual;
	pitchRateLast=pitchRateActual;
	yawRateLast=yawRateActual;
}

void FlightControl(void)
{
	static u32 vel_ctrl_time_now,vel_ctrl_time_last;
	static u32 alt_ctrl_time_now,alt_ctrl_time_last;
	static uint8_t pos_count=0;
	static uint8_t vel_ctrl_first=1;
	static uint8_t alt_ctrl_first=1;
	static uint8_t alt_count=0;

	if(global_params.ControlMode==ALT_HOLD)
	{
		rollDesired=global_params.AttitudeDesired.roll;
		pitchDesired=global_params.AttitudeDesired.pitch;
		
		pos_count=0;
		vel_ctrl_first=1;
	}
	else if(global_params.ControlMode==HOVER)
	{
		if(pos_count==5)//5
		{
			pos_count=0;
			get_dt_in_seconds(&vel_ctrl_time_now,&vel_ctrl_time_last,&vel_ctrl_dt);
			if(vel_ctrl_first)
			{
				vel_ctrl_first=0;
				vel_ctrl_dt=0.01;
			}
			
			if(POS_LOCK)
			{
//				conrollerCorrectPositionPID(EST_X[0],EST_Y[0],posDesired,VelDesired);
				conrollerCorrectPositionPID(pixy_PosX,pixy_PosY,posDesired,VelDesired);
			}
			else
			{
//				VelDesired[0]=global_params.VelocityDesired.x;
//				VelDesired[1]=global_params.VelocityDesired.y;
				VelDesired[0]=0;
				VelDesired[1]=0;
			}
			body_vel[0]=pixy_v_est_x;
			body_vel[1]=pixy_v_est_y;
			controllerCorrectVelocityPID(body_vel,VelDesired,&pitchDesired,&rollDesired);
		}
		pos_count++;
	}
	
	if(global_params.MotorLocked)
	{
		PidControllerStopIntegral();
	}
	
	yawDesired=global_params.AttitudeDesired.yaw;
	AltitudeActual=global_params.AltitudeActual;
	AltitudeDesired=global_params.AltitudeDesired;
	if(alt_count==5)
	{
		alt_count=0;
		get_dt_in_seconds(&alt_ctrl_time_now,&alt_ctrl_time_last,&alt_ctrl_dt);
		if(alt_ctrl_first)
		{
			alt_ctrl_first=0;
			alt_ctrl_dt=0.01;
		}
		controllerCorrectAltitudePID(AltitudeActual,AltitudeDesired,&VerVelDesired);
		controllerCorrectVerVelPID(VerVelEst,VerVelDesired);
	}
	alt_count++;
	//attitude control.
	controllerCorrectAttitudePID(Roll,Pitch,Yaw,
															 rollDesired,pitchDesired,yawDesired,
															 &rollRateDesired,&pitchRateDesired,&yawRateDesired);
	controllerCorrectRatePID(body_gyro.x,body_gyro.y,body_gyro.z,
													 rollRateDesired,pitchRateDesired,yawRateDesired);
	GetAngularAcceleration_2(body_gyro.x,body_gyro.y,body_gyro.z);
	controllerCorrectAngularAccPID(rollAccActual,pitchAccActual,yawAccActual,rollAccDesired,pitchAccDesired,yawAccDesired);
	
	PIDcontrolOutput();
}

float GetCompensationFactor(float angles[3])
{
	float roll,pitch;
	float z=1;
	
	roll =angles[0];
	pitch=angles[1];
	
	
	if(roll>0.42)		roll=0.42;
	if(roll<-0.42)	roll=-0.42;
	if(pitch>0.42)	pitch=0.42;
	if(pitch<-0.42)	pitch=-0.42;
		
	z=cos(roll)*cos(pitch);
	if(z<0)
		z=-z;
	
	return z;
}

void controllerIntegReset(void)
{
	pidRollRate.integ=0;
	pidPitchRate.integ=0;
	pidYawRate.integ=0;
	
	pidRollAcc.integ=0;
	pidPitchAcc.integ=0;
	pidYawAcc.integ=0;
	
	pidYawAcc.error=0;
	pidYawAcc.prevError=0;
	pidYawAcc.deriv=0;
	
	pidVerVelocity.integ=0;
	pidVerVelocity.error=0;
	pidVerVelocity.prevError=0;
	pidVerVelocity.deriv=0;
	
	pidVel_X.integ=0;
	pidVel_Y.integ=0;
	
	pidAcc_X.integ=0;
	pidAcc_Y.integ=0;
}


void DoNotIntegral(void)
{
	pidRollAcc.wheInteg=false;
	pidPitchAcc.wheInteg=false;
	pidYawAcc.wheInteg=false;
	
	pidRollRate.wheInteg=false;
	pidPitchRate.wheInteg=false;
	pidYawRate.wheInteg=false;
	//altitude control
	pidVerVelocity.wheInteg=false;
	//position control

	//velocity control
	pidVel_X.wheInteg=false;
	pidVel_Y.wheInteg=false;
	
	pidAcc_X.wheInteg=false;
	pidAcc_Y.wheInteg=false;
}

void DoIntegral(void)
{
	pidRollAcc.wheInteg=true;
	pidPitchAcc.wheInteg=true;
	pidYawAcc.wheInteg=true;
	
	pidRollRate.wheInteg=true;
	pidPitchRate.wheInteg=true;
	pidYawRate.wheInteg=true;
	//altitude control
	pidVerVelocity.wheInteg=true;
	//position control

	//velocity control
	pidVel_X.wheInteg=true;
	pidVel_Y.wheInteg=true;
	pidAcc_X.wheInteg=true;
	pidAcc_Y.wheInteg=true;
	
}

