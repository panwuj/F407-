#ifndef __CONTROLLER_H
#define __CONTROLLER_H
#include "pid.h"
#include "sys.h"

void controllerInit(void);
void controllerCorrectRatePID(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired);
			 
void controllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired);
void FlightControl(void);			 
void controllerCorrectAltituduPID(float AltitudeActual,float AltitudeDesired,float* VerticalAccDesired);
float GetCompensationFactor(float angles[3]);
void controllerIntegReset(void);
extern PidObject pidRollRate;
extern PidObject pidPitchRate;
extern PidObject pidYawRate;
extern PidObject pidRoll;
extern PidObject pidPitch;
extern PidObject pidYaw;
extern PidObject pidAltitude;	
extern PidObject pidVerVelocity;			 
extern PidObject pidVel_X;
extern PidObject pidVel_Y;
extern PidObject pidRollAcc;
extern PidObject pidPitchAcc;
extern PidObject pidYawAcc;
extern PidObject pidAcc_X;
extern PidObject pidAcc_Y;
			 
extern float rollDesired;
extern float pitchDesired;
extern float yawDesired;	
extern float yawRateDesired;			 
extern float rollRateDesired;
extern float pitchRateDesired;
extern uint8_t YAWRATECON;		

extern float VerVelDesired;
extern uint8_t VERVELCON;
extern uint8_t POS_LOCK;
extern uint8_t VEL_X_CTRL,VEL_Y_CTRL;
extern float vel_ctrl_dt;

extern int16_t AltitudeOutPut;
extern int16_t rollOutput;
extern int16_t pitchOutput;
extern int16_t yawOutput;
			 
extern int16_t motor1,motor2,motor3,motor4;
extern int16_t throttle;
extern int16_t throttleDesired;

extern float VelDesired[2];
extern float posDesired[2];

void DoNotIntegral(void);
void DoIntegral(void);
#endif
