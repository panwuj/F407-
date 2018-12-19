#ifndef __PID_H
#define __PID_H

#include <stdbool.h>
//#include "params.h"

#define PID_ROLL_ACC_KP		6
#define PID_ROLL_ACC_KI		6
#define PID_ROLL_ACC_KD		0

#define PID_PITCH_ACC_KP		6
#define PID_PITCH_ACC_KI		6
#define PID_PITCH_ACC_KD		0

#define PID_YAW_ACC_KP		6
#define PID_YAW_ACC_KI		6
#define PID_YAW_ACC_KD		0
//rate control
#define PID_ROLL_RATE_KP  22   //1.4
#define PID_ROLL_RATE_KI  0
#define PID_ROLL_RATE_KD  1.0 
#define PID_ROLL_RATE_INTEGRATION_LIMIT         300
#define PID_ROLL_RATE_INTEGRATION_LIMIT_LOW    -300

#define PID_PITCH_RATE_KP  22
#define PID_PITCH_RATE_KI  0
#define PID_PITCH_RATE_KD  1.0
#define PID_PITCH_RATE_INTEGRATION_LIMIT        300
#define PID_PITCH_RATE_INTEGRATION_LIMIT_LOW   -300

#define PID_YAW_RATE_KP  30 //1
#define PID_YAW_RATE_KI  0
#define PID_YAW_RATE_KD  1.0//0.12
#define PID_YAW_RATE_INTEGRATION_LIMIT         300
#define PID_YAW_RATE_INTEGRATION_LIMIT_LOW    -300

#define PID_ROLL_KP  0.0175*6.5			//5
#define PID_ROLL_KI  0.0
#define PID_ROLL_KD  0.0
#define PID_ROLL_INTEGRATION_LIMIT    400

#define PID_PITCH_KP  0.0175*6.5
#define PID_PITCH_KI  0.0
#define PID_PITCH_KD  0.0
#define PID_PITCH_INTEGRATION_LIMIT   400

#define PID_YAW_KP  0.0175*8         //2
#define PID_YAW_KI  0
#define PID_YAW_KD  0
#define PID_YAW_INTEGRATION_LIMIT     100

#define DEFAULT_PID_INTEGRATION_LIMIT  5000.0  //默认积分上限
#define IMU_UPDATE_DT 0.01										 //四元数更新时间/s

//高度PID
#define ATlTITUDE_UPDATE_DT 0.01
#define PID_ALTITUDE_KP    0.001  //1 for barometer  //0.00015 for sonar
#define PID_ALTITUDE_KI    0
#define PID_ALTITUDE_KD    0
#define PID_ALTITUDE_INTEGRATION_LIMIT          300
#define PID_ALTITUDE_INTEGRATION_LIMIT_LOW     -100
//vertical velocity pid parameters
#define VER_VEL_UPDATE_DT  0.01
#define VER_VEL_KP				 300
#define VER_VEL_KI				 250
#define VER_VEL_KD				 30
#define VER_VEL_INTEGRATION_LIMIT         10
#define VER_VEL_INTEGRATION_LIMIT_LOW     -10

//speed control
#define PID_VEL_X_KP         6
#define PID_VEL_X_KI				 0
#define PID_VEL_X_KD				 0.025
#define PID_VEL_X_INTEGRATION_LIMIT        20
#define PID_VEL_X_INTEGRATION_LIMIT_LOW   -20

#define PID_VEL_Y_KP         6
#define PID_VEL_Y_KI				 0
#define PID_VEL_Y_KD				 0.025
#define PID_VEL_Y_INTEGRATION_LIMIT        20
#define PID_VEL_Y_INTEGRATION_LIMIT_LOW   -20

#define PID_ACC_X_KP				3
#define PID_ACC_X_KI				0
#define PID_ACC_X_KD				0
#define PID_ACC_X_INTEGRATION_LIMIT        20
#define PID_ACC_X_INTEGRATION_LIMIT_LOW   -20

#define PID_ACC_Y_KP				3
#define PID_ACC_Y_KI				0
#define PID_ACC_Y_KD				0
#define PID_ACC_Y_INTEGRATION_LIMIT        20
#define PID_ACC_Y_INTEGRATION_LIMIT_LOW   -20


typedef struct
{
  float desired;      //< set point
  float error;        //< error
  float prevError;    //< previous error
  float integ;        //< integral
  float deriv;        //< derivative
  float kp;           //< proportional gain
  float ki;           //< integral gain
  float kd;           //< derivative gain
  float outP;         //< proportional output (debugging)
  float outI;         //< integral output (debugging)
  float outD;         //< derivative output (debugging)
  float iLimit;       //< integral limit
  float iLimitLow;    //< integral limit
  float dt;           //< delta-time dt
	bool  wheInteg;     //< whether intergral
} PidObject;

typedef struct
{
	float desired;
	float curError;
	float Kp;
	float Ki;
	float Kd;
	float lastError;
	float prevError;
}IncPidObject;

void pidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt);
void pidSetIntegralLimit(PidObject* pid, const float limit);
void pidSetIntegralLimitLow(PidObject* pid, const float limitLow);
float pidUpdate(PidObject* pid, const float measured);
void pidSetDesired(PidObject* pid, const float desired);
void pidSetError(PidObject* pid, const float error);
float pidGetDesired(PidObject* pid);
void pidClearIntegral(PidObject *pid);
void pidSetWhetherIntegral(PidObject *pid,bool wheInteg);
void pidSetDt(PidObject* pid, const float dt);
void pidSetKp(PidObject* pid, const float kp);
void pidSetKi(PidObject* pid, const float ki);
void pidSetKd(PidObject* pid, const float kd);
void pidParameterSet(PidObject* pid,const float kp,const float ki, const float kd);

void incPidSetDesired(IncPidObject *pid,float desired);
void incPidInit(IncPidObject *pid,float Kp,float Ki,float Kd,float desired);
float incPidUpdate(IncPidObject* pid, const float measured);
#endif
