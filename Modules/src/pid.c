#include "pid.h"
void pidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
  pid->desired = desired;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->iLimit    = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->iLimitLow = -DEFAULT_PID_INTEGRATION_LIMIT;
  pid->dt        = dt;
	pid->wheInteg  = false;
}
void pidParameterSet(PidObject* pid,float kp,float ki,float kd)
{
	pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}
float pidUpdate(PidObject* pid, const float measured)
{
    float output;
		
		pid->error = pid->desired - measured;
    if (pid->wheInteg)
    {
				pid->integ += pid->error * pid->dt;
    }
    if (pid->integ > pid->iLimit)
    {
        pid->integ = pid->iLimit;
    }
    else if (pid->integ < pid->iLimitLow)
    {
        pid->integ = pid->iLimitLow;
    }
    pid->deriv = (pid->error - pid->prevError) / pid->dt;

    pid->outP = pid->kp * pid->error;
    pid->outI = pid->ki * pid->integ;
    pid->outD = pid->kd * pid->deriv;
    output = pid->outP + pid->outI + pid->outD;

    pid->prevError = pid->error;

    return output;
}
void pidSetError(PidObject* pid, const float error)
{
  pid->error = error;
}

void pidSetDesired(PidObject* pid, const float desired)
{
  pid->desired = desired;
}

float pidGetDesired(PidObject* pid)
{
  return pid->desired;
}

void pidSetIntegralLimit(PidObject* pid, const float limit) {
    pid->iLimit = limit;
}

void pidSetIntegralLimitLow(PidObject* pid, const float limitLow) {
    pid->iLimitLow = limitLow;
}

void pidSetWhetherIntegral(PidObject *pid,bool wheInteg)
{
	pid->wheInteg = wheInteg;
}
void pidClearIntegral(PidObject *pid)
{
	pid->integ=0;
	pid->outI=0;
}
void pidSetKp(PidObject* pid, const float kp)
{
  pid->kp = kp;
}

void pidSetKi(PidObject* pid, const float ki)
{
  pid->ki = ki;
}

void pidSetKd(PidObject* pid, const float kd)
{
  pid->kd = kd;
}
void pidSetDt(PidObject* pid, const float dt) 
{
    pid->dt = dt;
}

//ÔöÁ¿Ê½PID
float incPidUpdate(IncPidObject* pid, const float measured)
{
	float output;
	pid->curError=pid->desired-measured;
	output=pid->Kp*pid->curError-pid->Ki*pid->lastError+pid->Kd*pid->prevError;
	pid->prevError=pid->lastError;
	pid->lastError=pid->curError;
	return output;
}
void incPidInit(IncPidObject *pid,float Kp,float Ki,float Kd,float desired)
{
	pid->Kp=Kp;
	pid->Ki=Ki;
	pid->Kd=Kd;
	pid->desired=desired;
}
void incPidSetDesired(IncPidObject *pid,float desired)
{
	pid->desired=desired;
}























