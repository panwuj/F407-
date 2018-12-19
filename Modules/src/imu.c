#include "imu.h"
#include "math.h"

float Pitch=0,Roll=0,Yaw=0;
float imu_update_dt=0.01;
float q[4]={0,0,0,0};
float rot[9];
//×ËÌ¬¼ÆËã
void GetAttitude(void)
{
		static uint32_t ahrs_time_now=0,ahrs_time_last=0;
		static uint8_t fun_first_run=1;
		float dt=0;
	
		MPU6050_Get_Motion6();
		HMC5883L_Get_Magnetism();
	
		get_dt_in_seconds(&ahrs_time_now,&ahrs_time_last,&dt);
	
		ahrs_time_now=micros();
		if(fun_first_run)
		{
			fun_first_run=0;
			dt=0.01;
		}
		imu_update_dt=dt;
		
		AttitudeEKF(body_gyro.x,body_gyro.y,body_gyro.z,mpu6050.ax,mpu6050.ay,mpu6050.az,
								hmc5883l.y,-hmc5883l.x,hmc5883l.z,dt);
		
//		AttitudeCF(dt);
}

void GetHorizontalAcceleration(float angles[3])
{
	float roll,pitch;
	float sin_r;
	float cos_r;
	float sin_p;
	float cos_p;
	//float norm;
	roll =angles[0];
	pitch=angles[1];
	sin_r=arm_sin_f32(roll);
	sin_p=arm_sin_f32(pitch);
	cos_r=arm_cos_f32(roll);
	cos_p=arm_cos_f32(pitch);
	
	hor_acc.x=cos_p*body_acc.x+sin_r*sin_p*body_acc.y+cos_r*sin_p*body_acc.z;
	hor_acc.y=cos_r*body_acc.y-sin_r*body_acc.z;
	hor_acc.z=-sin_p*body_acc.x+sin_r*cos_p*body_acc.y+cos_r*cos_p*body_acc.z;
	
	hor_acc.x -=0;
	hor_acc.y -=0.09;
	hor_acc.z -=9.23;
}

void GetRotationMatrix(float q[4],float rot[9]) 
{
	float q0q0 = q[0]*q[0];
  float q0q1 = q[0]*q[1];
  float q0q2 = q[0]*q[2];
  float q0q3 = q[0]*q[3];
  float q1q1 = q[1]*q[1];
  float q1q2 = q[1]*q[2];
  float q1q3 = q[1]*q[3];
  float q2q2 = q[2]*q[2];
  float q2q3 = q[2]*q[3];
  float q3q3 = q[3]*q[3];
	rot[0] = 2*(0.5f - q2q2 - q3q3);
	rot[1] = 2*(q1q2 - q0q3);
	rot[2] = 2*(q1q3 + q0q2);
	
	rot[3] = 2*(q1q2 + q0q3);
	rot[4] = 2*(0.5f - q1q1 - q3q3);
	rot[5] = 2*(q2q3 - q0q1);
	
	rot[6] = 2*(q1q3 - q0q2);
	rot[7] = 2*(q2q3 + q0q1);
	rot[8] = 2*(0.5f - q1q1 - q2q2); 
}
void RotateVectorToBody(float v_n[3],float v_b[3])
{
	uint8_t i;
	for(i=0;i<3;i++)
	{
		v_b[i]=rot[i]*v_n[0]+rot[i+3]*v_n[1]+rot[i+6]*v_n[2];
	}
}
void RotateVectorToNED(float v_b[3],float v_n[3])
{
	uint8_t i;
	for(i=0;i<3;i++)
	{
		v_n[i]=rot[i*3]*v_b[0]+rot[i*3+1]*v_b[1]+rot[i*3+2]*v_b[2];
	}
}

void GetNEDAcceleration(void)
{
//	float yaw;
//	float cos_yaw;
//	float sin_yaw;
//	
//	yaw=angles[2];
//	cos_yaw=cos(yaw);
//	sin_yaw=sin(yaw);
//	
//	ned_acc.x=hor_acc.x*cos_yaw-hor_acc.y*sin_yaw;
//	ned_acc.y=hor_acc.x*sin_yaw+hor_acc.y*cos_yaw;
//	ned_acc.z=hor_acc.z;
		
	ned_acc.x=rot[0]*body_acc.x+rot[1]*body_acc.y+rot[2]*body_acc.z;
	ned_acc.y=rot[3]*body_acc.x+rot[4]*body_acc.y+rot[5]*body_acc.z;
	ned_acc.z=rot[6]*body_acc.x+rot[7]*body_acc.y+rot[8]*body_acc.z;
	ned_acc.z -= 9.18;
}


void RotateVelToBody(float body_vel[],float ned_vel_x,float ned_vel_y)
{
	float yaw;
	float sin_yaw;
	float cos_yaw;
	
	yaw=angles[2];
	cos_yaw=arm_cos_f32(yaw);
	sin_yaw=arm_sin_f32(yaw);
	
	body_vel[0]=ned_vel_x*cos_yaw+ned_vel_y*sin_yaw;
	body_vel[1]=ned_vel_x*(-sin_yaw)+ned_vel_y*cos_yaw;
}