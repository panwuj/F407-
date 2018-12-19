#include "Attitude_EKF.h"
#include "math.h"
#include "params.h"

//[q0,q1,q2,q3,Dgx,Dgy,Dgz]' 
float X[7]={1,0,0,0,0,0,0};     //state vector.
float X_k[7]={1,0,0,0,0,0,0};   //preliminary predict state vector.
//[q0,q1,q2,q3]
float Z[4]={1,0,0,0};						//observation vector.

float	P_k[7][7];                  //error covariance matrix.     
float	P[7][7] = {                   
	{10, 0, 0, 0, 0, 0, 0},
	{0, 10, 0, 0, 0, 0, 0},
	{0, 0, 10, 0, 0, 0, 0},
	{0, 0, 0, 10, 0, 0, 0},
	{0, 0, 0, 0, 10, 0, 0},
	{0, 0, 0, 0, 0, 10, 0},
	{0, 0, 0, 0, 0, 0, 10}};

const float	I[7][7] = {
	{1, 0, 0, 0, 0, 0, 0},
	{0, 1, 0, 0, 0, 0, 0},
	{0, 0, 1, 0, 0, 0, 0},
	{0, 0, 0, 1, 0, 0, 0},
	{0, 0, 0, 0, 1, 0, 0},
	{0, 0, 0, 0, 0, 1, 0},
	{0, 0, 0, 0, 0, 0, 1}};

//0.000001
float Q[3][3]={{0.00001,0,0},
							 {0,0.00001,0},
							 {0,0,0.00001}};         //System process noise variance matrix.
//0.000001
float	R[4][4] ={{0.00002, 0, 0, 0},
								{0, 0.00002, 0, 0},
								{0, 0, 0.00002, 0},
								{0, 0, 0, 0.00002}};  //System observation noise variance matrix.
float T[7][3];   //Noise input matrix.
float T_t[3][7]; //Tanspose of noise input matrix.
float	K[7][4];   //Kalman gain matrix.
float	A[7][7];   //System state transition matrix.
float	A_t[7][7]; //Transpose of system state transition matrix.					
float	angles[3]; //angle of roll,pitch,yaw.								
																
/**********************************/

//temporary variable.
float roll_z,pitch_z,yaw_z;
float angles_z[3];

/**********************************/
void GetObservationByGravity(float q[4],float ax,float ay,float az)
{
	float pitch,roll,yaw;
	float roll_d2,pitch_d2;//yaw_d2;
	float norm;
	if(az==0)
		return;
	pitch=asin(-ax);
	roll=atan(ay/az);
	
//	pitch_z=pitch*57.3;
//	roll_z=roll*57.3;
	yaw=0;
	
	roll_d2=roll/2;
	pitch_d2=pitch/2;
	//yaw_d2=yaw/2;	//yaw==yaw_d2==0;
	
//	q[0] = cos(roll_d2)*cos(pitch_d2)*cos(yaw_d2)+sin(roll_d2)*sin(pitch_d2)*sin(yaw_d2);
//	q[1] = sin(roll_d2)*cos(pitch_d2)*cos(yaw_d2)-cos(roll_d2)*sin(pitch_d2)*sin(yaw_d2);
//	q[2] = cos(roll_d2)*sin(pitch_d2)*cos(yaw_d2)+sin(roll_d2)*cos(pitch_d2)*sin(yaw_d2);
//	q[3] = cos(roll_d2)*cos(pitch_d2)*sin(yaw_d2)+sin(roll_d2)*sin(pitch_d2)*cos(yaw_d2);
	
	q[0] = arm_cos_f32(roll_d2)*arm_cos_f32(pitch_d2);
	q[1] = arm_sin_f32(roll_d2)*arm_cos_f32(pitch_d2);
	q[2] = arm_cos_f32(roll_d2)*arm_sin_f32(pitch_d2);
	q[3] = arm_sin_f32(roll_d2)*arm_sin_f32(pitch_d2);
	
	arm_sqrt_f32(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3],&norm);
	q[0]=q[0]/norm;
	q[1]=q[1]/norm;
	q[2]=q[2]/norm;
	q[3]=q[3]/norm;
}

void GetObservation(float q[4],float ax,float ay,float az,float mx,float my,float mz)
{
	float pitch,roll,yaw;
	float roll_d2,pitch_d2,yaw_d2;
	float cos_p,cos_r,cos_y,sin_p,sin_r,sin_y;
	float hx,hy,hz;
	float norm;
	if(az==0)
		return;
	pitch=asin(-ax);
	roll=atan(ay/az);
	
	cos_p=arm_cos_f32(angles[1]);//cos(pitch)
	cos_r=arm_cos_f32(angles[0]);//cos(roll)
	sin_p=arm_sin_f32(angles[1]);//sin(pitch)
	sin_r=arm_sin_f32(angles[0]);//sin(roll);

	hx=cos_p*mx+sin_r*sin_p*my+cos_r*sin_p*mz;
	hy=cos_r*my-sin_r*mz;
	hz=-sin_p*mx+sin_r*cos_p*my+cos_r*cos_p*mz;
	
	yaw=atan2(-hy,hx);
	
	roll_d2=roll/2;
	pitch_d2=pitch/2;
	yaw_d2=yaw/2;	
	
	cos_p=arm_cos_f32(pitch_d2);
	cos_r=arm_cos_f32(roll_d2);
	cos_y=arm_cos_f32(yaw_d2);
	sin_p=arm_sin_f32(pitch_d2);
	sin_r=arm_sin_f32(roll_d2);
	sin_y=arm_sin_f32(yaw_d2);
	
	
	q[0]=cos_r*cos_p*cos_y+sin_r*sin_p*sin_y;
	q[1]=sin_r*cos_p*cos_y-cos_r*sin_p*sin_y;
	q[2]=cos_r*sin_p*cos_y+sin_r*cos_p*sin_y;
	q[3]=cos_r*cos_p*sin_y+sin_r*sin_p*cos_y;
	
	arm_sqrt_f32(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3],&norm);
	q[0]=q[0]/norm;
	q[1]=q[1]/norm;
	q[2]=q[2]/norm;
	q[3]=q[3]/norm;
}

float ex_max=0,ey_max=0,ez_max=0;
void GetObservationByMahony(float Z[4],float ax,float ay,float az,float mx,float my,float mz)
{
	static float Kp=2.0f;
	static float Ki=0.005f;
	float gx=0,gy=0,gz=0;
	float exInt=0,eyInt=0,ezInt=0;
	float q[4];
	float rot[9];
	float norm;
	float halfT=0.0001f;
	float	hx, hy, hz, bx, bz;
  float vx, vy, vz;
	float wx, wy, wz;
  float ex, ey, ez;
	uint8_t i;
	
	q[0]=X[0];
	q[1]=X[1];
	q[2]=X[2];
	q[3]=X[3];
	
	for(i=0;i<6;i++)
	{
		GetRotationMatrix(q,rot);
		vx=rot[6];
		vy=rot[7];
		vz=rot[8];
		
		ex = (ay*vz - az*vy) ;   //向量外积在相减得到差分就是误差                        					 
		ey = (az*vx - ax*vz) ;
		ez = (ax*vy - ay*vx) ;
		
		if(fabs(ex_max)<fabs(ex))ex_max=ex;
		if(fabs(ey_max)<fabs(ey))ey_max=ey;
		if(fabs(ez_max)<fabs(ez))ez_max=ez;
		
		if(fabs(ex)<0.005&&fabs(ey)<0.005&&fabs(ez)<0.005)
		{
			break;
		}
		exInt = exInt + ex * Ki;	//对误差进行积分
		eyInt = eyInt + ey * Ki;
		ezInt = ezInt + ez * Ki;
		
		gx = gx + Kp*ex + exInt;					   				
		gy = gy + Kp*ey + eyInt;
		gz = gz + Kp*ez + ezInt;
		
		q[0] = q[0] + (-q[1]*gx - q[2]*gy - q[3]*gz)*halfT;
		q[1] = q[1] + (q[0]*gx + q[2]*gz - q[3]*gy)*halfT;
		q[2] = q[2] + (q[0]*gy - q[1]*gz + q[3]*gx)*halfT;
		q[3] = q[3] + (q[0]*gz + q[1]*gy - q[2]*gx)*halfT;

		// normalise quaternion
		arm_sqrt_f32(q[0]*q[0]+ q[1]*q[1] + q[2]*q[2] + q[3]*q[3],&norm);
		q[0] = q[0] / norm;
		q[1] = q[1] / norm;
		q[2] = q[2] / norm;
		q[3] = q[3] / norm;
	}
	if(fabs(ex)<0.1||fabs(ey)<0.1||fabs(ez)<0.1)
	{
		
	}
	Z[0]=q[0];
	Z[1]=q[1];
	Z[2]=q[2];
	Z[3]=q[3];
}
void GetMagneticVectorNED(float mx,float my,float mz,float *bx,float *by,float *bz)
{
	float roll,pitch;
	float sin_r;
	float cos_r;
	float sin_p;
	float cos_p;
	float norm;
	roll =angles[0];
	pitch=angles[1];
	sin_r=sin(roll);
	sin_p=sin(pitch);
	cos_r=cos(roll);
	cos_p=cos(pitch);
	
	*bx=cos_p*mx+sin_r*sin_p*my+cos_r*sin_p*mz;
	*by=cos_r*my-sin_r*mz;
	*bz=-sin_p*mx+sin_r*cos_p*my+cos_r*cos_p*mz;
	
	*bx=sqrt((*bx)*(*bx)+(*by)*(*by));
	*by=0;
	norm=sqrt((*bx)*(*bx)+(*bz)*(*bz));
	*bx=*bx/norm;
	*bz=*bz/norm;
}	

void GetMagneticVectorNED_2(float mx,float my,float mz,float *bx,float *by,float *bz)
{
	float v_b[3];
	float v_n[3];
	float temp;
	v_b[0]=mx;
	v_b[1]=my;
	v_b[2]=mz;
	
	RotateVectorToNED(v_n,v_b);
	
	arm_sqrt_f32(v_n[0]*v_n[0]+v_n[1]*v_n[1],&temp);
	*bx=temp;
	*by=0;
	*bz=v_n[2];
}


void GaussNewtonGetJacobian(float q[4], float ax, float ay, float az, float mx, float my, float mz, float J[6][4])
{
	float q0=q[0];
	float q1=q[1];
	float q2=q[2];
	float q3=q[3];
	J[0][0]=-(2*ax*q0 - 2*ay*q3 + 2*az*q2);
	J[0][1]=-(2*ax*q1 + 2*ay*q2 + 2*az*q3);
	J[0][2]=-(2*ay*q1 - 2*ax*q2 + 2*az*q0);
	J[0][3]=-(2*az*q1 - 2*ay*q0 - 2*ax*q3);
	
	J[1][0]=-(2*ax*q3 + 2*ay*q0 - 2*az*q1);
	J[1][1]=-(2*ax*q2 - 2*ay*q1 - 2*az*q0);
	J[1][2]=-(2*ax*q1 + 2*ay*q2 + 2*az*q3);
	J[1][3]=-(2*ax*q0 - 2*ay*q3 + 2*az*q2);
	
	J[2][0]=-(2*ay*q1 - 2*ax*q2 + 2*az*q0);
	J[2][1]=-(2*ax*q3 + 2*ay*q0 - 2*az*q1);
	J[2][2]=-(2*ay*q3 - 2*ax*q0 - 2*az*q2);
	J[2][3]=-(2*ax*q1 + 2*ay*q2 + 2*az*q3);
	
	J[3][0]=-(2*mx*q0 - 2*my*q3 + 2*mz*q2);
	J[3][1]=-(2*mx*q1 + 2*my*q2 + 2*mz*q3);
	J[3][2]=-(2*my*q1 - 2*mx*q2 + 2*mz*q0);
	J[3][3]=-(2*mz*q1 - 2*my*q0 - 2*mx*q3);
	
	J[4][0]=-(2*mx*q3 + 2*my*q0 - 2*mz*q1);
	J[4][1]=-(2*mx*q2 - 2*my*q1 - 2*mz*q0);
	J[4][2]=-(2*mx*q1 + 2*my*q2 + 2*mz*q3);
	J[4][3]=-(2*mx*q0 - 2*my*q3 + 2*mz*q2);
	
	J[5][0]=-(2*my*q1 - 2*mx*q2 + 2*mz*q0);
	J[5][1]=-(2*mx*q3 + 2*my*q0 - 2*mz*q1);
	J[5][2]=-(2*my*q3 - 2*mx*q0 - 2*mz*q2);
	J[5][3]=-(2*mx*q1 + 2*my*q2 + 2*mz*q3);

}								
/*
*@param:v_b[6] the measurement vector of body frame.
*@param:v_n[6] the measurement vector of NED  frame.
*/
void GetMeasurementVectorNED(float q[4],float v_b[6],float v_n[6])								
{
	float q0q0=q[0]*q[0];
	float q0q1=q[0]*q[1];
	float q0q2=q[0]*q[2];
	float q0q3=q[0]*q[3];
	float q1q1=q[1]*q[1];
	float q1q2=q[1]*q[2];
	float q1q3=q[1]*q[3];
	float q2q2=q[2]*q[2];
	float q2q3=q[2]*q[3];
	float q3q3=q[3]*q[3];
	float norm;
	//acceleration
	v_n[0]=(q0q0+q1q1-q2q2-q3q3)*v_b[0]+2*(q1q2-q0q3)*v_b[1]+2*(q1q3+q0q2)*v_b[2];
	v_n[1]=2*(q1q2+q0q3)*v_b[0]+(q0q0-q1q1+q2q2-q3q3)*v_b[1]+2*(q2q3-q0q1)*v_b[2];
	v_n[2]=2*(q1q3-q0q2)*v_b[0]+2*(q2q3+q0q1)*v_b[1]+(q0q0-q1q1-q2q2+q3q3)*v_b[2];
	//magnetism
	v_n[3]=(q0q0+q1q1-q2q2-q3q3)*v_b[3]+2*(q1q2-q0q3)*v_b[4]+2*(q1q3+q0q2)*v_b[5];
	v_n[4]=2*(q1q2+q0q3)*v_b[3]+(q0q0-q1q1+q2q2-q3q3)*v_b[4]+2*(q2q3-q0q1)*v_b[5];
	v_n[5]=2*(q1q3-q0q2)*v_b[3]+2*(q2q3+q0q1)*v_b[4]+(q0q0-q1q1-q2q2+q3q3)*v_b[5];
	arm_sqrt_f32(v_n[3]*v_n[3]+v_n[4]*v_n[4],&norm);
	v_n[3] /=norm;
	v_n[4] /=norm;
	v_n[5] =0;
}

void QuaternionToEuler(float q[4],float *roll,float *pitch,float *yaw)
{
	*roll  = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1); // roll
	*pitch = asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]); // pitch
	*yaw   = atan2(2*(q[1]*q[2]+q[0]*q[3]),q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3]);
	*roll*=57.3;
	*pitch*=57.3;
	*yaw*=57.3;
}
uint8_t ErrorIsSmall(float v_err[6])
{
	uint8_t flag[6]={0,0,0,0,0,0};
	uint8_t i;
	for(i=0;i<6;i++)
	{
		if(fabs(v_err[i])<0.01)
			flag[i]=1;
	}
	if(flag[0]&&flag[1]&&flag[2]&&flag[3]&&flag[4]&&flag[5])
		return 1;
	else
		return 0;
}

void GaussNewtonEstimator(float ax,float ay,float az,float mx,float my,float mz)
{
	float q[4]={1,0,0,0};						 //observation quaternion
	float v_b[6];								     //measurement vector in body frame.
	float v_n[6];								     //measurement vector in NED  frame.
	float v_err[6];							     //the error of v_n and v_b.
	float V_NED[6]={0,0,1,1,0,0};    //the real gravity vector and magnetic vector in NED frame. 
	float norm;
	static float J[6][4];						 //Jacobian Matrix.
	static float J_t[4][6];					 //transpose of Jacobian Matrix.
	static float temp44_1[4][4];		 //temporary variable,matrix for 4x4.
	static float temp44_2[4][4];		 //temporary variable,matrix for 4x4.
	static float temp46[4][6];			 //temporary variable,matrix for 4x6.
	static float temp41[4];					 //temporary variable,matrix for 4x1.
	unsigned char i;
					
//	GetMagneticVectorNED(mx,my,mz,&V_NED[3],&V_NED[4],&V_NED[5]);
	q[0]=X[0];q[1]=X[1];q[2]=X[2];q[3]=X[3];
	v_b[0]=ax;
	v_b[1]=ay;
	v_b[2]=az;
	v_b[3]=mx;
	v_b[4]=my;
	v_b[5]=mz;
	
	for(i=0;i<6;i++)
	{
		GaussNewtonGetJacobian(q,ax,ay,az,mx,my,mz,J);
		GetMeasurementVectorNED(q,v_b,v_n);
		//v_err=V_NED-v_n.
		matrix_subtraction(V_NED,v_n,6,1,v_err);
		
		if(ErrorIsSmall(v_err))
		{
			if(i!=0)
				break;
		}
		
		//get J_t.
		matrix_transpose((float*)J,6,4,(float*)J_t);
		//calculate J_t*J.
		matrix_multiply((float*)J_t,(float*)J,4,6,4,(float*)temp44_1);
		//calculate inversion of J_t*J,——>(J_t*J)^-1.
		matrix_inversion((float*)temp44_1,4,(float*)temp44_2);
		//calculate ((J_t*J)^-1)*J_t
		matrix_multiply((float*)temp44_2,(float*)J_t,4,4,6,(float*)temp46);
		//calculate ((J_t*J)^-1)*J_t*v_err
		matrix_multiply((float*)temp46,v_err,4,6,1,temp41);
		//calculate q(t)=q(t-1)-((J_t*J)^-1)*J_t*v_err;
		matrix_subtraction(q,temp41,4,1,q);
		
		norm=sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
		q[0]=q[0]/norm;
		q[1]=q[1]/norm;
		q[2]=q[2]/norm;
		q[3]=q[3]/norm;
	}

	Z[0]=q[0];
	Z[1]=q[1];
	Z[2]=q[2];
	Z[3]=q[3];
//	//	matrix_copy(q,4,1,Z);
//	angles_z[0] = atan2(2 * Z[2] * Z[3] + 2 * Z[0] * Z[1], -2 * Z[1] * Z[1] - 2 * Z[2] * Z[2] + 1); // roll
//	angles_z[1] = asin(-2 * Z[1] * Z[3] + 2 * Z[0] * Z[2]); // pitch
//	angles_z[2] = atan2(2*(Z[1]*Z[2]+Z[0]*Z[3]),Z[0]*Z[0]+Z[1]*Z[1]-Z[2]*Z[2]-Z[3]*Z[3]);

//	roll_z=angles_z[0]*57.3;
//	pitch_z=angles_z[1]*57.3;
//	yaw_z=angles_z[2]*57.3;
}			
void GetStateNoiseMatrixEKF(float q[4],float gx,float gy,float gz,float dt)
{
	int i,j;
	float HalfTgx=gx*dt*0.5;
	float HalfTgy=gy*dt*0.5;
	float HalfTgz=gz*dt*0.5;
	
	float HalfTq0=q[0]*dt*0.5;
	float HalfTq1=q[1]*dt*0.5;
	float HalfTq2=q[2]*dt*0.5;
	float HalfTq3=q[3]*dt*0.5;
	
	A[0][0]= 1;
	A[0][1]=-HalfTgx;
	A[0][2]=-HalfTgy;
	A[0][3]=-HalfTgz;
	A[0][4]=-HalfTq1;
	A[0][5]=-HalfTq2;
	A[0][6]=-HalfTq3;
	
	A[1][0]= HalfTgx;
	A[1][1]= 1;
	A[1][2]= HalfTgz;
	A[1][3]=-HalfTgy;
	A[1][4]= HalfTq0;
	A[1][5]=-HalfTq3;
	A[1][6]= HalfTq2;

	A[2][0]= HalfTgy;
	A[2][1]=-HalfTgz;
	A[2][2]= 1;
	A[2][3]= HalfTgx;
	A[2][4]= HalfTq3;
	A[2][5]= HalfTq0;
	A[2][6]=-HalfTq1;
	
	A[3][0]= HalfTgz;
	A[3][1]= HalfTgy;
	A[3][2]=-HalfTgx;
	A[3][3]= 1;
	A[3][4]=-HalfTq2;
	A[3][5]= HalfTq1;
	A[3][6]= HalfTq0;
	
	for(i=4;i<7;i++)
	{
		for(j=0;j<7;j++)
		{
			if(i==j)
				A[i][j]=1;
			else
				A[i][j]=0;
		}
	}
	//get noise input matrix T
	for(i=0;i<7;i++)
	{
		for(j=0;j<3;j++)
		{
			if(i<4)
				T[i][j]=A[i][j+4];
			else
				T[i][j]=0;
		}
	}
}	
void PredictStateEKF(void)
{
	X_k[0] = A[0][0]*X[0] + A[0][1]*X[1] + A[0][2]*X[2] + A[0][3]*X[3] + A[0][4]*X[4] + A[0][5]*X[5] + A[0][6]*X[6];
  X_k[1] = A[1][0]*X[0] + A[1][1]*X[1] + A[1][2]*X[2] + A[1][3]*X[3] + A[1][4]*X[4] + A[1][5]*X[5] + A[1][6]*X[6];
  X_k[2] = A[2][0]*X[0] + A[2][1]*X[1] + A[2][2]*X[2] + A[2][3]*X[3] + A[2][4]*X[4] + A[2][5]*X[5] + A[2][6]*X[6];
  X_k[3] = A[3][0]*X[0] + A[3][1]*X[1] + A[3][2]*X[2] + A[3][3]*X[3] + A[3][4]*X[4] + A[3][5]*X[5] + A[3][6]*X[6];
  X_k[4] = X[4];
  X_k[5] = X[5];
  X_k[6] = X[6];
}

void PredictErrorCovarianceEKF(void)
{
	float temp77[7][7];
	float temp73[7][3];
	//get A_t
	matrix_transpose((float*)A,7,7,(float*)A_t);
	//get T_t
	matrix_transpose((float*)T,7,3,(float*)T_t);
	//calculate A*P(k-1)
	matrix_multiply((float*)A,(float*)P,7,7,7,(float*)temp77);
	//calculate A*P(k-1)*A_t
	matrix_multiply((float*)temp77,(float*)A_t,7,7,7,(float*)P_k);
	
	//calculate T*Q
	matrix_multiply((float*)T,(float*)Q,7,3,3,(float*)temp73);
	//calculate T*Q*T_t
	matrix_multiply((float*)temp73,(float*)T_t,7,3,7,(float*)temp77);
	//calculate P_k=A*P(k-1)*A_t+T*Q*T_t.
	matrix_addition((float*)P_k,(float*)temp77,7,7,(float*)P_k);
}

void GetKalmanGainEKF(void)
{
	float temp44_1[4][4];
	float temp44_2[4][4];
	float temp74[7][4];
	int i,j;
	//calculate H*P_k*H_t+R
	temp44_1[0][0] = P_k[0][0] + R[0][0];
  temp44_1[0][1] = P_k[0][1];
  temp44_1[0][2] = P_k[0][2];
  temp44_1[0][3] = P_k[0][3];
    
  temp44_1[1][0] = P_k[1][0];
  temp44_1[1][1] = P_k[1][1] + R[1][1];
  temp44_1[1][2] = P_k[1][2];
  temp44_1[1][3] = P_k[1][3];

  temp44_1[2][0] = P_k[2][0];
  temp44_1[2][1] = P_k[2][1];
  temp44_1[2][2] = P_k[2][2] + R[2][2];
  temp44_1[2][3] = P_k[2][3];
    
  temp44_1[3][0] = P_k[3][0];
  temp44_1[3][1] = P_k[3][1];
  temp44_1[3][2] = P_k[3][2];
  temp44_1[3][3] = P_k[3][3] + R[3][3];
	
	//calculate (H*P_k*H_t+R)^-1
	matrix_inversion((float*)temp44_1,4,(float*)temp44_2);
	//calculate P_k*H_t
	for (i=0; i<7; i++)
	{
    for(j=0; j<4; j++) 
			temp74[i][j] = P_k[i][j];
	}
	//K = P_k*H_T*(H*P_k*H_T + R)^-1
	matrix_multiply((float*)temp74,(float*)temp44_2,7,4,4,(float*)K);
//  K[0][1]=K[0][2]=K[0][3]=0;
//	K[1][0]=K[1][2]=K[1][3]=0;
//	K[2][0]=K[2][1]=K[2][3]=0;
//	K[3][0]=K[3][1]=K[3][2]=0;
}
void CalculateEstimateEKF(void)
{
	float	temp41_1[4];
	float	temp41_2[4];
	float	temp71[7];
	
	//calculate H*X_k
	temp41_1[0] = X_k[0];
  temp41_1[1] = X_k[1];
  temp41_1[2] = X_k[2];
  temp41_1[3] = X_k[3];
  //calculate Z - H*X_k
	matrix_subtraction((float*)Z, (float*)temp41_1, 4, 1, (float*)temp41_2);
  //calculate K*(Z - H*X_k)
  matrix_multiply((float*)K, (float*)temp41_2, 7, 4, 1, (float*)temp71);
	//calculate X = X_k + K*(Z - H*X_k)
  matrix_addition((float*)X_k, (float*)temp71, 7, 1, X);
}

void CalculateErrorCovariance(void)
{
	float	temp77_1[7][7];
	float	temp77_2[7][7];
	int i,j;
	//calculate K*H
	for(i=0;i<7;i++)
	{
		for(j=0;j<7;j++)
		{
			if(j<4)
				temp77_1[i][j]=K[i][j];
			else
				temp77_1[i][j]=0;
		}
	}
	//calculate (I - K*H)
	matrix_subtraction((float*)I, (float*)temp77_1, 7, 7, (float*)temp77_2);
	//calculate P=(I - K*H)*P_k
	matrix_multiply((float*)temp77_2, (float*)P_k, 7, 7, 7, (float*)P);
}
void AttitudeEKF(float gx,float gy,float gz,float ax,float ay,float az,float mx,float my,float mz,float dt)
{
	float norm;
	
	if(ax==0&&ay==0&&az==0)
 		return;
	if(mx==0&&my==0&&mz==0)
 		return;
	arm_sqrt_f32(ax*ax+ay*ay+az*az,&norm);
	ax = ax/norm;
	ay = ay/norm;
	az = az/norm;
	
	arm_sqrt_f32(mx*mx+my*my+mz*mz,&norm);
	mx = mx/norm;
	my = my/norm;
	mz = mz/norm;
	//get Z
	GaussNewtonEstimator(ax,ay,az,mx,my,mz);
//	GetObservationByGravity(Z,ax,ay,az);
//	GetObservation(Z,ax,ay,az,mx,my,mz);
//	GetObservationByMahony(Z,ax,ay,az,mx,my,mz);
	
	//get state matrix A and noise input matrix T
	GetStateNoiseMatrixEKF(X,gx,gy,gz,dt);
	//predict state: X_k=A*X
	PredictStateEKF();//for compute faster.
	//matrix_multiply((float*)A,X,7,7,1,X_k);
	
	//predict error covariance: P_k=A*P(k-1)*A_t+T*Q*T_t.
	PredictErrorCovarianceEKF();
	//calculate kalman gain: K = P_k*H_T*(H*P_k*H_T + R)^-1
	GetKalmanGainEKF();
	//calculate the estimate: X = X_k + K*(Z - H*X_k)
	CalculateEstimateEKF();
	//calculate the error covariance:P=(I-K*H)P_k 
	CalculateErrorCovariance();

	// normalise quaternion
	arm_sqrt_f32(X[0]*X[0]+X[1]*X[1]+X[2]*X[2]+X[3]*X[3],&norm);
	X[0] = X[0]/norm;
	X[1] = X[1]/norm;
	X[2] = X[2]/norm;
	X[3] = X[3]/norm;

	q[0]=X[0];
	q[1]=X[1];
	q[2]=X[2];
	q[3]=X[3];
	
	GetRotationMatrix(q,rot);
	
	angles[0]=atan2(2*(X[2]*X[3]+X[0]*X[1]),1-2*X[1]*X[1]-2*X[2]*X[2]);
	angles[1]=asin(-2*(X[1]*X[3]-X[0]*X[2]));
	angles[2]=atan2(2*(X[1]*X[2]+X[0]*X[3]),1-2*X[2]*X[2]-2*X[3]*X[3]);
	
	Roll  = angles[0]* 57.3;
	Pitch = angles[1]* 57.3;
	Yaw   = angles[2]* 57.3;

	global_params.AttitudeActual.roll=Roll;
	global_params.AttitudeActual.pitch=Pitch;
	global_params.AttitudeActual.yaw=Yaw;
}														

void AttitudeEKF_Init(float ax,float ay,float az,float mx,float my,float mz)
{
//	float q[4];
	float norm;
	
	arm_sqrt_f32(ax*ax+ay*ay+az*az,&norm);
	ax = ax/norm;
	ay = ay/norm;
	az = az/norm;
	
	arm_sqrt_f32(mx*mx+my*my+mz*mz,&norm);
	mx = mx/norm;
	my = my/norm;
	mz = mz/norm;
//	GetObservationByGravity(q,ax,ay,az);
	GetObservation(q,ax,ay,az,mx,my,mz);
//	GetRotationMatrix(q,rot);
	X[0]=q[0];
	X[1]=q[1];
	X[2]=q[2];
	X[3]=q[3];  
	X[4]=0;
	X[5]=0;
	X[6]=0;
}								
								
								
								
								
								
								
								
								
								
								
								
								
								
								