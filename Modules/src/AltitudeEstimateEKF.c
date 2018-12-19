#include "AltitudeEstimateEKF.h"
#include "Matrix_math.h"

//current estimated value:s,v,da.
float baro_X_k[3]={0,0,0};
//last estimated value.
float baro_X[3]={0,0,0};

float baro_Z=0;//s

float baro_A[3][3]={{1,0,0},      //A=1 T T^2/2
                    {0,1,0},      //  0 1 T         
                    {0,0,1}};     //  0 0 1
float baro_K[3]={0,0,0};
//variance
float baro_P[3][3]={{1,0,0},
                    {0,1,0},
                    {0,0,1}};
//last variance.
float baro_P_k[3][3]={{1,0,0},
                      {0,1,0},
                      {0,0,1}};

//process noise variance matrix.
float baro_Q[3][3]={{0.001,0,0},
									  {0,0.0001,0},
										{0,0,0.001}};
//observed value variance.
//R:displacement variance
float baro_R=2;
										
float VerVelEst=0;
										
void baro_filter_init(float altitude)
{
  baro_X[0]=altitude;//original altitude,before the aircraft armed.
  baro_X[1]=0;//velocity.
  baro_X[2]=0;//acceleration.
}
void baro_altitude_predict_state(float a,float dt)
{
	baro_X_k[0]=baro_X[0]+baro_X[1]*dt+baro_X[2]*dt*dt/2+a*dt*dt/2;
	baro_X_k[1]=baro_X[1]+baro_X[2]*dt+a*dt;
	baro_X_k[2]=baro_X[2];
	
	baro_A[0][1]=dt;
  baro_A[0][2]=dt*dt/2;
  baro_A[1][2]=dt;
}
void baro_altitude_predict_covariance(void)
{
	float temp33_1[3][3];
	float temp33_2[3][3];
	//calculate A*P
	matrix_multiply((float*)baro_A,(float*)baro_P,3,3,3,(float*)temp33_1);
	//transpose A
	matrix_transpose((float*)baro_A,3,3,(float*)temp33_2);
	// A*P*A^-1
	matrix_multiply((float*)temp33_1,(float*)temp33_2,3,3,3,(float*)baro_P_k);
	//A*P*A^-1+Q
	baro_P_k[0][0]+=baro_Q[0][0];
	baro_P_k[1][1]+=baro_Q[1][1];
	baro_P_k[2][2]+=baro_Q[2][2];
}
void baro_altitude_get_kalman_gain(void)
{
	baro_K[0]=baro_P_k[0][0]/(baro_P_k[0][0]+baro_R);
	baro_K[1]=baro_P_k[1][0]/(baro_P_k[0][0]+baro_R);
	baro_K[2]=baro_P_k[2][0]/(baro_P_k[0][0]+baro_R);
}
void baro_altitude_calculate_estimate(void)
{

	float temp;
	//Z-H*X_k
	temp=baro_Z-baro_X_k[0];
	//X=X_k+K*(Z-H*X_k);
	baro_X[0]=baro_X_k[0]+baro_K[0]*temp;
	baro_X[1]=baro_X_k[1]+baro_K[1]*temp;
	baro_X[2]=baro_X_k[2]+baro_K[2]*temp;
	
}
void baro_altitude_calculate_error_covariance(void)
{
	float temp33[3][3];
	
	//I-K*H
	temp33[0][0]=1-baro_K[0];
	temp33[0][1]=0;
	temp33[0][2]=0;
	
	temp33[1][0]=-baro_K[1];
	temp33[1][1]=1;
	temp33[1][2]=0;
	
	temp33[2][0]=-baro_K[2];
	temp33[2][1]=0;
	temp33[2][2]=1;
	//(I-K*H)*P
	matrix_multiply((float*)temp33,(float*)baro_P_k,3,3,3,(float*)baro_P);
}
float baro_altitude_kalman_filter(float acceleration,float baro_altitude,float dt)
{
	if(baro_update)
	{
		baro_update=0;
		baro_Z=baro_altitude;
		baro_altitude_predict_state(acceleration,dt);
		baro_altitude_predict_covariance();
		baro_altitude_get_kalman_gain();
		baro_altitude_calculate_estimate();
		baro_altitude_calculate_error_covariance();
	}
	else
	{
		baro_X[0]=baro_X[0]+baro_X[1]*dt+baro_X[2]*dt*dt/2+acceleration*dt*dt/2;
		baro_X[1]=baro_X[1]+baro_X[2]*dt+acceleration*dt;
	}
	VerVelEst=baro_X[1];
	return baro_X[0];
}
