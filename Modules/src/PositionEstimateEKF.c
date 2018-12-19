#include "PositionEstimateEKF.h"

float body_vel[3];
float EST_X[3]={0,0,0};//s,v,da
float EST_X_k[3]={0,0,0};
float EST_Y[3]={0,0,0};//s,v,da
float EST_Y_k[3]={0,0,0};

float GPS_Z[2]={0,0};
float GPS_A[3][3] ={{1,0,0},      //A=1 T T^2/2
                    {0,1,0},      //  0 1 T         
                    {0,0,1}};     //  0 0 1
float GPS_K_X[3]={0,0,0};
float GPS_K_Y[3]={0,0,0};

//variance
float GPS_P_X[3][3]={{10,0,0},
                    {0,10,0},
                    {0,0,10}};
//last variance.
float GPS_P_X_k[3][3]={{10,0,0},
                      {0,10,0},
                      {0,0,10}};
//variance
float GPS_P_Y[3][3]={{10,0,0},
                    {0,10,0},
                    {0,0,10}};
//last variance.
float GPS_P_Y_k[3][3]={{10,0,0},
                      {0,10,0},
                      {0,0,10}};
//process noise variance matrix.
float GPS_Q[3][3]={{1,0,0},
									  {0,0.1,0},
										{0,0,1}};

//R:displacement variance
float GPS_R=2;
											
										
void position_predict_state(float ax,float ay,float dt)
{
	//x axis
	EST_X_k[0]=EST_X[0]+EST_X[1]*dt+EST_X[2]*dt*dt/2+ax*dt*dt/2;
	EST_X_k[1]=EST_X[1]+EST_X[2]*dt+ax*dt;
	EST_X_k[2]=EST_X[2];
	//y axis
	EST_Y_k[0]=EST_Y[0]+EST_Y[1]*dt+EST_Y[2]*dt*dt/2+ay*dt*dt/2;
	EST_Y_k[1]=EST_Y[1]+EST_Y[2]*dt+ay*dt;
	EST_Y_k[2]=EST_Y[2];
	
	GPS_A[0][1]=dt;
  GPS_A[0][2]=dt*dt/2;
  GPS_A[1][2]=dt;
}	

void position_predict_covariance(void)
{
	float temp33_1[3][3];
	float temp33_2[3][3];
	//x axis
	//calculate A*P
	matrix_multiply((float*)GPS_A,(float*)GPS_P_X,3,3,3,(float*)temp33_1);
	//transpose A
	matrix_transpose((float*)GPS_A,3,3,(float*)temp33_2);
	// A*P*A^-1
	matrix_multiply((float*)temp33_1,(float*)temp33_2,3,3,3,(float*)GPS_P_X_k);
	//A*P*A^-1+Q
	GPS_P_X_k[0][0]+=GPS_Q[0][0];
	GPS_P_X_k[1][1]+=GPS_Q[1][1];
	GPS_P_X_k[2][2]+=GPS_Q[2][2];
	//y axis
	//calculate A*P
	matrix_multiply((float*)GPS_A,(float*)GPS_P_Y,3,3,3,(float*)temp33_1);
	//transpose A
	matrix_transpose((float*)GPS_A,3,3,(float*)temp33_2);
	// A*P*A^-1
	matrix_multiply((float*)temp33_1,(float*)temp33_2,3,3,3,(float*)GPS_P_Y_k);
	//A*P*A^-1+Q
	GPS_P_Y_k[0][0]+=GPS_Q[0][0];
	GPS_P_Y_k[1][1]+=GPS_Q[1][1];
	GPS_P_Y_k[2][2]+=GPS_Q[2][2];
}

void position_get_kalman_gain(void)
{
	//x axis
	GPS_K_X[0]=GPS_P_X_k[0][0]/(GPS_P_X_k[0][0]+GPS_R);
	GPS_K_X[1]=GPS_P_X_k[1][0]/(GPS_P_X_k[0][0]+GPS_R);
	GPS_K_X[2]=GPS_P_X_k[2][0]/(GPS_P_X_k[0][0]+GPS_R);
	//y axis
	GPS_K_Y[0]=GPS_P_Y_k[0][0]/(GPS_P_Y_k[0][0]+GPS_R);
	GPS_K_Y[1]=GPS_P_Y_k[1][0]/(GPS_P_Y_k[0][0]+GPS_R);
	GPS_K_Y[2]=GPS_P_Y_k[2][0]/(GPS_P_Y_k[0][0]+GPS_R);
}

void position_calculate_estimate(void)
{
	float temp;
	//x axis
	//Z-H*X_k
	temp=GPS_Z[0]-EST_X_k[0];
	//X=X_k+K*(Z-H*X_k);
	EST_X[0]=EST_X_k[0]+GPS_K_X[0]*temp;
	EST_X[1]=EST_X_k[1]+GPS_K_X[1]*temp;
	EST_X[2]=EST_X_k[2]+GPS_K_X[2]*temp;
	//y axis
	//Z-H*X_k
	temp=GPS_Z[1]-EST_Y_k[0];
	//X=X_k+K*(Z-H*X_k);
	EST_Y[0]=EST_Y_k[0]+GPS_K_Y[0]*temp;
	EST_Y[1]=EST_Y_k[1]+GPS_K_Y[1]*temp;
	EST_Y[2]=EST_Y_k[2]+GPS_K_Y[2]*temp;
}
void position_calculate_error_covariance(void)
{
	float temp33[3][3];
	//X axis
	//I-K*H
	temp33[0][0]=1-GPS_K_X[0];
	temp33[0][1]=0;
	temp33[0][2]=0;
	
	temp33[1][0]=-GPS_K_X[1];
	temp33[1][1]=1;
	temp33[1][2]=0;
	
	temp33[2][0]=-GPS_K_X[2];
	temp33[2][1]=0;
	temp33[2][2]=1;
	//(I-K*H)*P
	matrix_multiply((float*)temp33,(float*)GPS_P_X_k,3,3,3,(float*)GPS_P_X);
	
	//Y axis
	//I-K*H
	temp33[0][0]=1-GPS_K_Y[0];
	temp33[0][1]=0;
	temp33[0][2]=0;
	
	temp33[1][0]=-GPS_K_Y[1];
	temp33[1][1]=1;
	temp33[1][2]=0;
	
	temp33[2][0]=-GPS_K_Y[2];
	temp33[2][1]=0;
	temp33[2][2]=1;
	//(I-K*H)*P
	matrix_multiply((float*)temp33,(float*)GPS_P_Y_k,3,3,3,(float*)GPS_P_Y);
	
}

void position_estimate_EKF(float ax,float ay,float pos_x,float pos_y,float dt)
{
	if(gps.update)
	{
		gps.update=0;
		GPS_Z[0]=pos_x;
		GPS_Z[1]=pos_y;
		position_predict_state(ax,ay,dt);
		position_predict_covariance();
		position_get_kalman_gain();
		position_calculate_estimate();
		position_calculate_error_covariance();
	}
	else if(gps.available)
	{
		//x axis
		EST_X[0]=EST_X[0]+EST_X[1]*dt+EST_X[2]*dt*dt/2+ned_acc.x*dt*dt/2;
		EST_X[1]=EST_X[1]+EST_X[2]*dt+ned_acc.x*dt;
		//y axis
		EST_Y[0]=EST_Y[0]+EST_Y[1]*dt+EST_Y[2]*dt*dt/2+ned_acc.y*dt*dt/2;
		EST_Y[1]=EST_Y[1]+EST_Y[2]*dt+ned_acc.y*dt;
	}
}

void position_estimate_reset(void)
{
	memset(&EST_X,0,3);
	memset(&EST_X_k,0,3);
	memset(&EST_Y,0,3);
	memset(&EST_Y_k,0,3);
	memset(&GPS_Z,0,2);
}