#include "pixy.h"

Pixy_Color Pixy_Color_Inf,Pixy_Color_Last;

float pixy_X[2]={0,0};//v,da
float pixy_Y[2]={0,0};//v,da

float pixy_X_K[2]={0,0};//v,da
float pixy_Y_K[2]={0,0};//v,da

float pixy_Z[2];

float pixy_A[2][2]={{1,0},
										{0,1}};
float pixy_K_X[2]={0,0};
float pixy_K_Y[2]={0,0};
										
float pixy_P_X[2][2]={{4,0},
											{0,4}};

float pixy_P_X_K[2][2]={{4,0},
											  {0,4}};
float pixy_P_Y[2][2]={{4,0},
											{0,4}};
float pixy_P_Y_K[2][2]={{4,0},
											  {0,4}};
float pixy_Q[2][2]={{0.05,0},
										{0,0.5}};
float pixy_R=5;
										
u16 pixy_PosX,pixy_PosY;
u16 pixy_PosX_last,pixy_PosY_last;

float p_velocity_x=0,p_velocity_y=0;
float pixy_v_est_x=0,pixy_v_est_y=0;										
u8 pixy_update=0;		
u8 pixy_available=0;										
u8 pixy_count=0;
u8 pix_first_run=1;
										
#define c_x  157.7585
#define c_y  100.7620
										
void Pixy_Analysis(u8 *buf)
{
	static uint32_t pixy_time_now=0,pixy_time_last=0;
	static u8 first_run=1;
	float dt;
	u32 r_r;
	float r;
	float fac;

	if(buf[2]!=0x55&&buf[3]!=0xaa&&buf[4]!=0x55&&buf[5]!=0xaa)
	{
		pixy_update=0;
		pixy_available=0;
		pixy_count=0;
		return;
	}
	get_dt_in_seconds(&pixy_time_now,&pixy_time_last,&dt);

//	Pixy_Color_Inf.Pixy_Color_Sig    = (int16_t)(buf[7]<<8)|buf[6];
//	Pixy_Color_Inf.Pixy_Color_PosX   = (int16_t)(buf[9]<<8)|buf[8];
//	Pixy_Color_Inf.Pixy_Color_PosY   = (int16_t)(buf[11]<<8)|buf[10];
//	Pixy_Color_Inf.Pixy_Color_Width  = (int16_t)(buf[13]<<8)|buf[12];
//	Pixy_Color_Inf.Pixy_Color_Height = (int16_t)(buf[15]<<8)|buf[14];
	
//	pixy_PosX = (int16_t)(buf[11]<<8)|buf[10];
//	pixy_PosY = (int16_t)(buf[13]<<8)|buf[12];
	
	pixy_PosY = (int16_t)(buf[11]<<8)|buf[10];//X->Y
	pixy_PosX = (int16_t)(buf[13]<<8)|buf[12];
	
	if(first_run)
	{
		first_run=0;
		dt=0.01;
		p_velocity_x=p_velocity_y=0;
		pixy_PosX_last=pixy_PosX;
		pixy_PosY_last=pixy_PosY;
		return;
	}
	
//	r_r=pixy_PosX*pixy_PosX+pixy_PosY*pixy_PosY;
//	arm_sqrt_f32(r_r,&r);
//	fac= (0.0418 *r_r + 6.2375*r + 9885.9 )/10000;
//	
//	pixy_PosX = (pixy_PosX-c_x)*fac+c_x;
//	pixy_PosY = (pixy_PosY-c_y)*fac+c_y;
	
	p_velocity_x=-(pixy_PosX-pixy_PosX_last)*0.000005*KS103_height/dt;
	p_velocity_y=-(pixy_PosY-pixy_PosY_last)*0.000005*KS103_height/dt;


	pixy_PosX_last=pixy_PosX;
	pixy_PosY_last=pixy_PosY;
	
	
	pixy_update=1;
	pixy_count++;
}

void velocity_predict_state(float ax,float ay,float dt)
{
	pixy_X_K[0]=pixy_X[0]+pixy_X[1]*dt+ax*dt;
	pixy_X_K[1]=pixy_X[1];
	
	pixy_Y_K[0]=pixy_Y[0]+pixy_Y[1]*dt+ay*dt;
	pixy_Y_K[1]=pixy_Y[1];
	
	pixy_A[0][1]=dt;
}

void velocity_predict_covariance(void)
{
	float temp22_1[2][2];
	float temp22_2[2][2];
	//x axis
	//calculate A*P
	matrix_multiply((float*)pixy_A,(float*)pixy_P_X,2,2,2,(float*)temp22_1);
	//transpose A
	matrix_transpose((float*)pixy_A,2,2,(float*)temp22_2);
	// A*P*A^-1
	matrix_multiply((float*)temp22_1,(float*)temp22_2,2,2,2,(float*)pixy_P_X_K);
	//A*P*A^-1+Q
	pixy_P_X_K[0][0]+=pixy_Q[0][0];
	pixy_P_X_K[1][1]+=pixy_Q[1][1];
	//y axis
	//calculate A*P
	matrix_multiply((float*)pixy_A,(float*)pixy_P_Y,2,2,2,(float*)temp22_1);
	//transpose A
	matrix_transpose((float*)pixy_A,2,2,(float*)temp22_2);
	// A*P*A^-1
	matrix_multiply((float*)temp22_1,(float*)temp22_2,2,2,2,(float*)pixy_P_Y_K);
	//A*P*A^-1+Q
	pixy_P_Y_K[0][0]+=pixy_Q[0][0];
	pixy_P_Y_K[1][1]+=pixy_Q[1][1];
}

void velocity_get_kalman_gain(void)
{
	//x axis
	pixy_K_X[0]=pixy_P_X_K[0][0]/(pixy_P_X_K[0][0]+pixy_R);
	pixy_K_X[1]=pixy_P_X_K[1][0]/(pixy_P_X_K[0][0]+pixy_R);
	//y axis
	pixy_K_Y[0]=pixy_P_Y_K[0][0]/(pixy_P_Y_K[0][0]+pixy_R);
	pixy_K_Y[1]=pixy_P_Y_K[1][0]/(pixy_P_Y_K[0][0]+pixy_R);

}

void velocity_calculate_estimate(void)
{
	float temp;
	//x axis
	//Z-H*X_k
	temp=pixy_Z[0]-pixy_X_K[0];
	//X=X_k+K*(Z-H*X_k);
	pixy_X[0]=pixy_X_K[0]+pixy_K_X[0]*temp;
	pixy_X[1]=pixy_X_K[1]+pixy_K_X[1]*temp;
	//y axis
	//Z-H*X_k
	temp=pixy_Z[1]-pixy_Y_K[0];
	//X=X_k+K*(Z-H*X_k);
	pixy_Y[0]=pixy_Y_K[0]+pixy_K_Y[0]*temp;
	pixy_Y[1]=pixy_Y_K[1]+pixy_K_Y[1]*temp;
}

void velocity_calculate_error_covariance(void)
{
	float temp22[2][2];
	//X axis
	//I-K*H
	temp22[0][0]=1-pixy_K_X[0];
	temp22[0][1]=0;
	
	temp22[1][0]=-pixy_K_X[1];
	temp22[1][1]=1;

	//(I-K*H)*P
	matrix_multiply((float*)temp22,(float*)pixy_P_X_K,2,2,2,(float*)pixy_P_X);
	
	//Y axis
	//I-K*H
	temp22[0][0]=1-pixy_K_Y[0];
	temp22[0][1]=0;
	
	temp22[1][0]=-pixy_K_Y[1];
	temp22[1][1]=1;
	//(I-K*H)*P
	matrix_multiply((float*)temp22,(float*)pixy_P_Y_K,2,2,2,(float*)pixy_P_Y);
	
}

void pixy_Velocity_estimateEKF(float ax,float ay,float dt)
{
	static u8 restart=1;
	if(restart)
	{
		if(pixy_available)
		{
			pixy_X[0]=pixy_Y[0]=0;//reset
			restart=0;
		}
		else
			return;
	}
	if(!pixy_available)
	{
		restart=1;
		return;
	}
	if(pixy_update)
	{
		pixy_update=0;
		pixy_Z[0]=p_velocity_x;
		pixy_Z[1]=p_velocity_y;
		velocity_predict_state(ax,ay,dt);
		velocity_predict_covariance();
		velocity_get_kalman_gain();
		velocity_calculate_estimate();
		velocity_calculate_error_covariance();
	}
	else if(pixy_available)
	{
		pixy_X[0]=pixy_X[0]+pixy_X[1]*dt+ax*dt;
		pixy_Y[0]=pixy_Y[0]+pixy_Y[1]*dt+ay*dt;
	}
	pixy_v_est_x=pixy_X[0];
	pixy_v_est_y=pixy_Y[0];
	
}

void pixy_Update(void)
{
	if(usart2.update)
	{
		usart2.update=0;
		if(usart2.rx_length<12)
		{
			pixy_available=0;
			pixy_count=0;
			pix_first_run=1;
			return;
		}
		Pixy_Analysis(usart2.rx_buf);
	}
	if(pixy_count>5)
	{
		pixy_count=5;
		pixy_available=1;
	}
}

void velociyt_est_reset(void)
{
	memset(&pixy_X,0,2);
	memset(&pixy_X_K,0,2);
	
	memset(&pixy_Y,0,2);
	memset(&pixy_Y_K,0,2);
	
//	pixy_P_X[0][0]=pixy_P_X_K[0][0]=4;
//	pixy_P_X[0][1]=pixy_P_X_K[0][1]=0;
//	pixy_P_X[1][0]=pixy_P_X_K[1][0]=0;
//	pixy_P_X[1][1]=pixy_P_X_K[1][1]=4;
//	
//	pixy_P_Y[0][0]=pixy_P_Y_K[0][0]=4;
//	pixy_P_Y[0][1]=pixy_P_Y_K[0][1]=0;
//	pixy_P_Y[1][0]=pixy_P_Y_K[1][0]=0;
//	pixy_P_Y[1][1]=pixy_P_Y_K[1][1]=4;
//	
	pixy_available=0;
	pidVel_X.error=pidVel_X.prevError=0;
	pidVel_Y.error=pidVel_Y.prevError=0;
}
