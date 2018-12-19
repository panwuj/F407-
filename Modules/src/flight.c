#include "flight.h"

_att_info_ att_info[5];

void get_att_info(int i)
{	
	if(i>=5)
		return;
	
	att_info[i].q0=q[0];
	att_info[i].q1=q[1];
	att_info[i].q2=q[2];
	att_info[i].q3=q[3];
	att_info[i].dt=imu_update_dt;
	att_info[i].roll=Roll;
	att_info[i].pitch=Pitch;
	att_info[i].yaw=Yaw;
	att_info[i].mpu6050[0]=mpu6050.ax;
	att_info[i].mpu6050[1]=mpu6050.ay;
	att_info[i].mpu6050[2]=mpu6050.az;
	att_info[i].mpu6050[3]=mpu6050.gx;
	att_info[i].mpu6050[4]=mpu6050.gy;
	att_info[i].mpu6050[5]=mpu6050.gz;
	
	att_info[i].hmc5883l[0]=hmc5883l.x;
	att_info[i].hmc5883l[1]=hmc5883l.y;
	att_info[i].hmc5883l[2]=hmc5883l.z;
}

void FlightCtrlSysInit(void)
{
	int i;
	u32 time_now=0,time_last=0;
	float dt;
	int count=0;
	alt_ground_set=0;
	for(i=0;i<20;i++)
	{
		MPU6050_Get_Motion6();
		HMC5883L_Get_Magnetism();
		if(i==20)
		{ 
			if(mpu6050.ax==0&&mpu6050.ay==0&&mpu6050.az==0)
				i=0;
			if(hmc5883l.x==0&&hmc5883l.y==0&&hmc5883l.z==0)
				i=0;
		}
	}
	AttitudeEKF_Init(mpu6050.ax,mpu6050.ay,mpu6050.az,hmc5883l.y,-hmc5883l.x,hmc5883l.z);
	delay_ms(10);
	gyro_filter_high=0.5;
	for(i=0;i<2500;i++)
	{
		time_last=micros();
		GetAttitude();
		//get_att_info(i);
		if(count==5)
		{
			count=0;
			altitude_baro=MS5611_get_height();
			if(i>50)
				get_alt_baro_ground(altitude_baro);
		}
		count++;
		time_now=micros();
		calculate_dt_in_mills(time_now,time_last,&dt);
		if(dt<2)
			delay_ms(2-dt);
	}
	gyro_filter_high=0.5;
	global_params.AttitudeDesired.yaw=Yaw;
	global_params.AltitudeDesired=0;
	alt_ground_set=1;
	altitude_baro=0;
	baro_update=0;
	Location_Init();
	delay_ms(2);
}


void flight_task_1(void)
{
	
}

void flight_task_2(void)
{
	u8 buf[1];
	while(true)
	{
		if(pixy_available)
		{
			buf[0]=0x22;
			USARTSendDatasByDMA(UART4,buf,1);
			GPIO_ResetBits(GPIOB,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5);  
		}
		else
		{
			buf[0]=0x11;
			USARTSendDatasByDMA(UART4,buf,1);
			GPIO_SetBits(GPIOB,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5);  
		}
		vTaskDelay(200);
	}
}

void flight_task_3(void) 
{
	global_params.AltitudeDesired=700;
	global_params.MotorLocked=0;
	pidVerVelocity.wheInteg=true;
	VERVELCON=0;
	throttle=2600;
}

void flight_task_4(void)
{
	global_params.AltitudeDesired=155;
	while(true)
	{
		if(fabs(global_params.AltitudeDesired-global_params.AltitudeActual)<100)
		{
				global_params.MotorLocked=1;
				Motor_PwmRefresh(2000,2000,2000,2000);
				break;
		}
		vTaskDelay(100);
	}
}


void BEEP_Init(void)
{   
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOF时钟
  
  //初始化蜂鸣器对应引脚GPIO B0
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
	
  GPIO_SetBits(GPIOB,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5);  
}