#include "mpu6500.h"
#include "math.h"

#define filter_high 0.8
#define filter_low  0.2

//#define g_filter_high 0.8
//#define g_filter_low  0.2

inertial_sensor_data mpu6500;
inertial_sensor_data mpu6500_raw;
inertial_sensor_data mpu6500_offest;
//the variable contains the accelation (body frame) converted from sensor data.
FLOAT_XYZ body_acc;		//unit m/s^2.  	
FLOAT_XYZ body_gyro;  //unit rad/s.  
FLOAT_XYZ hor_acc;		//plane acceleration in horizontal coordinates.
FLOAT_XYZ ned_acc;
//FLOAT_XYZ body_magn;

uint8_t mpu_buf[14];

void MPU6500_CS_LOW(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}

void MPU6500_CS_HIGH(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
}

void MPU6500_Pin_Init()
{
	GPIO_InitTypeDef   GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//PB1 for detecting  data is ready.
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);   
	GPIO_ResetBits(GPIOB,GPIO_Pin_1);
  delay_ms(1);
  MPU6500_CS_HIGH();
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
}

void MPU6500_Write_Reg(unsigned char addr,unsigned char value)
{
	u8 temp=0;
	MPU6500_CS_LOW();
	SPI1_ReadWriteByte(addr,&temp);
	SPI1_ReadWriteByte(value,&temp);
	MPU6500_CS_HIGH();
}
void MPU6500_Read_Regs(unsigned char *buf,uint8_t addr,uint8_t num)
{
	uint8_t i;
	u8 temp=0;
	MPU6500_CS_LOW();
	SPI1_ReadWriteByte(addr|=0x80,&temp);
	for(i=0;i<num;i++)
		buf[i]=SPI1_ReadWriteByte(0xFF,&temp);
	MPU6500_CS_HIGH();
}

unsigned char id;
void MPU6500_Init(void)
{
	MPU6500_Pin_Init();
	
	delay_ms(1);
	MPU6500_Read_Regs(&id,0x75,1);
	while(id!=0x70)
	{
		delay_ms(250);
		MPU6500_Read_Regs(&id,0x75,1);
		LED1=!LED1;
	}
	LED1=1;
	delay_ms(1);
	MPU6500_Write_Reg(MPU6500_USER_CTRL,0x10);  //
	delay_ms(1);
	MPU6500_Write_Reg(MPU6500_PWR_MGMT_1,0x80); //Set clock source
	delay_ms(10);
	MPU6500_Write_Reg(MPU6500_PWR_MGMT_1,0x03);
	delay_ms(1);
	MPU6500_Write_Reg(MPU6500_PWR_MGMT_2,0x00); 
	delay_ms(1);
	MPU6500_Write_Reg(MPU6500_SMPLRT_DIV,0x00);
	delay_ms(1);
	MPU6500_Write_Reg(MPU6500_CONFIG,0x01);
	delay_ms(1);
	MPU6500_Write_Reg(MPU6500_GYRO_CONFIG,0x18);
	delay_ms(1);
	MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG,0x18);
	delay_ms(1);
	MPU6500_Write_Reg(MPU6500_INT_PIN_CFG,0x30);
	delay_ms(1);
	MPU6500_Write_Reg(MPU6500_INT_ENABLE,0x01);
	delay_ms(1);
	MPU6500_Set_Offest();
}

unsigned char MPU6500_DRY(void)
{
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==Bit_SET)
		return 1;
	else
		return 0;
}
void MPU6500_Set_Offest(void)
{
	mpu6500_offest.ax=0;
	mpu6500_offest.ay=0;
	mpu6500_offest.az=0;
	mpu6500_offest.gx=0;
	mpu6500_offest.gy=0;
	mpu6500_offest.gz=0;
}
void MPU6500_Get_Motion6(void)
{
	if(!MPU6500_DRY())
		return;
	__disable_irq();
	MPU6500_Read_Regs(mpu_buf,MPU6500_ACCEL_XOUT_H,14);
	__enable_irq();
	mpu6500_raw.ax = (((int16_t)mpu_buf[0]) << 8) | mpu_buf[1];
	mpu6500_raw.ay = (((int16_t)mpu_buf[2]) << 8) | mpu_buf[3];
	mpu6500_raw.az = (((int16_t)mpu_buf[4]) << 8) | mpu_buf[5];
	
	mpu6500_raw.gx = ((((int16_t)mpu_buf[8]) << 8) | mpu_buf[9])-mpu6500_offest.gx;
	mpu6500_raw.gy = ((((int16_t)mpu_buf[10]) << 8) | mpu_buf[11])-mpu6500_offest.gy;
	mpu6500_raw.gz = ((((int16_t)mpu_buf[12]) << 8) | mpu_buf[13])-mpu6500_offest.gz;
		
	mpu6500.ax=mpu6500.ax*filter_high+mpu6500_raw.ax*filter_low;
	mpu6500.ay=mpu6500.ay*filter_high+mpu6500_raw.ay*filter_low;
	mpu6500.az=mpu6500.az*filter_high+mpu6500_raw.az*filter_low;
	
	
	mpu6500.gx =(mpu6500.gx+mpu6500_raw.gx)/2.0f;
	mpu6500.gy =(mpu6500.gy+mpu6500_raw.gy)/2.0f;
	mpu6500.gz =(mpu6500.gz+mpu6500_raw.gz)/2.0f;
	

	body_acc.x=mpu6500.ax*ACC_SCALE;
	body_acc.y=mpu6500.ay*ACC_SCALE;
	body_acc.z=mpu6500.az*ACC_SCALE;
	
	body_gyro.x=mpu6500.gx*GYRO_SCALE;
	body_gyro.y=mpu6500.gy*GYRO_SCALE;
	body_gyro.z=mpu6500.gz*GYRO_SCALE;
}

