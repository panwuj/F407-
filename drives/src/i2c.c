#include "delay.h"
#include "i2c.h"

uint32_t i2c_soft_err=0;

void i2cdevInit()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//先使能外设IO PORTC时钟 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	
	   
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT ;  //普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	IIC_SCL=1;
	IIC_SDA=1;
}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
  for(i=0;i<8;i++ )
	{
    IIC_SCL=0; 
    delay_us(2);
		IIC_SCL=1;
    receive<<=1;
    if(READ_SDA)receive++;   
		delay_us(1); 
	}					 
  if (!ack)
    IIC_NAck();//发送nACK
  else
    IIC_Ack(); //发送ACK   
  return receive;
}
void i2cWriteByte(u8 DevID,u8 Addr,u8 Dat)
{
	IIC_Start();
	IIC_Send_Byte(DevID << 1| 0);								//发送设备地址和写信号
	IIC_Wait_Ack();
	IIC_Send_Byte(Addr);
	IIC_Wait_Ack();
	IIC_Send_Byte(Dat);
	IIC_Wait_Ack();
	IIC_Stop();
	delay_ms(10);
}
void i2cReadBytes(u8 DevID,u8 Addr,u8 *Pbuf,u8 Num)
{
	u8 i;
	u8 err;
	IIC_Start();
	IIC_Send_Byte(DevID << 1 | 0);	//发送设备地址和写信号
	err=IIC_Wait_Ack();
	if(err) 
	{
		i2c_soft_err++;
		return;
	}
	IIC_Send_Byte(Addr);
	err=IIC_Wait_Ack();
	if(err)
	{
		i2c_soft_err++;
		return;
	}
	IIC_Start();
	IIC_Send_Byte(DevID << 1 | 1);	//发送设备地址和读信号
	err=IIC_Wait_Ack();
	if(err) 
	{
		i2c_soft_err++;
		return;
	}
	for(i = 0;i < (Num - 1);i ++)
	{
		Pbuf[i] = IIC_Read_Byte(1);
	}
	Pbuf[i] = IIC_Read_Byte(0);
	IIC_Stop();
}











