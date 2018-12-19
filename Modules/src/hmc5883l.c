#include "hmc5883l.h"
#include "stdio.h"

#define hmc_Filter_High 0.8f
#define hmc_Filter_low 0.2f

uint8_t hmc5883l_buffer[6];
uint8_t hmc_id=0;
S_INT16_XYZ HMC5883L_LAST={0,0,0};
MAGN_DATA hmc_latest;
MAGN_DATA hmc5883l={0,0,0};

void HMC5883L_Init(void)
{
	i2cReadBytes(HMCAddress,IDENTIFY_REG_A,&hmc_id,1);
	while(hmc_id!=0x48)
	{
		LED0=!LED0;
//		printf("%c",*DeviceID);
		delay_ms(200);
		i2cReadBytes(HMCAddress,IDENTIFY_REG_A,&hmc_id,1);
	}
  i2cWriteByte(HMCAddress,CONFIG_REG_A,0x18);   //50Hz//0x71
  i2cWriteByte(HMCAddress,CONFIG_REG_B,0x20);		//0x60
  i2cWriteByte(HMCAddress,MOD_REG,0x00);   //    0x01
}
//int16_t Xo=-79,Yo=-127,Zo=46;//first
//int16_t Xo=-374,Yo=-391,Zo=206;
//int16_t Xo=-17,Yo=-15,Zo=-65;
int16_t Xo=-85,Yo=-131,Zo=-15;
void HMC5883L_Get_Magnetism(void)
{
	//__disable_irq();
	i2cReadBytes(HMCAddress,DATA_X_MSB,hmc5883l_buffer,6);
	//__enable_irq();
	HMC5883L_LAST.X=((((int16_t)hmc5883l_buffer[0]) << 8) | hmc5883l_buffer[1]);
	HMC5883L_LAST.Z=((((int16_t)hmc5883l_buffer[2]) << 8) | hmc5883l_buffer[3]);
	HMC5883L_LAST.Y=((((int16_t)hmc5883l_buffer[4]) << 8) | hmc5883l_buffer[5]);
	
	hmc_latest.x=HMC5883L_LAST.X-Xo;
	hmc_latest.y=HMC5883L_LAST.Y-Yo;
	hmc_latest.z=HMC5883L_LAST.Z-Zo;
	
	hmc5883l.x =hmc_Filter_High*hmc5883l.x + hmc_Filter_low*hmc_latest.x;
	hmc5883l.y =hmc_Filter_High*hmc5883l.y + hmc_Filter_low*hmc_latest.y;
	hmc5883l.z =hmc_Filter_High*hmc5883l.z + hmc_Filter_low*hmc_latest.z;
//	printf("%d,%d,%d;\r\n",HMC5883L_LAST.X,HMC5883L_LAST.Y,HMC5883L_LAST.Z);
//	delay_ms(200);
}