#ifndef __HMC5883L_H
#define __HMC5883L_H
#include "main.h"
/* HMC5883 Register Map */
#define HMCAddress						0x1e
#define	CONFIG_REG_A					0x00
#define	CONFIG_REG_B					0x01  
#define	MOD_REG							0x02  
#define	DATA_X_MSB						0x03  
#define	DATA_X_LSB						0x04  
#define	DATA_Z_MSB						0x05  
#define	DATA_Z_LSB						0x06  
#define	DATA_Y_MSB						0x07  
#define	DATA_Y_LSB						0x08  
#define	STATE_REG	  					0x09  
#define	IDENTIFY_REG_A					0x0A  
#define	IDENTIFY_REG_B					0x0B  
#define	IDENTIFY_REG_C					0x0C  

typedef struct
{
	int16_t X;
	int16_t Y;
	int16_t Z;
}S_INT16_XYZ;

typedef struct
{
	float x;
	float y;
	float z;
}MAGN_DATA;

extern MAGN_DATA hmc5883l;
extern S_INT16_XYZ HMC5883L_LAST;
void HMC5883L_Init(void);
void HMC5883L_Get_Magnetism(void);

#endif
