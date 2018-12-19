#ifndef __DATACONVERT_H
#define __DATACONVERT_H

#include "stdint.h"
typedef union
{
	unsigned char Byte[8];
	double DoubleValue;
	float  FloatValue;
	int 	 IntValue;
	unsigned int UIntValue;
	short  ShortValue;
	unsigned short UShortValue;
}Data_Convert;

double ToDouble64(unsigned char *Bytes);
float ToFloat32(unsigned char *Bytes);
int ToInt32(unsigned char *Bytes);
unsigned char ToUInt32(unsigned char *Bytes);
short ToInt16(unsigned char *Bytes);
unsigned short ToUInt16(unsigned char *Bytes);
void FloatToBytes(unsigned char *Bytes,float num);
void DoubleToBytes(unsigned char *Bytes,double num);
void Int16ToBytes(unsigned char *Bytes,short num);
void UInt16ToBytes(unsigned char *Bytes,unsigned short num);
#endif
