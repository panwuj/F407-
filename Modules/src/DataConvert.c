#include "DataConvert.h"

double ToDouble64(unsigned char *Bytes)
{
	unsigned char i;
	Data_Convert convert;
	for(i=0;i<8;i++)
	{
		convert.Byte[i]=Bytes[i];
	}
	return convert.DoubleValue;
}

float ToFloat32(unsigned char *Bytes)
{
	unsigned char i;
	Data_Convert convert;
	for(i=0;i<4;i++)
	{
		convert.Byte[i]=Bytes[i];
	}
	return convert.FloatValue;
}

int ToInt32(unsigned char *Bytes)
{
	unsigned char i;
	Data_Convert convert;
	for(i=0;i<4;i++)
	{
		convert.Byte[i]=Bytes[i];
	}
	return convert.IntValue;
}

unsigned char ToUInt32(unsigned char *Bytes)
{
	unsigned char i;
	Data_Convert convert;
	for(i=0;i<4;i++)
	{
		convert.Byte[i]=Bytes[i];
	}
	return convert.UIntValue;
}

short ToInt16(unsigned char *Bytes)
{
	unsigned char i;
	Data_Convert convert;
	for(i=0;i<2;i++)
	{
		convert.Byte[i]=Bytes[i];
	}
	return convert.ShortValue;
}

unsigned short ToUInt16(unsigned char *Bytes)
{
	unsigned char i;
	Data_Convert convert;
	for(i=0;i<2;i++)
	{
		convert.Byte[i]=Bytes[i];
	}
	return convert.UShortValue;
}
void FloatToBytes(unsigned char *Bytes,float num)
{
	unsigned char i;
	Data_Convert convert;
	convert.FloatValue=num;
	for(i=0;i<4;i++)
	{
		Bytes[i]=convert.Byte[i];
	}
}
void DoubleToBytes(unsigned char *Bytes,double num)
{
	unsigned char i;
	Data_Convert convert;
	convert.FloatValue=num;
	for(i=0;i<8;i++)
	{
		Bytes[i]=convert.Byte[i];
	}
}

void Int16ToBytes(unsigned char *Bytes,short num)
{
	unsigned char i;
	Data_Convert convert;
	convert.ShortValue=num;
	for(i=0;i<2;i++)
	{
		Bytes[i]=convert.Byte[i];
	}
}
void UInt16ToBytes(unsigned char *Bytes,unsigned short num)
{
	unsigned char i;
	Data_Convert convert;
	convert.UShortValue=num;
	for(i=0;i<2;i++)
	{
		Bytes[i]=convert.Byte[i];
	}
}
