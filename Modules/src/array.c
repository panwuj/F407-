#include "array.h"

void ByteArrayCopy(unsigned char *buf1,unsigned char *buf2,uint16_t i_1,uint16_t i_2,uint16_t length)
{
	uint16_t i;
	for(i=0;i<length;i++)
		buf2[i_1+i]=buf1[i_2+i];
}

unsigned char ByteArrayAdd8(unsigned char *buf,uint16_t index,uint16_t length)
{
  unsigned char sum=0;
  uint16_t i;
  for(i=index;i<length;i++)
  {
    sum += buf[i];
  }
  return sum;
}

//unsigned int ByteArrayAdd32(unsigned int *buf,uint16_t index,uint16_t length)
//{
//	unsigned int sum=0;
//	
//	return sum;
//}

//quickly sort
// void quickly_sort(...)
