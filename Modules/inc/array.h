#ifndef __ARRAY_H
#define __ARRAY_H
#include "stdint.h"

void ByteArrayCopy(unsigned char *buf1,unsigned char *buf2,uint16_t i_1,uint16_t i_2,uint16_t length);
unsigned char ByteArrayAdd8(unsigned char *buf,uint16_t index,uint16_t length);

#endif
