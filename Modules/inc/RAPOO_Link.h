#ifndef __RAPOO_H
#define __RAPOO_H
#include "stdint.h"
#include "main.h"
extern uint16_t RAPOO_remote_rc[10];
unsigned char Transmit_Add_CRC(unsigned char InputBytes[],unsigned char data_lenth);
void RAPOO_data_send(void);
//void RAPOO_data_rc_anasysis(unsigned char *rc_buf);

#endif
