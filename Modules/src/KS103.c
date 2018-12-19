#include "KS103.h"

const unsigned char KS103Cmd[3]={0xe8,0x02,0xb0};
//u8 KS103Cmd[3]={0xe8,0x02,0xb0};

void SendCmdToKS103(void)
{
//	USARTSendDatas(USART3,(u8*)KS103Cmd,3);
	  while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){}    
	  USART_SendData(USART3, 0xe8);
		delay_us(50);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){}
	  USART_SendData(USART3, 0x02);
		delay_us(50);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){}
	  USART_SendData(USART3, 0xb0);
}
u8 KS103_height_update=0;
int16_t KS103_height=0;
int KS103_Count=0;
int16_t KS103GetHeigth(void)
{
	static uint32_t KS_time_now=0,KS_time_last=0;
	static uint8_t first_run=1;
	float dt;
	static int16_t height=0;
	static int16_t height_last=0;
	
	KS_time_now = micros();  
  if(KS_time_now < KS_time_last) 
	{     
		dt = (KS_time_now + (0xffffffff- KS_time_last))/1000.0f;
  }
	else
	{ 
		dt = (KS_time_now - KS_time_last)/1000.0f;
	}
	if(first_run)
	{
		first_run=0;
		SendCmdToKS103();					//first run,send command to KS103 for measurement.
		KS_time_last=KS_time_now;
		return 0;
	}
	if(usart3.update)
	{
		usart3.update=0;
		if(usart3.rx_length==2)
		{
			usart3.locked=1;
			height=usart3.rx_buf[0]<<8|usart3.rx_buf[1];
			KS103_height_update=1;
			usart3.locked=0;
			KS103_Count++;
		}
	}
	if(dt>=100)
	{
		SendCmdToKS103();			 //Send command to KS103 for next measurement.10Hz
		KS_time_last=KS_time_now;
	}
	return height;
}