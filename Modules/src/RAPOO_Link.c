#include "RAPOO_Link.h"
#include "DataConvert.h"
//return the data receive status to GroundControl
//frame header:0x0A
//data length:5
//ID         :0x0F
//status     :0  non error;1  data received error  
//verify         
unsigned char data_rc_status[5]={0x0A,5,0x0F,0,0};
unsigned char RAPOO_data_send_buf[256];
uint16_t RAPOO_remote_rc[10];

unsigned char RAPOO_Transmit_Add_CRC(unsigned char InputBytes[],unsigned char data_lenth)
{
  unsigned char byte_crc=0;
  unsigned char i;
  for(i=0;i<data_lenth;i++)
  {
    byte_crc+=InputBytes[i];
  }
  return byte_crc;
}
void return_ack(void)
{
  data_rc_status[4]=Transmit_Add_CRC(data_rc_status,4);
  USARTSendDatasByDMA(UART4,data_rc_status,5);//when using the uart4
}
uint8_t RAPOO_data_rc_anasysis(unsigned char *rc_buf)
{
  uint8_t  status;
  unsigned data_length=rc_buf[1];
  unsigned data_id    =rc_buf[2];
  unsigned data_crc=Transmit_Add_CRC(rc_buf,data_length-1);
  
  unsigned char i,j;//declare the variables here to be compatible with the C
  if(rc_buf[0]!=0xA0)
  {
    status=data_rc_status[3]=0x02;//data lost the frame header.
    //return_ack();
    return status;
  }
  if(data_crc!=rc_buf[data_length-1])
  {
    status=data_rc_status[3]=0x01;//data verify error
//    return_ack();
    return status;
  }
  //receive non error
  status=0;
  if(data_id==0x01)
  {
    for(i=3,j=0;i<23;i+=2,j++)
    {
      RAPOO_remote_rc[j]=rc_buf[i]<<8|rc_buf[i+1];
    }
    return status;
  }
  if(data_id==0x0F)//PID parameters setting
  {
//    PIDParamsSetByGCS(rc_buf);
    data_rc_status[3]=0;
    return_ack();
    return status;
  }
	return 0;
}

void RAPOO_data_send(void)
{
	unsigned char length=20;
	RAPOO_data_send_buf[0]=0x0A;
	RAPOO_data_send_buf[1]=length;
	RAPOO_data_send_buf[2]=0xC1;
	
	FloatToBytes(RAPOO_data_send_buf+3,body_vel[0]);
	FloatToBytes(RAPOO_data_send_buf+7,body_vel[1]);
	FloatToBytes(RAPOO_data_send_buf+11,0);
	FloatToBytes(RAPOO_data_send_buf+15,0);
	
//	FloatToBytes(RAPOO_data_send_buf+3,altitude_baro);
//	FloatToBytes(RAPOO_data_send_buf+7,global_params.AltitudeActual);
//	FloatToBytes(RAPOO_data_send_buf+11,VerVelEst);
//	FloatToBytes(RAPOO_data_send_buf+15,hor_acc.z);
	
//	FloatToBytes(RAPOO_data_send_buf+3,body_vel[0]);
//	FloatToBytes(RAPOO_data_send_buf+7,body_vel[1]);
//	FloatToBytes(RAPOO_data_send_buf+11,VelDesired[0]);
//	FloatToBytes(RAPOO_data_send_buf+15,VelDesired[1]);
	
	RAPOO_data_send_buf[length-1]=RAPOO_Transmit_Add_CRC(RAPOO_data_send_buf,length-1);
	USARTSendDatasByDMA(UART4,RAPOO_data_send_buf,length);
}