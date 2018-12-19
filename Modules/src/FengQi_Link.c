#include "FengQi_Link.h"
#include "DataConvert.h"
#include "params.h"
#include "USART.h"

unsigned char FQ_ReplyCode[11]={0x24,0x12,0x07,0x00,0x00,0x00,0x00,0x00};
unsigned char FrameOfFlightData[37]={0x24,0x12,0xA7,0x1D,0x00,0xea};
unsigned char GPSLoadBuf[26]={0x24,0x12,0x06,0x12,0x00,0x15};
unsigned char FQ_Rc_ErrorCode;
unsigned char Transparent_buf_1[255]={0x24,0x32,0x06};
unsigned char ToSearchingModuleBuf[255];

unsigned char GPS_Rc_Counter=0;
unsigned char ack_counter=0;
unsigned char GPS_ack=0;
unsigned char S=0;
//unsigned char Transparent_buf_2[255]={0x24,0x32,0x06};

unsigned char Transmit_Add_CRC(unsigned char InputBytes[],unsigned char data_lenth)
{
  unsigned char byte_crc=0;
  unsigned char i;
  for(i=0;i<data_lenth;i++)
  {
    byte_crc+=InputBytes[i];
  }
  return byte_crc;
}
/*
return 0:Non error;1:Data check error;2:Data receive incomplete;3:Data lost frame header
*/
unsigned char FQ_ReceivedDataCRC(unsigned char *buf,uint16_t buf_len)
{
	unsigned char data_len;
	unsigned char crc=0;
	if(buf[0]!=0x24&&(buf[1]!=0x21||buf[1]!=0x23))
		return 3;
	data_len=buf[3]+8;
	if(data_len!=buf_len)
		return 2;
	crc=Transmit_Add_CRC(buf,data_len-1);
	if(crc!=buf[data_len-1])
		return 1;
	else
		return 0;
}

void FQ_AckSend(unsigned short addr)
{
	FQ_ReplyCode[0]=0x24;
  FQ_ReplyCode[1]=0x12;
  FQ_ReplyCode[2]=0x07;
  FQ_ReplyCode[3]=0x03;
	FQ_ReplyCode[4]=addr>>8;
	FQ_ReplyCode[5]=addr&0xff;
          
  FQ_ReplyCode[9]=0x00;
	FQ_ReplyCode[10]=Transmit_Add_CRC(FQ_ReplyCode,10);        
//  USARTSendDatas(UART4,11,FQ_ReplyCode);
	USARTSendDatasByDMA(UART4,FQ_ReplyCode,11);
}

void FQ_DataAnalysis(unsigned char *buf,unsigned char buf_len)
{
	unsigned short addr=buf[4]<<8|buf[5];
  unsigned char data_len=buf[3];
  unsigned char i;
	unsigned char cur_p;//current point.present number of GPS point
	FQ_Rc_ErrorCode=FQ_ReceivedDataCRC(buf,buf_len);
	FQ_ReplyCode[6]=0x00;
  FQ_ReplyCode[7]=0x00;
  FQ_ReplyCode[8]=FQ_Rc_ErrorCode;
	if(FQ_Rc_ErrorCode!=0)
	{
		FQ_Rc_ErrorCode=1;     //need redefine again.
		FQ_AckSend(00);
		return;
	}
  if(buf[1]==0x23)
  {
    for(i=0;i<data_len;i++)
    {
      ToSearchingModuleBuf[i]=buf[i+6];
    }
//  USARTSendDatas(UART5,data_len,ToSearchingModuleBuf);
    return;
  }
	if(buf[1]==0x21)
  {
    if(buf[2]==0x03)//read register
    {
			switch(addr)
			{
				case 0x0015:
					GPSLoadBuf[6]=global_params.GPSPointCounter;
					GPSLoadBuf[7]=buf[6];
					FloatToBytes(GPSLoadBuf+8,global_params.GPSTargetPoints[buf[6]-1].latitude);
					FloatToBytes(GPSLoadBuf+12,global_params.GPSTargetPoints[buf[6]-1].longitude);
					FloatToBytes(GPSLoadBuf+16,global_params.GPSTargetPoints[buf[6]-1].altitude);
					UInt16ToBytes(GPSLoadBuf+20,global_params.GPSTargetPoints[buf[6]-1].hover_time);
					GPSLoadBuf[22]=global_params.GPSTargetPoints[buf[6]].speed;
					GPSLoadBuf[23]=global_params.GPSTargetPoints[buf[6]].photo;
					GPSLoadBuf[24]=0;
					GPSLoadBuf[25]=Transmit_Add_CRC(GPSLoadBuf,25);
					USARTSendDatas(UART4,GPSLoadBuf,26);
					break;
			}
			return;
    }
    if(buf[2]==0x06)//write register
    {
      switch(addr)
		  {
				case 0x0000:
          global_params.ControlMode=buf[6];
          break;
				case 0x0001:
          global_params.AltitudeDesired=ToFloat32(buf+6);
          global_params.AttitudeDesired.roll=ToInt16(buf+10)/100.0;
					global_params.AttitudeDesired.pitch=ToInt16(buf+12)/100.0;
					global_params.AttitudeDesired.yaw=ToInt16(buf+14)/100.0;
          break;
				case 0x0005://Task Launch
					global_params.TaskLaunch=buf[6];
					break;
				case 0x0006://Task Pause Or Resume.
					global_params.TaskPauseOrResume=buf[6];
					break;
				case 0x0007:
					global_params.MotorLocked=buf[6];
					break;
				case 0x0008:
					global_params.EmergenciesDispose=buf[6];
					break;
				case 0x0009:
					global_params.GoHome=buf[6];
					break;
        case 0x0014://upload the air route.(setting GPS point).
					cur_p=buf[7];
          global_params.GPSUploaded=0;
          if(buf[6]>20)
             break;
          global_params.GPSPointCounter=buf[6];//total of points.
          global_params.GPSTargetPoints[cur_p-1].latitude=ToFloat32(buf+8);
          global_params.GPSTargetPoints[cur_p-1].longitude=ToFloat32(buf+12);  
          global_params.GPSTargetPoints[cur_p-1].altitude=ToFloat32(buf+16);
          //remian time
          //velocity
          //taking photos 
          if(cur_p==global_params.GPSPointCounter)
          {
             global_params.GPSUploaded=1;
          }
          FQ_ReplyCode[6]=global_params.GPSPointCounter;
          FQ_ReplyCode[7]=cur_p;
          break;
      }
			if(buf[data_len+6]==0x01)
			{
			FQ_AckSend(addr);
			}
    }

	}	
}

void FQ_FrameOfFlightDataUpdate(void)
{
	short Int16_Roll,Int16_Pitch,Int16_Yaw;
	unsigned short i;
	FrameOfFlightData[6]=global_params.ControlMode;
	FloatToBytes(FrameOfFlightData+7,56.42);     //flight altitude (byte7--byte10)
	Int16_Roll=global_params.AttitudeActual.roll*100;
	Int16_Pitch=global_params.AttitudeActual.pitch*100;
//	Int16_Roll=100;
//	Int16_Pitch=100;
	Int16_Yaw=0;
	Int16ToBytes(FrameOfFlightData+11,Int16_Roll);
	Int16ToBytes(FrameOfFlightData+13,Int16_Pitch);
	Int16ToBytes(FrameOfFlightData+15,Int16_Yaw);
	Int16ToBytes(FrameOfFlightData+17,3500);
	Int16ToBytes(FrameOfFlightData+19,2600); //su du
	FrameOfFlightData[21]=56;
	FloatToBytes(FrameOfFlightData+22,121.524);
	FloatToBytes(FrameOfFlightData+26,31.084);
	FrameOfFlightData[30]=42;
	FrameOfFlightData[31]=88;
	Int16ToBytes(FrameOfFlightData+32,4230);
	FrameOfFlightData[34]=55;
	FrameOfFlightData[35]=0;
	FrameOfFlightData[36]=0;
	for(i=0;i<36;i++)
	{
		FrameOfFlightData[36] += FrameOfFlightData[i];
	}
	USARTSendDatasByDMA(UART4,FrameOfFlightData,37);
	
}
void FQ_Transparent_Transmit_for_SearchingModule(unsigned char *buf,unsigned char buf_len)
{
  unsigned char i;
  Transparent_buf_1[3]=buf_len;
  Transparent_buf_1[4]=0x00;
  Transparent_buf_1[5]=0x00;

  for(i=0;i<buf_len;i++)
  {
    Transparent_buf_1[i+6]=buf[i];
  }
  Transparent_buf_1[buf_len+6]=0x00;
  Transparent_buf_1[buf_len+7]=Transmit_Add_CRC(Transparent_buf_1,buf_len+6);
  USARTSendDatasByDMA(UART4,Transparent_buf_1,buf_len+8);
}
