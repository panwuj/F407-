#include "communicate.h"
#include "FengQi_Link.h"
#include "stdint.h"

void ReadDataTransfer()
{
	if(!uart4.update)
		return;
	uart4.locked=1;
	//FQ_DataAnalysis(uart4.rx_buf,uart4.rx_length);
	ANO_Data_Analysis(uart4.rx_buf,uart4.rx_length);
	uart4.locked=0;
	uart4.update=0;
}

void GetSystemInput(void)
{
//	ReadRemoteControl();
	ReadDataTransfer();
	//ReadUSARTData();
}
void DataDelayedTx(void)
{
	static uint8_t delay_count=0;
	if(delay_count==20)//100ms,10Hz frequency of data send;
	{
		delay_count=0;
		FQ_FrameOfFlightDataUpdate();
		return;
	}
	delay_count++;
}
