#include "usart.h"	
#include "stdio.h"
#include "dma.h"
#include "array.h"

//uint8_t		USART1_Tx_buffer[USART1_TX_BUFFER_SIZE] = {0};
uint8_t		USART1_Rx_Buffer[USART1_RX_BUFFER_SIZE] = {0};

uint8_t 	USART2_Rx_Buffer[USART2_RX_BUFFER_SIZE] = {0};
uint8_t 	USART2_Tx_Buffer[USART2_RX_BUFFER_SIZE] = {0};
uint8_t 	USART2_Rx_DMA_Buffer[USART2_RX_BUFFER_SIZE] = {0};
uint8_t 	USART2_Tx_DMA_Buffer[USART2_RX_BUFFER_SIZE] = {0};

uint8_t		USART3_Rx_Buffer[USART3_RX_BUFFER_SIZE] = {0};
uint8_t		USART3_Tx_Buffer[USART3_RX_BUFFER_SIZE] = {0};
uint8_t		USART3_Rx_DMA_Buffer[USART3_RX_BUFFER_SIZE] = {0};
uint8_t		USART3_Tx_DMA_Buffer[USART3_RX_BUFFER_SIZE] = {0};

uint8_t		UART4_Tx_Buffer[UART4_TX_BUFFER_SIZE] = {0};
uint8_t		UART4_Rx_Buffer[UART4_RX_BUFFER_SIZE] = {0};
uint8_t		UART4_Tx_DMA_Buffer[UART4_TX_BUFFER_SIZE] = {0};
uint8_t		UART4_Rx_DMA_Buffer[UART4_RX_BUFFER_SIZE] = {0};

uint8_t		UART5_Tx_Buffer[UART5_RX_BUFFER_SIZE] ={0};
uint8_t		UART5_Rx_Buffer[UART5_RX_BUFFER_SIZE] = {0};
uint8_t		UART5_Tx_DMA_Buffer[UART5_RX_BUFFER_SIZE] ={0};
uint8_t		UART5_Rx_DMA_Buffer[UART5_RX_BUFFER_SIZE] = {0};

usart_rt usart1,usart2,usart3,uart4,uart5;

//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
//#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
int _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((UART4->SR&0X40)==0);//循环发送,直到发送完毕   
	UART4->DR = (u8) ch;      
	return ch;
}
#endif
 
//USART1 for SBUS,using the DMA to receive
void USART1_Init(uint32_t bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
 
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); 
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	
  USART_Init(USART1, &USART_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、	
  
	USART_ClearFlag(USART1, USART_FLAG_TC);
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启接收中断
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//开启空闲中断
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	USART_Cmd(USART1, ENABLE);
	
	//initialize the DMA channel.
	DMA_Config(DMA2_Stream2,DMA_Channel_4, 
						 (uint32_t)&(USART1->DR),
						 (uint32_t)USART1_Rx_Buffer,
						 DMA_DIR_PeripheralToMemory,
						 USART1_RX_BUFFER_SIZE);
						 
	usart1.locked=0;
	usart1.tx_length=0;
	usart1.rx_length=0;
	usart1.tx_size=0;
	usart1.rx_size=USART1_RX_BUFFER_SIZE;
	usart1.tx_buf=NULL;
	usart1.rx_buf=USART1_Rx_Buffer;
}
//USART2 for device1(BeiDou GPS) transparent transmit,using DMA send and receive
void USART2_Init(uint32_t bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOD5复用为USART2_TX
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOD6复用为USART2_RX

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2| GPIO_Pin_3; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
  USART_Init(USART2, &USART_InitStructure); 
  
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//开启相关中断
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);	
	USART_Cmd(USART2, ENABLE); 
	
	//DMA for rx
	DMA_Config(DMA1_Stream5,DMA_Channel_4, 
						 (uint32_t)&(USART2->DR),
						 (uint32_t)USART2_Rx_DMA_Buffer,
						 DMA_DIR_PeripheralToMemory,  
						 USART2_RX_BUFFER_SIZE);
	//DMA for tx					 
	DMA_Config(DMA1_Stream6,DMA_Channel_4, 
						 (uint32_t)&(USART2->DR),
						 (uint32_t)USART2_Tx_DMA_Buffer,
						 DMA_DIR_MemoryToPeripheral,
						 USART2_RX_BUFFER_SIZE);
	usart2.update=0;
	usart2.locked=0;
	usart2.tx_length=0;
	usart2.rx_length=0;
	usart2.rx_size=USART2_RX_BUFFER_SIZE;
	usart2.tx_size=USART2_TX_BUFFER_SIZE;
	usart2.tx_buf=USART2_Tx_DMA_Buffer;
	usart2.rx_buf=USART2_Rx_DMA_Buffer;
}
//USART3 for GPS receive,using DMA
void USART3_Init(uint32_t bound)
{
	//GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
 
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); //GPIOD8复用为USART3_TX
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); //GPIOD9复用为USART3_RX
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure);
	
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
//	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);	
	USART_Cmd(USART3, ENABLE); 
	
		//DMA for rx
	DMA_Config(DMA1_Stream1,DMA_Channel_4, 
						 (uint32_t)&(USART3->DR),
						 (uint32_t)USART3_Rx_DMA_Buffer,
						 DMA_DIR_PeripheralToMemory,  
						 USART3_RX_BUFFER_SIZE);
	//DMA for tx					 
	DMA_Config(DMA1_Stream3,DMA_Channel_4, 
						 (uint32_t)&(USART3->DR),
						 (uint32_t)USART3_Tx_DMA_Buffer,
						 DMA_DIR_MemoryToPeripheral,
						 USART3_RX_BUFFER_SIZE);
	usart3.update=0;
	usart3.locked=0;
	usart3.tx_length=0;
	usart3.rx_length=0;
	usart3.rx_size=USART3_RX_BUFFER_SIZE;
	usart3.tx_size=USART3_TX_BUFFER_SIZE;
	usart3.tx_buf=USART3_Tx_Buffer;
	usart3.rx_buf=USART3_Rx_Buffer;
}

//UART4 for data transfer,using DMA
void UART4_Init(uint32_t bound)
{
	USART_InitTypeDef USART_InitStructure; 
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure; 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);	
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF; 	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 	
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	
  USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);
	
	USART_ClearFlag(UART4,USART_FLAG_RXNE);
//	USART_ITConfig(UART4,USART_IT_RXNE,ENABLE);
  USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);

	USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE); 
	USART_Cmd(UART4, ENABLE); 
	
		//DMA for rx
	DMA_Config(DMA1_Stream2,DMA_Channel_4, 
						 (uint32_t)&(UART4->DR),
						 (uint32_t)UART4_Rx_DMA_Buffer,
						 DMA_DIR_PeripheralToMemory,  
						 UART4_RX_BUFFER_SIZE);
	//DMA for tx					 
	DMA_Config(DMA1_Stream4,DMA_Channel_4, 
						 (uint32_t)&(UART4->DR),
						 (uint32_t)UART4_Tx_DMA_Buffer,
						 DMA_DIR_MemoryToPeripheral,
						 UART4_TX_BUFFER_SIZE);
	uart4.update=0;					 
	uart4.locked=0;
	uart4.tx_length=0;
	uart4.rx_length=0;
	uart4.rx_size=UART4_RX_BUFFER_SIZE;
	uart4.tx_size=UART4_TX_BUFFER_SIZE;
	uart4.tx_buf=UART4_Tx_Buffer;
	uart4.rx_buf=UART4_Rx_Buffer;
}

// UART5 for device2(searching module) transparent transmit,using DMA
void UART5_Init(uint32_t bound)                                                     
{
	USART_InitTypeDef USART_InitStructure; 
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure; 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;        
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	
	USART_InitStructure.USART_BaudRate            = bound;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;	
	USART_InitStructure.USART_Parity              = USART_Parity_No;	
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART5, &USART_InitStructure);
	
	USART_ClearFlag(UART5,USART_FLAG_RXNE);       
//	USART_ITConfig(UART5,USART_IT_RXNE,DISABLE);	
	USART_ITConfig(UART5,USART_IT_IDLE,ENABLE);		
	USART_DMACmd(UART5,USART_DMAReq_Rx,ENABLE);	
	
	USART_Cmd(UART5, ENABLE); 
		//DMA for rx
	DMA_Config(DMA1_Stream0,DMA_Channel_4, 
						 (uint32_t)&(UART5->DR),
						 (uint32_t)UART5_Rx_DMA_Buffer,
						 DMA_DIR_PeripheralToMemory,  
						 UART5_RX_BUFFER_SIZE);
	//DMA for tx					 
	DMA_Config(DMA1_Stream7,DMA_Channel_4, 
						 (uint32_t)&(UART5->DR),
						 (uint32_t)UART5_Tx_DMA_Buffer,
						 DMA_DIR_MemoryToPeripheral,
						 UART5_RX_BUFFER_SIZE);
	uart5.update=0;					 
	uart5.locked=0;
	uart5.rx_length=0;
	uart5.tx_length=0;
	uart5.rx_size=UART5_RX_BUFFER_SIZE;
	uart5.tx_size=UART5_TX_BUFFER_SIZE;
	uart5.tx_buf=UART5_Tx_Buffer;
	uart5.rx_buf=UART5_Rx_Buffer;
}
void USARTSendDatas(USART_TypeDef* USARTX,u8 *data,u8 length)
{
		int i;
    for(i=0;i<length;i++)
    {
      while(USART_GetFlagStatus(USARTX, USART_FLAG_TXE) == RESET);    
	    USART_SendData(USARTX, data[i]);    
    }
}
void USARTSendDatasByDMA(USART_TypeDef* USARTX,u8 *buf,uint16_t length)
{
	DMA_Stream_TypeDef *DMA_StreamX;
	uint32_t FLAG;
	if(USARTX==USART2)
	{
		DMA_StreamX=DMA1_Stream6;
		FLAG=DMA_FLAG_TCIF6;
		ByteArrayCopy(buf,USART2_Tx_DMA_Buffer,0,0,length);
	}
	else if(USARTX==USART3)
	{
		DMA_StreamX=DMA1_Stream3;
		FLAG=DMA_FLAG_TCIF3;
		ByteArrayCopy(buf,USART3_Tx_DMA_Buffer,0,0,length);
	}
	else if(USARTX==UART4)
	{
		DMA_StreamX=DMA1_Stream4;
		FLAG=DMA_FLAG_TCIF4;
		ByteArrayCopy(buf,UART4_Tx_DMA_Buffer,0,0,length);
	}
	else if(USARTX==UART5)
	{	
		DMA_StreamX=DMA1_Stream7;
		FLAG=DMA_FLAG_TCIF7;
		ByteArrayCopy(buf,UART5_Tx_DMA_Buffer,0,0,length);
	}
	USART_DMACmd(USARTX,USART_DMAReq_Tx,ENABLE);
	if(DMA_GetFlagStatus(DMA_StreamX,FLAG)!=RESET) //if send finish
	{ 
		DMA_ClearFlag(DMA_StreamX,FLAG);
	}		
	else
		return;
	DMA_Transfer_Enable(DMA_StreamX,length);
}
