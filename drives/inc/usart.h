#ifndef __USART_H
#define __USART_H
#include "sys.h"	

#define USART1_TX_BUFFER_SIZE     50
#define USART1_RX_BUFFER_SIZE     50

#define	USART2_TX_BUFFER_SIZE    	1024
#define	USART2_RX_BUFFER_SIZE    	1024

#define USART3_TX_BUFFER_SIZE		  1024
#define USART3_RX_BUFFER_SIZE     1024

#define	UART4_RX_BUFFER_SIZE    	1024
#define	UART4_TX_BUFFER_SIZE    	1024

#define	UART5_RX_BUFFER_SIZE    	1024
#define	UART5_TX_BUFFER_SIZE    	1024

//define struct for usart receive and transfer.
typedef struct
{
	uint8_t update;
	uint8_t locked;
	uint8_t rx_length;
	uint8_t tx_length;
	uint16_t tx_size;
	uint16_t rx_size; 
	uint8_t *tx_buf;
	uint8_t *rx_buf;
}usart_rt;

//extern uint8_t	USART1_Tx_buffer[USART1_TX_BUFFER_SIZE] = {0};
extern uint8_t	USART1_Rx_Buffer[USART1_RX_BUFFER_SIZE];

extern uint8_t 	USART2_Rx_Buffer[USART2_RX_BUFFER_SIZE];
extern uint8_t 	USART2_Tx_Buffer[USART2_RX_BUFFER_SIZE];
extern uint8_t 	USART2_Rx_DMA_Buffer[USART2_RX_BUFFER_SIZE];
extern uint8_t 	USART2_Tx_DMA_Buffer[USART2_RX_BUFFER_SIZE];

extern uint8_t	USART3_Rx_Buffer[USART3_RX_BUFFER_SIZE];
extern uint8_t	USART3_Tx_Buffer[USART3_RX_BUFFER_SIZE];
extern uint8_t	USART3_Rx_DMA_Buffer[USART3_RX_BUFFER_SIZE];
extern uint8_t	USART3_Tx_DMA_Buffer[USART3_RX_BUFFER_SIZE];

extern uint8_t	UART4_Tx_Buffer[UART4_TX_BUFFER_SIZE];
extern uint8_t	UART4_Rx_Buffer[UART4_RX_BUFFER_SIZE];
extern uint8_t	UART4_Tx_DMA_Buffer[UART4_TX_BUFFER_SIZE];
extern uint8_t	UART4_Rx_DMA_Buffer[UART4_RX_BUFFER_SIZE];

extern uint8_t	UART5_Tx_Buffer[UART5_RX_BUFFER_SIZE];
extern uint8_t	UART5_Rx_Buffef[UART5_RX_BUFFER_SIZE];
extern uint8_t	UART5_Tx_DMA_Buffer[UART5_RX_BUFFER_SIZE];
extern uint8_t	UART5_Rx_DMA_Buffer[UART5_RX_BUFFER_SIZE];

extern usart_rt usart1,usart2,usart3,uart4,uart5;

void USART1_Init(uint32_t bound);
void USART2_Init(uint32_t bound);
void USART3_Init(uint32_t bound);
void UART4_Init(uint32_t bound);
void UART5_Init(uint32_t bound);

void USARTSendDatas(USART_TypeDef* USARTX,u8 *data,u8 length);
void USARTSendDatasByDMA(USART_TypeDef* USARTX,u8 *buf,uint16_t length);
	
#endif


