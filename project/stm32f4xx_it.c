/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "main.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
////  TimingDelay_Decrement();
//}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void) 
{
		
	uint8_t	temp = 0;
	uint8_t sbus_count = 0;
	
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET ) {   
		
		temp = USART1->SR;
		temp = USART1->DR;

		DMA_Cmd(DMA2_Stream2, DISABLE);
		DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);	// Clear Transfer Complete flag
		DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TEIF2);	// Clear Transfer error flag
		
		temp = USART1_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA2_Stream2);	
		if(temp==25)
		{
			for (sbus_count=0; sbus_count<temp; sbus_count++) 
			{	
				SBUS_DMA_buffer[sbus_count] = USART1_Rx_Buffer[sbus_count];
			}
		
			if (SBUS_DMA_buffer[0]==0x0F && SBUS_DMA_buffer[24]==0x00) {   
				
				SBUS_data[0]= ((SBUS_DMA_buffer[2]&0x07)<<8 )| SBUS_DMA_buffer[1];
				SBUS_data[1]= ((SBUS_DMA_buffer[2]&0xF8)>>3) + ((SBUS_DMA_buffer[3]&0x3F)<<5);
				SBUS_data[2]= ((SBUS_DMA_buffer[3]&0xC0)>>6)+ ((SBUS_DMA_buffer[4]&0xFF)<<2) + ((SBUS_DMA_buffer[5]&0x01)<<10);
				SBUS_data[3]= ((SBUS_DMA_buffer[5]&0xFE)>>1) + ((SBUS_DMA_buffer[6]&0x0F)<<7);
				
				SBUS_data[4]= ((SBUS_DMA_buffer[6]&0xF0)>>4) + ((SBUS_DMA_buffer[7]&0x7F)<<4);
				SBUS_data[5]= ((SBUS_DMA_buffer[7]&0x80)>>7) + ((SBUS_DMA_buffer[8]&0xFF)<<1) + ((SBUS_DMA_buffer[9]&0x03)<<9);
				SBUS_data[6]= ((SBUS_DMA_buffer[9]&0xFC)>>2) + ((SBUS_DMA_buffer[10]&0x1F)<<6) ;
				SBUS_data[7]= ((SBUS_DMA_buffer[10]&0xE0)>>5) + ((SBUS_DMA_buffer[11]&0xFF)<<3);
				
				SBUS_data[8]=  (SBUS_DMA_buffer[12]&0xFF) + ((SBUS_DMA_buffer[13]&0x07)<<8);	
				SBUS_data[9]=  ((SBUS_DMA_buffer[13]&0xF8)>>3) + ((SBUS_DMA_buffer[14]&0x3F)<<5) ;
				SBUS_data[10]= ((SBUS_DMA_buffer[14]&0xC0)>>6) + ((SBUS_DMA_buffer[15]&0xFF)<<2)+ ((SBUS_DMA_buffer[16]&0x01)<<10);
				SBUS_data[11]= ((SBUS_DMA_buffer[16]&0xFE)>>1) + ((SBUS_DMA_buffer[17]&0x0F)<<7) ;
				SBUS_data[12]= ((SBUS_DMA_buffer[17]&0xF0)>>4) + ((SBUS_DMA_buffer[18]&0x7F)<<4);
				SBUS_data[13]= ((SBUS_DMA_buffer[18]&0x80)>>7) + ((SBUS_DMA_buffer[19]&0xFF)<<1) +((SBUS_DMA_buffer[20]&0x03)<<9);
				SBUS_data[14]= ((SBUS_DMA_buffer[20]&0xFC)>>2) + ((SBUS_DMA_buffer[21]&0x1F)<<6);
				SBUS_data[15]= ((SBUS_DMA_buffer[21]&0xE0)>>5) + ((SBUS_DMA_buffer[22]&0xFF)<<3);
				
				SBUS_update=1;
			}
		}
		DMA_SetCurrDataCounter(DMA2_Stream2, USART1_RX_BUFFER_SIZE);
		DMA_Cmd(DMA2_Stream2, ENABLE);        
    }
}

/**
  * @}
  */ 
void USART2_IRQHandler(void)
{
		uint8_t rc_tmp;
		uint16_t rc_len;
		uint16_t i;
	  USART_ClearITPendingBit(USART2,USART_IT_RXNE); 
    if(USART_GetITStatus(USART2,USART_IT_IDLE)!=RESET)
    {
      rc_tmp=USART2->SR;
      rc_tmp=USART2->DR;
      DMA_Cmd(DMA1_Stream5, DISABLE);
      DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);	// Clear Transfer Complete flag
      DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TEIF5);	// Clear Transfer error flag	
      rc_len = USART2_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Stream5);
			//if usart2.buf unlocked
			if(!usart2.locked)
			{
				usart2.rx_length=rc_len;
				for(i=0;i<rc_len;i++)
				{
					usart2.rx_buf[i]=USART2_Rx_DMA_Buffer[i];
				}
				usart2.update=1;
			}
			
			DMA_SetCurrDataCounter(DMA1_Stream5, USART2_RX_BUFFER_SIZE);
			DMA_Cmd(DMA1_Stream5, ENABLE);   
		}

}
/**
  * @}
  */ 
void USART3_IRQHandler(void)
{
		uint8_t  rc_tmp;
		uint16_t rc_len;
		uint16_t i;
	  USART_ClearITPendingBit(USART3,USART_IT_RXNE); 
    if(USART_GetITStatus(USART3,USART_IT_IDLE)!=RESET)
    {
      rc_tmp=USART3->SR;
      rc_tmp=USART3->DR;
      DMA_Cmd(DMA1_Stream1, DISABLE);
      DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);	// Clear Transfer Complete flag
      DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TEIF1);	// Clear Transfer error flag	
      rc_len = USART3_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1);
			//if usart2.buf unlocked
			if(!usart3.locked)
			{
				usart3.rx_length=rc_len;
				for(i=0;i<rc_len;i++)
				{
					usart3.rx_buf[i]=USART3_Rx_DMA_Buffer[i];
				}
				usart3.update=1;
			}
			DMA_SetCurrDataCounter(DMA1_Stream1, USART3_RX_BUFFER_SIZE);
			DMA_Cmd(DMA1_Stream1, ENABLE);   
		}
}
/**
  * @}
  */ 
void UART4_IRQHandler(void)
{
		uint8_t rc_tmp;
		uint16_t rc_len;
		uint16_t i;
	  USART_ClearITPendingBit(UART4,USART_IT_RXNE); 
    if(USART_GetITStatus(UART4,USART_IT_IDLE)!=RESET)
    {
      
      rc_tmp=UART4->SR;
      rc_tmp=UART4->DR;
      DMA_Cmd(DMA1_Stream2, DISABLE);
      DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);	// Clear Transfer Complete flag
      DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TEIF2);	// Clear Transfer error flag	
      rc_len = UART4_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Stream2);
			if(!uart4.locked)
			{
				uart4.rx_length=rc_len;
				for(i=0;i<rc_len;i++)
					uart4.rx_buf[i]=UART4_Rx_DMA_Buffer[i];
				uart4.update=1;
			}
    }
		DMA_SetCurrDataCounter(DMA1_Stream2, UART4_RX_BUFFER_SIZE);
    DMA_Cmd(DMA1_Stream2, ENABLE);   
}

void UART5_IRQHandler(void)
{
		uint8_t rc_tmp;
		uint16_t rc_len;
		uint16_t i;
    if(USART_GetITStatus(UART5,USART_IT_IDLE) != RESET ) {   
		
		rc_tmp = UART5->SR;
		rc_tmp = UART5->DR;

		DMA_Cmd(DMA1_Stream0, DISABLE);
		DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0);	// Clear Transfer Complete flag
		DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TEIF0);	// Clear Transfer error flag
		rc_len = UART5_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Stream0);
		if(!uart5.locked)
		{
			uart5.rx_length=rc_len;
			for(i=0;i<rc_len;i++)
				uart5.rx_buf[i]=UART5_Rx_DMA_Buffer[i];
			uart5.update=1;
		}
	}  
	DMA_SetCurrDataCounter(DMA1_Stream0, UART5_RX_BUFFER_SIZE);
	DMA_Cmd(DMA1_Stream0, ENABLE);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
