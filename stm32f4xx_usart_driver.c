/*
 * stm32f4xx_usart_driver.c
 *
 *  Created on: Jul 11, 2021
 *      Author: Minh Long Nguyen
 */

#include "stm32f4xx_usart_driver.h"

uint8_t USART_Flag_status(USART_RegPer_t *pUSART, uint32_t flagName)
{
	if(!(pUSART->SR&(flagName)))
	{
		return FLAG_RESET;
	}
	else return FLAG_SET;
}
void USART_ClearFlag(USART_RegPer_t *pUSART, uint16_t flagname);

void USART_Init(USART_handler_t *pUSARTHandler)
{
	uint32_t temp=0;

	//Enable the clock peripheral for USART
	USART_PeriClockControl(pUSARTHandler->pUSART, ENABLE);

	/*****************CR1*************/

	//Enable the USART TX and RX corresponding to the USART mode
	if ( pUSARTHandler->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//RX
		temp |= (1 << USART_CR1_RE);
	}
	else if (pUSARTHandler->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//TX
		temp |= ( 1 << USART_CR1_TE);

	}
	else if (pUSARTHandler->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//TX and RX
		temp |= ( ( 1 << USART_CR1_TE) | ( 1 << USART_CR1_RE) );
	}

	//Configure the Word length
	temp |= (pUSARTHandler->USART_Config.USART_WordLength) << USART_CR1_M;

    //Configuration of parity control bit fields
	if ( pUSARTHandler->USART_Config.USART_ParityManage == USART_PARITY_EN_EVEN)
	{
		//Enable the parity function
		temp |= ( 1 << USART_CR1_PCE);

		//Enable EVEN parity
		temp &=~ (1 << USART_CR1_PS);

	}
	else if (pUSARTHandler->USART_Config.USART_ParityManage == USART_PARITY_EN_ODD )
	{
		//Enable the parity control
	    temp |= ( 1 << USART_CR1_PCE);

	    //Enable ODD parity
	    temp |= (1 << USART_CR1_PS);
	}

	//Assign value in temp to CR1
	pUSARTHandler->pUSART->CR1 = temp;

	/*****************CR2*************/
	temp = 0;

	//Configure the number of stop bits
	temp |= pUSARTHandler->USART_Config.USART_Stopbits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandler->pUSART->CR2 = temp;

	/*****************CR3*************/
	temp = 0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandler->USART_Config.USART_HWFlowManage == USART_HW_FLOW_CTRL_CTS)
	{
		//Enable CTS flow control
		temp |= (1 << USART_CR3_CTSE);


	}
	else if (pUSARTHandler->USART_Config.USART_HWFlowManage == USART_HW_FLOW_CTRL_RTS)
	{
		//Enable RTS flow control
		temp |= (1 << USART_CR3_RTSE);

	}
	else if (pUSARTHandler->USART_Config.USART_HWFlowManage == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Enable both CTS and RTS Flow control
		temp |= ( (1 << USART_CR3_RTSE) | (1 << USART_CR3_CTSE) );
	}

	//Program the CR3 register
	pUSARTHandler->pUSART->CR3 = temp;

	//Baud rate calculation
	USART_SetBaudRate(pUSARTHandler->pUSART, pUSARTHandler->USART_Config.USART_Baud);
}

void USART_PeripheralControl(USART_RegPer_t *pUSARTx, uint8_t EnOrDis);

void USART_SendData(USART_handler_t *pUSARTHandler, uint8_t *pTXbuffer, uint32_t len)
{
	uint16_t *pdata;

	//The loop operates until transferred bits are over
	for(uint32_t i = 0; i < len; i++)
	{
		//wait until TXE is set
		while(!USART_Flag_status(pUSARTHandler->pUSART, USART_FLAG_TXE));

		//Check the word length
		//9 bits case
		if(pUSARTHandler->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//load 2 bytes to DR with masking the first 9 bits
			pdata = (uint16_t*)pTXbuffer;
			pUSARTHandler->pUSART->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityManage
			if(pUSARTHandler->USART_Config.USART_ParityManage == USART_PARITY_DISABLE)
			{
				//No parity is used so 9 bits so increment pTXbuffer twice, because the remaining is
				//not enough to hold the next 9 bits
				pTXbuffer++;
				pTXbuffer++;
			}
			else
			{
				//Parity bit is used, so one bit is parity bit, and the remaining is user's data
				//skip the first 8 bits to use the remaining bit for new data
				pTXbuffer++;
			}
		}
	}

	//Wait until the TC flag is set in SR (Transmission completed)
	while(!USART_Flag_status(pUSARTHandler->pUSART, USART_FLAG_TC));
}
void USART_ReceiveData(USART_handler_t *pUSARTHandler, uint8_t *pRXbuffer, uint32_t len)
{

}

uint8_t USART_SendDataIT(USART_handler_t *pUSARTHandler, uint8_t *pTXbuffer, uint32_t len);
uint8_t USART_ReceiveDataIT(USART_handler_t *pUSARTHandler, uint8_t *pTXbuffer, uint32_t len);

void USART_IRQNumberConfig(uint8_t IRQNumber, uint8_t EnOrDis);
void USART_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void USART_IRQHandling(USART_handler_t *pUSARTHandler);

void USART_DeInit(USART_RegPer_t *pUSARTx)
{
	//RCC reset register
	if(pUSARTx == USART1)
	{
		USART1_PCLK_RESET();
	}
	else if(pUSARTx == USART2)
	{
		USART2_PCLK_RESET();
	}
	else if(pUSARTx == USART3)
	{
		USART3_PCLK_RESET();
	}
	else if(pUSARTx == USART6)
	{
		USART6_PCLK_RESET();
	}
}

void USART_PeriClockControl(USART_RegPer_t *pUSARTx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
	}
	else if(EnOrDis == DISABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DIS();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DIS();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_DIS();
		}
	}
}

void USART_SetBaudRate(USART_RegPer_t *pUSART, uint32_t BaudRate)
{
	uint32_t pclkX;
	uint32_t usart_div = 0;
	uint32_t M_term, F_term;

	uint32_t temp = 0;

	if(pUSART == USART1 || pUSART == USART6)
	{
		//get the clock value on APB2 bus
		pclkX == RCC_GetPclk2value();
	}
	else
	{
		//the remaining are hold on APB1 bus
		pclkX == RCC_GetPclk1value();
	}

	//Check OVER8 bit in CR1
	if(pUSART->CR1 & (1 << USART_CR1_OVER8))
	{
		 //OVER8 = 1 , over sampling by 8
		 usart_div = ((25 * pclkX) / (2 *BaudRate));
	}
	else
	{
		 //over sampling by 16
		 usart_div = ((25 * pclkX) / (4 *BaudRate));
	}
	  //Calculate the Mantissa part
	  M_term = usart_div/100;

	  //Place the Mantissa part in appropriate bit position . refer USART_BRR
	  temp |= M_term << 4;

	  //Extract the fraction part
	  F_term = (usart_div - (M_term * 100));

	  //Calculate the final fractional
	  if(pUSART->CR1 & ( 1 << USART_CR1_OVER8))
	   {
		  //OVER8 = 1 , over sampling by 8
		  F_term = ((( F_term * 8)+ 50) / 100)& ((uint8_t)0x07);

	   }
	  else
	   {
		   //over sampling by 16
		   F_term = ((( F_term * 16)+ 50) / 100) & ((uint8_t)0x0F);
	   }

	  //Place the fractional part in appropriate bit position . refer USART_BRR
	  temp |= F_term;

	  //copy the value of tempreg in to BRR register
	  pUSART->BBR = temp;
}

