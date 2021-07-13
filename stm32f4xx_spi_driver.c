/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: Jun 3, 2021
 *      Author: Minh Long Nguyen
 */


#include "stm32f4xx_spi_driver.h"


void SPI_Init(SPI_handler_t *pSPIHandler)
{
	//clock enable
	SPI_PeriClockControl(pSPIHandler->pSPI, ENABLE);

	uint16_t temp = 0;

	//1. Device mode
	temp |= pSPIHandler->SPIConfig.SPI_DeviceMode<<SPI_CR1_MSTR;


	//2. Bus config
	if(pSPIHandler->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//clear BIDIMODE
		temp &= ~(1<<SPI_CR1_BIDIMO);
	}
	else if(pSPIHandler->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//set BIDIMODE
		temp |= (1<<SPI_CR1_BIDIMO);
	}
	else if(pSPIHandler->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX)
	{
		//clear BIDIMODE
		temp &= ~(1<<SPI_CR1_BIDIMO);
		//set RXONLY
		temp |= (1<<SPI_CR1_RXO);
	}

	//3. Clock Speed
	temp |= (pSPIHandler->SPIConfig.SPI_SclkSpeed<<SPI_CR1_BR0_2);


	//4. Data format
	temp |= (pSPIHandler->SPIConfig.SPI_DFF<<SPI_CR1_DFF);


	//5. CPHA
	temp |= (pSPIHandler->SPIConfig.SPI_CPHA<<SPI_CR1_CPHA);


	//6. CPOL
	temp |= (pSPIHandler->SPIConfig.SPI_CPOL<<SPI_CR1_CPOL);

	//7. SSM
	temp |= (pSPIHandler->SPIConfig.SPI_SSM<<SPI_CR1_SSM);

	pSPIHandler->pSPI->CR1 |= temp;

	temp = 0;
}
void SPI_DeInit(SPI_RegPer_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_PCLK_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_PCLK_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_PCLK_RESET();
	}
}

uint8_t SPI_Flag_status(SPI_RegPer_t *pSPI, uint32_t flagName)
{
	if(!(pSPI->SR&(flagName)))
	{
		return FLAG_RESET;
	}
	else return FLAG_SET;
}
void SPI_PeriClockControl(SPI_RegPer_t *pSPIx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else if(EnOrDis == DISABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DIS();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DIS();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DIS();
		}
	}
}

void SPI_SendData(SPI_RegPer_t *pSPI, uint8_t *pTXBuffer, uint32_t len)
{
	while(len > 0)
	{
		//1. Wait until TXE is set
		while(SPI_Flag_status(pSPI,SPI_TXE_FLAG)==FLAG_RESET);

		//2. Check the DFF bit in CR1
		if(((pSPI->CR1) & (1<<SPI_CR1_DFF)) != 0)
		{
			//16 bit DFF
			//1. Load 16-bit data to the DR
			pSPI->DR = *((uint16_t*)pTXBuffer);
			len-=2;
			(uint16_t*)pTXBuffer++;
		}
		else
		{
			//8 bit DFF
			pSPI->DR = *pTXBuffer;
			len--;
			pTXBuffer++;
		}
	}
}
void SPI_ReceiveData(SPI_RegPer_t *pSPI, uint8_t *pRXBuffer, uint32_t len)
{
	while(len > 0)
	{
		//1. Wait until RXNE is set
		while(SPI_Flag_status(pSPI,SPI_RXNE_FLAG)==FLAG_RESET);

		//2. Check the DFF bit in CR1
		if(((pSPI->CR1) & (1<<SPI_CR1_DFF)) != 0)
		{
			//16 bit DFF
			//1. Load 16-bit data to the DR
			*((uint16_t*)pRXBuffer) = pSPI->DR;
			len-=2;
			(uint16_t*)pRXBuffer++;
		}
		else
		{
			//8 bit DFF
			*pRXBuffer = pSPI->DR;
			len--;
			pRXBuffer++;
		}
	}
}

uint8_t SPI_SendDataIT(SPI_handler_t *pSPIHandler, uint8_t *pTXBuffer, uint32_t len)
{
	uint8_t state = pSPIHandler->TXstate;

	if(state != SPI_BUSY_IN_TX)
	{
		//1. Save TX buffer address and len variable in global variables
		pSPIHandler->pRXBuffer = pTXBuffer;
		pSPIHandler->TXlen = len;

		//2. Set the SPI to busy state in order to prevent other code to overwhelm the SPI
		//during the transmission
		pSPIHandler->TXstate = SPI_BUSY_IN_TX;

		//3. Enable TXEIE control bit to get interrupt whenever TXE flag in SR is set
		pSPIHandler->pSPI->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}
uint8_t SPI_ReceiveDataIT(SPI_handler_t *pSPIHandler, uint8_t *pRXBuffer, uint32_t len)
{
	uint8_t state = pSPIHandler->RXstate;

	if(state != SPI_BUSY_IN_RX)
	{
		//1. Save TX buffer address and len variable in global variables
		pSPIHandler->pRXBuffer = pRXBuffer;
		pSPIHandler->RXlen = len;

		//2. Set the SPI to busy state in order to prevent other code to overwhelm the SPI
		//during the transmission
		pSPIHandler->RXstate = SPI_BUSY_IN_RX;

		//3. Enable TXEIE control bit to get interrupt whenever RXNE flag in SR is set
		pSPIHandler->pSPI->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}

void SPI_SSIcontrol(SPI_RegPer_t *pSPI, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pSPI->CR1 |= (1<<SPI_CR1_SSI);
	}
	else
	{
		pSPI->CR1 &=~ (1<<SPI_CR1_SSI);
	}
}

void SPI_SSOEcontrol(SPI_RegPer_t *pSPI, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pSPI->CR2 |= (1<<SPI_CR2_SSOE);
	}
	else
	{
		pSPI->CR2 &=~ (1<<SPI_CR2_SSOE);
	}
}

void SPI_Implementation(SPI_RegPer_t *pSPI, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pSPI->CR1 |= (1<<SPI_CR1_SPE);
	}
	else
	{
		pSPI->CR1 &=~ (1<<SPI_CR1_SPE);
	}
}

void SPI_IRQNumberConfig(uint8_t IRQNumber, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//ISER1
			*NVIC_ISER1 |= (1 << (IRQNumber%32));
		}
		else if(IRQNumber >= 64  && IRQNumber <= 95)
		{
			//ISER2
			*NVIC_ISER2 |= (1 << (IRQNumber%64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//ISER0
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//ISER1
			*NVIC_ICER1 |= (1 << (IRQNumber%32));
		}
		else if(IRQNumber >= 64  && IRQNumber <= 95)
		{
			//ISER2
			*NVIC_ICER2 |= (1 << (IRQNumber%64));
		}
	}
}
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);

static void spi_txe_it_handler(SPI_handler_t *pSPIhandler)
{
	//1. Check the DFF bit in CR1
	if(((pSPIhandler->pSPI->CR1) & (1<<SPI_CR1_DFF)) != 0)
	{
		//16 bit DFF
		//1. Load 16-bit data to the DR
		pSPIhandler->pSPI->DR = *((uint16_t*)pSPIhandler->pTXBuffer);
		pSPIhandler->TXlen-=2;
		(uint16_t*)pSPIhandler->pTXBuffer++;
	}
	else
	{
		//8 bit DFF
		pSPIhandler->pSPI->DR = *pSPIhandler->pTXBuffer;
		pSPIhandler->TXlen--;
		pSPIhandler->pTXBuffer++; //increase the TXBuffer address to point to the next data point
	}
	//2. Check if the len is 0 or not
	if(!pSPIhandler->TXlen)
	{
		//if zero, then close SPI transmission
		//a. Close the transmission
		SPI_CloseTransmission(pSPIhandler);
		//b. Inform the application about the disable
		SPI_ApplicationEventCallBack(pSPIhandler,SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_it_handler(SPI_handler_t *pSPIhandler)
{
	//1. Check the DFF bit in CR1
	if(((pSPIhandler->pSPI->CR1) & (1<<SPI_CR1_DFF)) != 0)
	{
		//16 bit DFF
		//1. Load 16-bit data from the DR
		*((uint16_t*)pSPIhandler->pRXBuffer) = pSPIhandler->pSPI->DR;
		pSPIhandler->RXlen-=2;
		(uint16_t*)pSPIhandler->pRXBuffer++;
	}
	else
	{
		//8 bit DFF
		//1. Load 8-bit data from the DR
		*pSPIhandler->pRXBuffer = pSPIhandler->pSPI->DR;
		pSPIhandler->RXlen--;
		pSPIhandler->pRXBuffer++;
	}
	//2. Check if the len is 0 or not
	if(!pSPIhandler->RXlen)
	{
		//if zero, then close SPI transmission
		//a. Close the reception
		SPI_CloseReception(pSPIhandler);
		//b. Inform the application about the disable
		SPI_ApplicationEventCallBack(pSPIhandler,SPI_EVENT_RX_CMPLT);
	}
}
static void spi_ovr_error_it_handler(SPI_handler_t *pSPIhandler)
{
	uint8_t temp;
	//1. Clear OVR flag
	if(pSPIhandler->TXstate == SPI_BUSY_IN_TX)
	{
		temp = pSPIhandler->pSPI->DR;
		temp = pSPIhandler->pSPI->SR;
	}
	(void)temp;
	//2. Inform the application
	SPI_ApplicationEventCallBack(pSPIhandler,SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_handler_t *pSPIhandler)
{
	uint8_t temp;
	temp = pSPIhandler->pSPI->DR;
	temp = pSPIhandler->pSPI->SR;
	(void)temp;
}

void SPI_CloseTransmission(SPI_handler_t *pSPIhandler)
{
	//a. Deactivate the interrupt (avoid setting TXE flag)
	//by set TXEIE to 0 in CR2 (masked)
	pSPIhandler->pSPI->CR2 &=~ (1<<SPI_CR2_TXEIE);
	//b. reset TX buffer
	pSPIhandler->pTXBuffer = NULL;
	//c. reset the length
	pSPIhandler->TXlen = 0;
	//d. change SPI from BUSY to READY
	pSPIhandler->TXstate = SPI_READY;
}
void SPI_CloseReception(SPI_handler_t *pSPIhandler)
{
	//a. Deactivate the interrupt (avoid setting RXNE flag)
	//by set RXNEIE to 0 in CR2 (masked)
	pSPIhandler->pSPI->CR2 &=~ (1<<SPI_CR2_RXNEIE);
	//b. reset RX buffer
	pSPIhandler->pRXBuffer = NULL;
	//c. reset the length
	pSPIhandler->RXlen = 0;
	//d. change SPI from BUSY to READY
	pSPIhandler->RXstate = SPI_READY;
}

void SPI_IRQHandling(SPI_handler_t *pHandler)
{
	//1. Check what cause the interrupt (in SR register)
	uint8_t temp1, temp2;
	//a. Check TXE
	temp1 = pHandler->pSPI->SR & (1 << SPI_SR_TXE);
	temp2 = pHandler->pSPI->SR & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_it_handler(pHandler);
	}

	//b. Check RXNE
	temp1 = pHandler->pSPI->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandler->pSPI->SR & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_it_handler(pHandler);
	}

	//c. Check Overrun flag (OVR)
	temp1 = pHandler->pSPI->SR & (1 << SPI_SR_OVR);
	temp2 = pHandler->pSPI->SR & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		spi_ovr_error_it_handler(pHandler);
	}

	//d.
}

__weak void SPI_ApplicationEventCallBack(SPI_handler_t *pSPIhandler, uint8_t event)
{
	//This is the weak implementation, the application may override this.
}
