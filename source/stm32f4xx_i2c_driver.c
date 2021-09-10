/*
 * stm32f4xx_i2c_driver.c
 *
 *  Created on: Jun 30, 2021
 *      Author: Minh Long Nguyen
 */


#include "stm32f4xx_i2c_driver.h"


static void I2C_GenerateStartCondition(I2C_RegPer_t *pI2C)
{
	pI2C->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegPer_t *pI2C,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr<<1;	//give space for r/w bit
	SlaveAddr &=~ (1);			//assign 0 to the first bit of SlaveAddr (LSB) for writing
	pI2C->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegPer_t *pI2C,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;				//assign 1 to the first bit of SlaveAddr (LSB) for reading
	pI2C->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_handler_t *pI2Chandler)
{
	uint32_t dummy_read;
	//check the device mode
	if(pI2Chandler->pI2C->SR2 & (1<I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2Chandler->TXRXStatus == I2C_BUSY_IN_RX)
		{
			if(pI2Chandler->RXsize == 1)
			{
				//1. disable ACK
				I2C_ACK_control(pI2Chandler->pI2C, DISABLE);

				//2. Clear ADDR flag
				dummy_read = pI2Chandler->pI2C->SR1;
				dummy_read = pI2Chandler->pI2C->SR2;
				(void)dummy_read;
			}
		}
		else
		{
			//device is in slave mode
			//Clear ADDR flag
			dummy_read = pI2Chandler->pI2C->SR1;
			dummy_read = pI2Chandler->pI2C->SR2;
			(void)dummy_read;
		}
	}
	else
	{
		//device is in slave mode
		//Clear ADDR flag
		dummy_read = pI2Chandler->pI2C->SR1;
		dummy_read = pI2Chandler->pI2C->SR2;
		(void)dummy_read;
	}
}

static void I2C_GenerateStopCondition(I2C_RegPer_t *pI2C)
{
	pI2C->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_MasterTXInterruptHandling(I2C_handler_t *pI2CHandler)
{
	if(pI2CHandler->TXRXStatus == I2C_BUSY_IN_TX)
	{
		if(pI2CHandler->TXlen > 0)
		{
			//1. Load data into DR
			pI2CHandler->pI2C->DR = *(pI2CHandler->pTXbuffer);

			//2. Decrement TX len
			pI2CHandler->TXlen--;

			//3. Increment the buffer address
			pI2CHandler->pTXbuffer++;
		}
	}
}

static void I2C_MasterRXInterruptHandling(I2C_handler_t *pI2CHandler)
{
	if(pI2CHandler->RXsize == 1)
	{
		*pI2CHandler->pRXbuffer = pI2CHandler->pI2C->DR;
		pI2CHandler->RXlen--;
	}
	else if(pI2CHandler->RXsize > 1)
	{
		if(pI2CHandler->RXlen == 2)
		{
			//1. disable ACK
			I2C_ACK_control(pI2CHandler->pI2C, DISABLE);
		}
			//2. read DR
			*pI2CHandler->pRXbuffer = pI2CHandler->pI2C->DR;
			pI2CHandler->RXlen--;
			pI2CHandler->pRXbuffer++;
	}
	if(pI2CHandler->RXlen == 0)
	{
		//Close the reception and notify the application

		//1. Generate the stop condition
		if(pI2CHandler->Rs == I2C_RS_DIS)
		{
			I2C_GenerateStopCondition(pI2CHandler->pI2C);
		}
		//2. Close the I2C RX
		I2C_CloseReception(pI2CHandler);

		//3. Notify the application
		I2C_ApplicationEventCallBack(pI2CHandler, I2C_EV_RX_STOP);
	}
}


void I2C_ACK_control(I2C_RegPer_t *pI2C, uint8_t EnOrDis)
{
	if(EnOrDis == I2C_ACK_ENABLE)
	{
		//enable ACK
		pI2C->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		//disable ACK
		pI2C->CR1 &=~ (1 << I2C_CR1_ACK);
	}
}


uint8_t I2C_Flag_status(I2C_RegPer_t *pI2C, uint32_t flagName)
{
	if(!(pI2C->SR1&(flagName)))
	{
		return FLAG_RESET;
	}
	else return FLAG_SET;
}
void I2C_PeripheralControl(I2C_RegPer_t *pI2Cx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pI2Cx->CR1 |= (1<<I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &=~ (1<<I2C_CR1_PE);
	}
}

void I2C_Init(I2C_handler_t *pI2CHandler)
{
	//Clock enable
	I2C_PeriClockControl(pI2CHandler->pI2C, ENABLE);

	uint32_t temp = 0;

	//1. ACK control
	temp |= (pI2CHandler->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	pI2CHandler->pI2C->CR1 = temp;

	//2. configure the FREQ of CR2
	temp = 0;
	temp = RCC_GetPclk1value()/1000000U;
	pI2CHandler->pI2C->CR2 = (temp & 0x3F);

	//3. device address (if device is slave)
	temp = 0;
	temp |= (pI2CHandler->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD7_1);
	temp |= (1<<14);
	pI2CHandler->pI2C->OAR1 = temp;

	//4. CCR calculations
	uint16_t ccr_value = 0;
	temp = 0;
	if(pI2CHandler->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//standard mode
		ccr_value = (RCC_GetPclk1value()/(2*pI2CHandler->I2C_Config.I2C_SCLSpeed));
		temp |= (ccr_value & 0xFFF);
	}
	else
	{
		//fast mode
		temp |= (1<<I2C_CCR_FS); //set mode
		temp |= (pI2CHandler->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY); //set duty
		if(pI2CHandler->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPclk1value()/(3*pI2CHandler->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccr_value = (RCC_GetPclk1value()/(25*pI2CHandler->I2C_Config.I2C_SCLSpeed));
		}
		temp |= (ccr_value & 0xFFF);
	}
	pI2CHandler->pI2C->CCR = temp;
	temp = 0;
	//5. Trise calculation
	if(pI2CHandler->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//standard mode
		temp = RCC_GetPclk1value()/1000000U + 1;
	}
	else
	{
		//fast mode
		temp = RCC_GetPclk1value()*300/1000000000U + 1;
	}
	pI2CHandler->pI2C->TRISE = (temp & 0x3F);
}
void I2C_DeInit(I2C_RegPer_t *pI2Cx)
{
	//RCC reset register
	if(pI2Cx == I2C1)
	{
		I2C1_PCLK_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_PCLK_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_PCLK_RESET();
	}
}
void I2C_PeriClockControl(I2C_RegPer_t *pI2Cx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else if(EnOrDis == DISABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DIS();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DIS();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DIS();
		}
	}
}

void I2C_MasterSendData(I2C_handler_t *pI2CHandler, uint8_t *pTXBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Rs)
{
	//1. Generate the start condition
	I2C_GenerateStartCondition(pI2CHandler->pI2C);

	//2. Confirm the start generation is completed by checking the SB flag in SR1
	while(!I2C_Flag_status(pI2CHandler->pI2C,I2C_FLAG_SB));

	//3. Send the address of the slave and r/w bit
	I2C_ExecuteAddressPhaseWrite(pI2CHandler->pI2C,SlaveAddr);

	//4. Confirm that the address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_Flag_status(pI2CHandler->pI2C,I2C_FLAG_ADDR));

	//5. Clear ADDR Flag
	//by reading SR1 followed by SR2
	I2C_ClearADDRFlag(pI2CHandler);

	//6. send data until the length is 0 (len = 0)
	while(len>0)
	{
		while(!I2C_Flag_status(pI2CHandler->pI2C,I2C_FLAG_TXE));
		pI2CHandler->pI2C->DR = *pTXBuffer;
		pTXBuffer++;
		len--;
	}

	//7. when len = 0, wait for TXE = 1, BTF = 1 before generating the STOP condition
	while(!I2C_Flag_status(pI2CHandler->pI2C,I2C_FLAG_TXE));
	while(!I2C_Flag_status(pI2CHandler->pI2C,I2C_FLAG_BTF));

	//8. Generate the stop condition
	if(Rs == I2C_RS_DIS)
	{
		I2C_GenerateStopCondition(pI2CHandler->pI2C);
	}
}
void I2C_MasterReceiveData(I2C_handler_t *pI2CHandler, uint8_t *pRXBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Rs)
{
	//1. Generate the start condition
	I2C_GenerateStartCondition(pI2CHandler->pI2C);

	//2. Confirm that start generation is completed by checking the SB flag in SR1
	// need to clear SB after that
	while(!I2C_Flag_status(pI2CHandler->pI2C,I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandler->pI2C,SlaveAddr);

	//4. Wait until address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_Flag_status(pI2CHandler->pI2C,I2C_FLAG_ADDR));

	//a. Read only 1 byte from  slave
	if(len == 1)
	{
		//1. Disable ACK
		I2C_ACK_control(pI2CHandler->pI2C, I2C_ACK_ENABLE);

		//2. Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandler);

		//3. Wait until RXNE is set
		while(!I2C_Flag_status(pI2CHandler->pI2C,I2C_FLAG_RXNE));

		//4. generate stop condition
		I2C_GenerateStopCondition(pI2CHandler->pI2C);

		//5. read data into buffer
		*pRXBuffer = pI2CHandler->pI2C->DR;
	}

	//b. Read more than 1 byte from slave
	if(len > 1)
	{
		//1. clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandler);

		//2. Read the data until len = 0
		for(uint32_t i = len; i > 0; i--)
		{
			//wait until RXNE is set
			while(!I2C_Flag_status(pI2CHandler->pI2C,I2C_FLAG_RXNE));

			if(i ==	 2)	//the remaining 2 bytes
			{
				//disable ACK
				I2C_ACK_control(pI2CHandler->pI2C, I2C_ACK_ENABLE);

				//generate the stop condition
				if(Rs == I2C_RS_DIS)
				{
					I2C_GenerateStopCondition(pI2CHandler->pI2C);
				}
			}

			//read data from DR reg
			*pRXBuffer = pI2CHandler->pI2C->DR;

			//increment the buffer address
			pRXBuffer++;
		}
	}
	//re-enable the ACK
	if(pI2CHandler->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ACK_control(pI2CHandler->pI2C, ENABLE);
	}
}

uint8_t I2C_MasterSendDataIT(I2C_handler_t *pI2CHandler, uint8_t *pTXBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Rs)
{
	uint8_t busystate = pI2CHandler->TXRXStatus;
	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandler->pTXbuffer = pTXBuffer;
		pI2CHandler->TXlen = len;
		pI2CHandler->TXRXStatus = I2C_BUSY_IN_TX;
		pI2CHandler->DevAddr = SlaveAddr;
		pI2CHandler->Rs = Rs;

		//Generate the start condition
		I2C_GenerateStartCondition(pI2CHandler->pI2C);

		//Enable ITBUFEN control bit in I2C_CR2 to enable the event interrupt when TXE or RXNE = 1
		pI2CHandler->pI2C->CR2 = (1<<I2C_CR2_ITBUFEN);

		//Enable ITEVTEN control bit in I2C_CR2 to enable the event interrupt
		pI2CHandler->pI2C->CR2 = (1<<I2C_CR2_ITEVTEN);

		//Enable ITERREN control bit in I2C_CR2 to enable the error interrupt
		pI2CHandler->pI2C->CR2 = (1<<I2C_CR2_ITERREN);
	}
	return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_handler_t *pI2CHandler, uint8_t *pRXBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Rs)
{
	uint8_t busystate = pI2CHandler->TXRXStatus;
	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandler->pRXbuffer = pRXBuffer;
		pI2CHandler->RXlen = len;
		pI2CHandler->TXRXStatus = I2C_BUSY_IN_RX;
		pI2CHandler->RXsize = len;
		pI2CHandler->DevAddr = SlaveAddr;
		pI2CHandler->Rs = Rs;

		//Generate the start condition
		I2C_GenerateStartCondition(pI2CHandler->pI2C);

		//Enable ITBUFEN control bit in I2C_CR2 to enable the event interrupt when TXE or RXNE = 1
		pI2CHandler->pI2C->CR2 = (1<<I2C_CR2_ITBUFEN);

		//Enable ITEVTEN control bit in I2C_CR2 to enable the event interrupt
		pI2CHandler->pI2C->CR2 = (1<<I2C_CR2_ITEVTEN);

		//Enable ITERREN control bit in I2C_CR2 to enable the error interrupt
		pI2CHandler->pI2C->CR2 = (1<<I2C_CR2_ITERREN);
	}
	return busystate;
}

void I2C_SlaveSendData(I2C_RegPer_t *pI2C, uint8_t data)
{
	pI2C->DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegPer_t *pI2C)
{
	return pI2C->DR;
}

void I2C_IRQNumberConfig(uint8_t IRQNumber, uint8_t EnOrDis)
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{

}

void I2C_CloseTransmission(I2C_handler_t *pI2CHandler)
{
	//1. disable the interrupt (ITBUFEN in CR2)
	pI2CHandler->pI2C->CR2 &=~ (1<<I2C_CR2_ITBUFEN);

	//2. disable the event interrupt (ITEVTEN)
	pI2CHandler->pI2C->CR2 &=~ (1<<I2C_CR2_ITEVTEN);

	//3. Reset the variable in the Config structure
	pI2CHandler->TXRXStatus = I2C_READY;
	pI2CHandler->pTXbuffer = NULL;
	pI2CHandler->TXlen = 0;
}
void I2C_CloseReception(I2C_handler_t *pI2CHandler)
{
	//1. disable the interrupt (ITBUFEN in CR2)
	pI2CHandler->pI2C->CR2 &=~ (1<<I2C_CR2_ITBUFEN);

	//2. disable the event interrupt (ITEVTEN)
	pI2CHandler->pI2C->CR2 &=~ (1<<I2C_CR2_ITEVTEN);

	//3. Reset the variable in the Config structure
	pI2CHandler->TXRXStatus = I2C_READY;
	pI2CHandler->pRXbuffer = NULL;
	pI2CHandler->RXlen = 0;
	pI2CHandler->RXsize = 0;
	if(pI2CHandler->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ACK_control(pI2CHandler->pI2C, ENABLE);
	}
}

void I2C_EV_IRQHandling(I2C_handler_t *pI2CHandler)
{
	//Interrupt handling for device in both master and slave mode
	uint8_t temp1 = 0, temp2 = 0, temp3 = 0;
	temp1 = pI2CHandler->pI2C->CR2 & (1<<I2C_CR2_ITEVTEN);
	temp2 = pI2CHandler->pI2C->CR2 & (1<<I2C_CR2_ITBUFEN);

	//1. Handle the interrupt by SB event (master mode) (SB is always 0 in slave mode)
	temp3 = pI2CHandler->pI2C->SR1 & (1<<I2C_SR1_SB);
	if(temp1 && temp3)
	{
		//SB flag is set
		if(pI2CHandler->TXRXStatus == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandler->pI2C, pI2CHandler->DevAddr);
		}
		else if(pI2CHandler->TXRXStatus == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandler->pI2C, pI2CHandler->DevAddr);
		}
	}

	//2. Handle the interrupt by ADDR event
	temp3 = 0;
	temp3 = pI2CHandler->pI2C->SR1 & (1<<I2C_SR1_ADDR);
	if(temp1 && temp3)
	{
		//ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandler);
	}

	//3. Handle the interrupt by BTF event
	temp3 = 0;
	temp3 = pI2CHandler->pI2C->SR1 & (1<<I2C_SR1_BTF);
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandler->TXRXStatus == I2C_BUSY_IN_TX)
		{
			//check if TXE = 1 or not
			if(pI2CHandler->pI2C->SR1 & (1 << I2C_SR1_TXE))
			{
				//BTF = 1, TXE = 1 => close the transmission
				if(pI2CHandler->TXlen == 0)
				{
					//1. Generate the stop condition
					if(pI2CHandler->Rs == I2C_RS_DIS)
					{
						I2C_GenerateStopCondition(pI2CHandler->pI2C);
					}
					//2. Reset all the variable in the handler structure in the header file
					I2C_CloseTransmission(pI2CHandler);

					//3. Notify the application about the transmission completed
					I2C_ApplicationEventCallBack(pI2CHandler, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if(pI2CHandler->TXRXStatus == I2C_BUSY_IN_RX)
		{
			//check if RXNE = 1 or not
			;
		}
	}

	//4. Handle the interrupt by STOPF event
	//Note: Only applicable in slave mode (receive data)
	temp3 = 0;
	temp3 = pI2CHandler->pI2C->SR1 & (1<<I2C_SR1_STOPF);
	if(temp1 && temp3)
	{
		//STOPF flag is set
		pI2CHandler->pI2C->CR1 |= 0x0000;

		//Notify the application that the stop condition is detected
		I2C_ApplicationEventCallBack(pI2CHandler, I2C_EV_TX_STOP);
	}

	//5. Handle the interrupt by TXE event
	temp3 = 0;
	temp3 = pI2CHandler->pI2C->SR1 & (1<<I2C_SR1_TXE);
	if(temp1 && temp2 && temp3)
	{
		//TXE flag is set
		if(pI2CHandler->pI2C->SR2 & (1<<I2C_SR2_MSL)) //check if it is in master mode or not
		{
			I2C_MasterTXInterruptHandling(pI2CHandler);
		}
		else
		{
			//slave
			if(pI2CHandler->pI2C->SR2 & (1<<I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallBack(pI2CHandler, I2C_EV_DATA_REQ);
			}
		}
	}

	//6. Handle the interrupt by RXNE event
	temp3 = 0;
	temp3 = pI2CHandler->pI2C->SR1 & (1<<I2C_SR1_RXNE);
	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandler->pI2C->SR2 & (1<<I2C_SR2_MSL)) //Check the device mode (master mode)
		{
			//RXNE flag is set
			if(pI2CHandler->TXRXStatus == I2C_BUSY_IN_RX)
			{
				I2C_MasterRXInterruptHandling(pI2CHandler);
			}
		}
		else
		{
			if(!(pI2CHandler->pI2C->SR2 & (1<<I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallBack(pI2CHandler, I2C_EV_DATA_RCV);
			}
		}
	}
}
void I2C_ER_IRQHandling(I2C_handler_t *pI2CHandler)
{
	uint32_t temp1, temp2;

	//Know the status of ITERREN in CR2
	temp2 = (pI2CHandler->pI2C->CR2) & (1<<I2C_CR2_ITERREN);

	temp1 = (pI2CHandler->pI2C->SR1) & (1<<I2C_SR1_BERR);
	if(temp1 && temp2)
	{
		//Bus error

		//Clear the bus error

		//Notify the application
	}
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegPer_t *pI2C, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pI2C->CR2 |= (1<<I2C_CR2_ITEVTEN);
		pI2C->CR2 |= (1<<I2C_CR2_ITBUFEN);
		pI2C->CR2 |= (1<<I2C_CR2_ITERREN);
	}
	else
	{
		pI2C->CR2 &=~ (1<<I2C_CR2_ITEVTEN);
		pI2C->CR2 &=~ (1<<I2C_CR2_ITBUFEN);
		pI2C->CR2 &=~ (1<<I2C_CR2_ITERREN);
	}
}


__weak void I2C_ApplicationEventCallBack(I2C_handler_t *pI2Chandler, uint8_t event);
