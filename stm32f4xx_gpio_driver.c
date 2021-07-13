/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: May 18, 2021
 *      Author: Minh Long Nguyen
 */


#include "stm32f4xx_gpio_driver.h"

void GPIO_Init(GPIO_Handler_t *pGPIOHandler)
{
	//clock enable
	GPIO_PeriClockControl(pGPIOHandler->pGPIOx, ENABLE);

	uint32_t temp = 0;
	//mode configuration
	if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode <= GPIO_ModeAnalog)
	{
		//non-interrupt
		temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode) << (2*pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandler->pGPIOx->MODER &=~ (0x3 << (2*pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber)); //clear
		pGPIOHandler->pGPIOx->MODER |= temp; //set
	}
	else
	{
		//interrupt
		if(pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_ModeIT_FT)
		{
			//Clear the RTSR corresponding bit if have
			EXTI->RTSR &=~(1<<pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
			//1. EXTI_FTSR configuration
			EXTI->FTSR |=(1<<pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_ModeIT_RT)
		{
			//Clear the FTSR corresponding bit if have
			EXTI->FTSR &=~(1<<pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
			//1. EXTI_RTSR configuration
			EXTI->RTSR |=(1<<pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_ModeIT_RFT)
		{
			//1. both EXTI_FTSR and EXTI_RTSR
			EXTI->FTSR |=(1<<pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |=(1<<pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. SYSCFG_EXTICR configuration of GPIO port
		uint8_t temp1 = pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t num_port = PORT_NUM_SYSEXTICR(pGPIOHandler->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = num_port << (temp2*4);
		//3. mask register EXTI_IMR
		EXTI->IMR |=(1<<pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;
	//output type
	temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinOType) << (pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandler->pGPIOx->OTYPER &=~ (0x1 << (pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber)); //clear
	pGPIOHandler->pGPIOx->OTYPER |= temp; //set
	temp = 0;
	//output speed
	temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinSpeed) << (2*pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandler->pGPIOx->OSPEEDR &=~ (0x3 << (2*pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber)); //clear
	pGPIOHandler->pGPIOx->OSPEEDR |= temp; //set
	temp = 0;
	//PuPd
	temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinPuPd) << (2*pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandler->pGPIOx->PUPDR &=~ (0x3 << (2*pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber)); //clear
	pGPIOHandler->pGPIOx->PUPDR |= temp; //set
	temp = 0;
	//Alt
	if(pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_ModeALT)
	{
		uint32_t temp_alt1 = pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber/8;
		uint32_t temp_alt2 = pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber%8;
		temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinAltFunMode) << (4*temp_alt2);
		pGPIOHandler->pGPIOx->AFR[temp_alt1] &=~ (0xF << (temp_alt2)); //clear
		pGPIOHandler->pGPIOx->AFR[temp_alt1] |= temp; //set
	}
}
void GPIO_DeInit(GPIOx_RegPer_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_PCLK_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_PCLK_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_PCLK_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_PCLK_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_PCLK_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_PCLK_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_PCLK_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_PCLK_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_PCLK_RESET();
	}
}
void GPIO_PeriClockControl(GPIOx_RegPer_t *pGPIOx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else if(EnOrDis == DISABLE)
		{
			if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_DIS();
			}
			else if(pGPIOx == GPIOB)
			{
				GPIOB_PCLK_EN();
			}
			else if(pGPIOx == GPIOC)
			{
				GPIOC_PCLK_DIS();
			}
			else if(pGPIOx == GPIOD)
			{
				GPIOD_PCLK_DIS();
			}
			else if(pGPIOx == GPIOE)
			{
				GPIOE_PCLK_DIS();
			}
			else if(pGPIOx == GPIOF)
			{
				GPIOF_PCLK_DIS();
			}
			else if(pGPIOx == GPIOG)
			{
				GPIOG_PCLK_DIS();
			}
			else if(pGPIOx == GPIOH)
			{
				GPIOH_PCLK_DIS();
			}
			else if(pGPIOx == GPIOI)
			{
				GPIOI_PCLK_DIS();
			}
		}
}

uint8_t GPIO_ReadFromInputPin(GPIOx_RegPer_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIOx_RegPer_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}
void GPIO_WriteToOutputPin(GPIOx_RegPer_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value == SET)
	{
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else
	{
		pGPIOx->ODR &=~ (1<<PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIOx_RegPer_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR |= value;
}
void GPIO_ToggleOutputPin(GPIOx_RegPer_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}
void GPIO_IRQNumberConfig(uint8_t IRQNumber, uint8_t EnOrDis)
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	uint8_t iprx_reg = IRQNumber/4;
	uint8_t iprx_field = IRQNumber%4;
	uint8_t shift = (8*iprx_field + 8 - LOW_ORDER_BITS);
	*(BASE_ADDR_IPR + iprx_reg) |= (IRQPriority << shift);
}
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear exti pr register corresponding to the pin number
	if(EXTI->PR & (1<<PinNumber))
	{
		EXTI->PR |= (1<<PinNumber);
	}
}

