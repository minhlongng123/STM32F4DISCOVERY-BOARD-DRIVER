/*
 * stm32f4xx_rcc_driver.c
 *
 *  Created on: Jul 12, 2021
 *      Author: Minh Long Nguyen
 */

#include "stm32f4xx_rcc_driver.h"

uint16_t AHBPrescaler[] = {2,4,8,16,64,128,256,512};
uint16_t APBPrescaler[] = {2,4,8,16};

uint32_t RCC_GetPLLOutputClock()
{

}

uint32_t RCC_GetPclk1value()
{
	uint32_t pclk1 = 0, systemClk = 0;
	uint8_t clksrc = 0, temp = 0, ahbp = 0, apbp = 0;
	clksrc = (RCC->CFGR >> 2) & (0x3);

	//1. Check the clock source
	if(clksrc == 0)
	{
		//HSI
		systemClk = 16000000;
	}
	else if(clksrc == 1)
	{
		//HSE
		systemClk = 8000000;
	}
	else
	{
		//PLL
		systemClk = RCC_GetPLLOutputClock();
	}

	//2. Check the prescaler of AHP bus
	temp = (RCC->CFGR >> 4) & (0xF);
	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHBPrescaler[temp - 8];
	}

	//3. Check the prescaler of ABP bus
	temp = (RCC->CFGR >> 10) & (0x7);
	if(temp < 4)
	{
		apbp = 1;
	}
	else
	{
		apbp = APBPrescaler[temp - 4];
	}
	pclk1 = systemClk/ahbp/apbp;
	return pclk1;
}

uint32_t RCC_GetPclk2value()
{
	uint32_t pclk2 = 0, systemClk = 0;
	uint8_t clksrc = 0, temp = 0, ahbp = 0, apbp = 0;
	clksrc = (RCC->CFGR >> 2) & (0x3);

	//1. Check the clock source
	if(clksrc == 0)
	{
		//HSI
		systemClk = 16000000;
	}
	else if(clksrc == 1)
	{
		//HSE
		systemClk = 8000000;
	}
	else
	{
		//PLL
		systemClk = RCC_GetPLLOutputClock();
	}

	//2. Check the prescaler of AHP bus
	temp = (RCC->CFGR >> 4) & (0xF);
	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHBPrescaler[temp - 8];
	}

	//3. Check the prescaler of APB bus
	temp = (RCC->CFGR >> 13) & (0x7);
	if(temp < 4)
	{
		apbp = 1;
	}
	else
	{
		apbp = APBPrescaler[temp - 4];
	}
	pclk2 = systemClk/ahbp/apbp;
	return pclk2;
}
