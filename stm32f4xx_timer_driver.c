/*
 * stm32f4xx_tim_driver.c
 *
 *  Created on: May 30, 2021
 *      Author: Minh Long Nguyen
 */


#include "stm32f4xx_tim_driver.h"

void Ba_tim_init(Basic_tim_handler_t *ba_tim, uint8_t EnorDis)
{
	ba_tim->pba_tim->ARR = ba_tim->pba_tim_config.Period;
	ba_tim->pba_tim->PSC = ba_tim->pba_tim_config.Prescaler;
	ba_tim->pba_tim->CR1 |= (EnorDis << 0);
}

