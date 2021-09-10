/*
 * stm32f4xx_tim_driver.h
 *
 *  Created on: May 30, 2021
 *      Author: Minh Long Nguyen
 */

#ifndef INC_STM32F4XX_TIM_DRIVER_H_
#define INC_STM32F4XX_TIM_DRIVER_H_

#include "stm32f4xx.h"

#define TIM_COUNTER_EN		SET
#define TIM_COUNTER_DIS		RESET
/*
 * Basic timer configuration
 */
typedef struct
{
	uint32_t Prescaler;
	uint32_t Period;
}TIM_per_config_t;

/*
 * Basic timer handler
 */
typedef struct
{
	Ba_TIM_RegPer_t *pba_tim;
	TIM_per_config_t pba_tim_config;
}Basic_tim_handler_t;


/*
 * Basic timer init
 */
void Ba_tim_init(Basic_tim_handler_t *ba_tim, uint8_t EnorDis);


#endif /* INC_STM32F4XX_TIM_DRIVER_H_ */
