/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: May 18, 2021
 *      Author: Minh Long Nguyen
 */

#ifndef INC_STM32F4XX_GPIO_DRIVER_H_
#define INC_STM32F4XX_GPIO_DRIVER_H_

#include "stm32f4xx.h"

/*
 * GPIO pin number
 */
#define GPIO_NO_0 	0
#define GPIO_NO_1 	1
#define GPIO_NO_2 	2
#define GPIO_NO_3 	3
#define GPIO_NO_4 	4
#define GPIO_NO_5 	5
#define GPIO_NO_6 	6
#define GPIO_NO_7 	7
#define GPIO_NO_8 	8
#define GPIO_NO_9 	9
#define GPIO_NO_10 	10
#define GPIO_NO_11 	11
#define GPIO_NO_12 	12
#define GPIO_NO_13 	13
#define GPIO_NO_14 	14
#define GPIO_NO_15 	15

/*
 * GPIO pin possible modes
 */
#define GPIO_ModeIN 0
#define GPIO_ModeOUT 1
#define GPIO_ModeALT 2
#define GPIO_ModeAnalog 3
#define GPIO_ModeIT_FT 4
#define GPIO_ModeIT_RT 5
#define GPIO_ModeIT_RFT 6

/*
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP 0
#define GPIO_OP_TYPE_OD 1

/*
 * GPIO pin possible output speeds
 */
#define GPIO_OP_SPEED_L 0
#define GPIO_OP_SPEED_M 1
#define GPIO_OP_SPEED_H 2
#define GPIO_OP_SPEED_VH 3

/*
 * GPIO pin pull-up and pull-down configuration
 */
#define DR_GPIO_PUPD_NONE 0
#define DR_GPIO_PUPD_PU 1
#define DR_GPIO_PUPD_PD 2
#define DR_GPIO_PUPD_RE 3

/*
 * GPIO configuration
 */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinOType;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPd;
	uint8_t GPIO_PinAltFunMode;
}GPIOx_Pin_Config_t;

/*
 * GPIO handler
 */
typedef struct
{
	//pointer to hold the base address of GPIO peripheral
	GPIOx_RegPer_t *pGPIOx;     //hold the base address of any assigned GPIOX
	GPIOx_Pin_Config_t GPIO_PinConfig;
}GPIO_Handler_t;

/************************************************API requirement***************************************/
void GPIO_Init(GPIO_Handler_t *pGPIOHandler);
void GPIO_DeInit(GPIOx_RegPer_t *pGPIOx);
void GPIO_PeriClockControl(GPIOx_RegPer_t *pGPIOx, uint8_t EnOrDis);
uint8_t GPIO_ReadFromInputPin(GPIOx_RegPer_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIOx_RegPer_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIOx_RegPer_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIOx_RegPer_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIOx_RegPer_t *pGPIOx, uint8_t PinNumber);
void GPIO_IRQNumberConfig(uint8_t IRQNumber, uint8_t EnOrDis);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);
#endif /* INC_STM32F4XX_GPIO_DRIVER_H_ */
