/*
 * stm32f4xx.h
 *
 *  Created on: May 17, 2021
 *      Author: Minh Long Nguyen
 */

#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_
#include<stdint.h>
#include<stddef.h>
#include<stdio.h>

#define _vo volatile
#define __weak __attribute__((weak))
/*
 * base address of Flash and SRAM memories
 */
#define BASE_ADDR_FLASH			0x08000000U
#define BASE_ADDR_SRAM1			0x20000000U					//112 kB = 1C000
#define BASE_ADDR_SRAM2			(BASE_ADDR_SRAM1 + 1C000UL)
#define BASE_ADDR_SROM			0x1FFF0000U					//system memory
#define BASE_ADDR_SRAM			BASE_ADDR_SRAM1

/*
 * base address of different bus peripherals
 */
#define BASE_ADDR_AHB1			0x40020000U
#define BASE_ADDR_AHB2			0x50000000U
#define BASE_ADDR_APB1			0x40000000U
#define BASE_ADDR_APB2			0x40010000U
#define BASE_PERIPH				BASE_ADDR_APB1

/*
 * base address of AHB1 peripherals
 */
#define BASE_ADDR_GPIOA			0x40020000U
#define BASE_ADDR_GPIOB			0x40020400U
#define BASE_ADDR_GPIOC			0x40020800U
#define BASE_ADDR_GPIOD			0x40020C00U
#define BASE_ADDR_GPIOE			0x40021000U
#define BASE_ADDR_GPIOF			0x40021400U
#define BASE_ADDR_GPIOG			0x40021800U
#define BASE_ADDR_GPIOH			0x40021C00U
#define BASE_ADDR_GPIOI			0x40022000U
#define BASE_ADDR_GPIOJ			0x40022400U
#define BASE_ADDR_GPIOK			0x40022800U
#define BASE_ADDR_RCC			0x40023800U
#define BASE_ADDR_EXTI			0x40013C00U
/*
 * base address of APB1 peripherals
 */
#define BASE_ADDR_I2C1 			0x40005400U
#define BASE_ADDR_I2C2 			0x40005800U
#define BASE_ADDR_I2C3 			0x40005C00U
#define BASE_ADDR_SPI2			0x40003800U
#define BASE_ADDR_SPI3			0x40003C00U
#define BASE_ADDR_USART2			0x40004400U
#define BASE_ADDR_USART3			0x40004800U
#define BASE_ADDR_UART4			0x40004C00U
#define BASE_ADDR_UART5			0x40005000U

/*
 * base address of APB2 peripherals
 */
#define BASE_ADDR_SPI1			0x40013000U
#define BASE_ADDR_USART1			0x40011000U
#define BASE_ADDR_USART6			0x40011400U
#define BASE_ADDR_EXTI			0x40013C00U
#define BASE_ADDR_SYSCFG			0x40013800U

/*
 * base address of ISER
 */
#define NVIC_ISER0				(_vo uint32_t*)0xE000E100
#define NVIC_ISER1				(_vo uint32_t*)0xE000E104
#define NVIC_ISER2				(_vo uint32_t*)0xE000E108
#define NVIC_ISER3				(_vo uint32_t*)0xE000E10C

/*
 * base address of ICER
 */
#define NVIC_ICER0				(_vo uint32_t*)0XE000E180
#define NVIC_ICER1				(_vo uint32_t*)0XE000E184
#define NVIC_ICER2				(_vo uint32_t*)0XE000E188
#define NVIC_ICER3				(_vo uint32_t*)0XE000E18C

/*
 * base address of IPR
 */
#define BASE_ADDR_IPR			(_vo uint32_t*)0XE000E400

/*
 * base address of TIM6
 */
#define BASE_ADDR_TIM6 			0x40001000U

#define LOW_ORDER_BITS			4

/*peripheral register structure*/
/*
 * GPIOX structure
 */
typedef struct
{
	_vo uint32_t MODER;					//offset: 0x00
	_vo uint32_t OTYPER;				//offset: 0x04
	_vo uint32_t OSPEEDR;				//offset: 0x08
	_vo uint32_t PUPDR;					//offset: 0x0C
	_vo uint32_t IDR;					//offset: 0x10
	_vo uint32_t ODR;					//offset: 0x14
	_vo uint32_t BSRR;					//offset: 0x18
	_vo uint32_t LCKR;					//offset: 0x1C
	_vo uint32_t AFR[2];				//offset of 1(L): 0x20; offset of 2(H): 0x24
}GPIOx_RegPer_t;

/*
 * RCC structure
 */
typedef struct
{
	_vo uint32_t CR;
	_vo uint32_t PLLCFGR;
	_vo uint32_t CFGR;
	_vo uint32_t CIR;
	_vo uint32_t AHB1RSTR;
	_vo uint32_t AHB2RSTR;
	_vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	_vo uint32_t APB1RSTR;
	_vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	_vo uint32_t AHB1ENR;
	_vo uint32_t AHB2ENR;
	_vo uint32_t AHB3ENR;
	uint32_t RESERVED2;
	_vo uint32_t APB1ENR;
	_vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	_vo uint32_t AHB1LPENR;
	_vo uint32_t AHB2LPENR;
	_vo uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	_vo uint32_t APB1LPENR;
	_vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	_vo uint32_t BDCR;
	_vo uint32_t CSR;
	uint32_t RESERVED6[2];
	_vo uint32_t SSCGR;
	_vo uint32_t PLLI2SCFGR;
}RCC_RegPer_t;

/*
 * EXTI structure
 */
typedef struct
{
	_vo uint32_t IMR;
	_vo uint32_t EMR;
	_vo uint32_t RTSR;
	_vo uint32_t FTSR;
	_vo uint32_t SWIER;
	_vo uint32_t PR;
}EXTI_RegPer_t;

/*
 * SYSCFG structure
 */
typedef struct
{
	_vo uint32_t MEMRMP;
	_vo uint32_t PMC;
	_vo uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	_vo uint32_t CMPCR;
	uint32_t RESERVED2[2];
	_vo uint32_t CFGR;
}SYSCFG_RegPer_t;

/*
 * basic timer structure
 */
typedef struct
{
	_vo uint32_t CR1;
	_vo uint32_t CR2;
	uint32_t RESERVED1;
	_vo uint32_t DIER;
	_vo uint32_t SR;
	_vo uint32_t EGR;
	uint32_t RESERVED2[3];
	_vo uint32_t CNT;
	_vo uint32_t PSC;
	_vo uint32_t ARR;
}Ba_TIM_RegPer_t;

/*
 * SPI structure
 */
typedef struct
{
	_vo uint32_t CR1;
	_vo uint32_t CR2;
	_vo uint32_t SR;
	_vo uint32_t DR;
	_vo uint32_t CRCPR;
	_vo uint32_t RXCRCR;
	_vo uint32_t TXCRCR;
	_vo uint32_t I2SCFGR;
	_vo uint32_t I2SPR;
}SPI_RegPer_t;

/*
 * I2C structure
 */
typedef struct
{
	_vo uint32_t CR1;
	_vo uint32_t CR2;
	_vo uint32_t OAR1;
	_vo uint32_t OAR2;
	_vo uint32_t DR;
	_vo uint32_t SR1;
	_vo uint32_t SR2;
	_vo uint32_t CCR;
	_vo uint32_t TRISE;
	_vo uint32_t FLTR;
}I2C_RegPer_t;

/*
 * USART structure
 */
typedef struct
{
	_vo uint32_t SR;
	_vo uint32_t DR;
	_vo uint32_t BBR;
	_vo uint32_t CR1;
	_vo uint32_t CR2;
	_vo uint32_t CR3;
	_vo uint32_t GTPR;
}USART_RegPer_t;

/*
 * peripheral define
 */
#define GPIOA 					((GPIOx_RegPer_t*)BASE_ADDR_GPIOA)
#define GPIOB 					((GPIOx_RegPer_t*)BASE_ADDR_GPIOB)
#define GPIOC 					((GPIOx_RegPer_t*)BASE_ADDR_GPIOC)
#define GPIOD 					((GPIOx_RegPer_t*)BASE_ADDR_GPIOD)
#define GPIOE 					((GPIOx_RegPer_t*)BASE_ADDR_GPIOE)
#define GPIOF 					((GPIOx_RegPer_t*)BASE_ADDR_GPIOF)
#define GPIOG 					((GPIOx_RegPer_t*)BASE_ADDR_GPIOG)
#define GPIOH 					((GPIOx_RegPer_t*)BASE_ADDR_GPIOH)
#define GPIOI 					((GPIOx_RegPer_t*)BASE_ADDR_GPIOI)

#define RCC						((RCC_RegPer_t*)BASE_ADDR_RCC)

#define EXTI					((EXTI_RegPer_t*)BASE_ADDR_EXTI)

#define SYSCFG					((SYSCFG_RegPer_t*)BASE_ADDR_SYSCFG)

#define TIM6					((Ba_TIM_RegPer_t*)BASE_ADDR_TIM6)

#define SPI1					((SPI_RegPer_t*)BASE_ADDR_SPI1)
#define SPI2					((SPI_RegPer_t*)BASE_ADDR_SPI2)
#define SPI3					((SPI_RegPer_t*)BASE_ADDR_SPI3)

#define I2C1					((I2C_RegPer_t*)BASE_ADDR_I2C1)
#define I2C2					((I2C_RegPer_t*)BASE_ADDR_I2C2)
#define I2C3					((I2C_RegPer_t*)BASE_ADDR_I2C3)

#define USART1					((USART_RegPer_t*)BASE_ADDR_USART1)
#define USART2					((USART_RegPer_t*)BASE_ADDR_USART2)
#define USART3					((USART_RegPer_t*)BASE_ADDR_USART3)
#define USART4					((USART_RegPer_t*)BASE_ADDR_USART4)
#define USART5					((USART_RegPer_t*)BASE_ADDR_USART5)
#define USART6					((USART_RegPer_t*)BASE_ADDR_USART6)

#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()			(RCC->AHB1ENR |= (1<<8))

/*
 * Clock enable macros for I2Cx peripheral
 */
#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR |= (1<<23))

/*
 * Clock enable macros for SPIx peripheral
 */
#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1<<15))

/*
 * Clock enable macros for USARTx peripheral
 */
#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1<<18))
#define USART6_PCLK_EN()			(RCC->APB2ENR |= (1<<5))
/*
 * Clock enable macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1<<14))

/*
 * Clock enable for TIM6
 */
#define TIM6_PCLK_EN()				(RCC->APB1ENR |= (1<<4))

/*
 * Clock disable macros for GPIOx peripheral
 */
#define GPIOA_PCLK_DIS()			(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DIS()			(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DIS()			(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DIS()			(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DIS()			(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DIS()			(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DIS()			(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DIS()			(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DIS()			(RCC->AHB1ENR &= ~(1<<8))

/*
 * Reset macros for GPIOx peripheral
 */
#define GPIOA_PCLK_RESET()			do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &=~ (1<<0));}while(0)
#define GPIOB_PCLK_RESET()			do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &=~ (1<<1));}while(0)
#define GPIOC_PCLK_RESET()			do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &=~ (1<<2));}while(0)
#define GPIOD_PCLK_RESET()			do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &=~ (1<<3));}while(0)
#define GPIOE_PCLK_RESET()			do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &=~ (1<<4));}while(0)
#define GPIOF_PCLK_RESET()			do{(RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &=~ (1<<5));}while(0)
#define GPIOG_PCLK_RESET()			do{(RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &=~ (1<<6));}while(0)
#define GPIOH_PCLK_RESET()			do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &=~ (1<<7));}while(0)
#define GPIOI_PCLK_RESET()			do{(RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &=~ (1<<8));}while(0)

/*
 * Reset macros for SPIx peripheral
 */
#define SPI1_PCLK_RESET()			do{(RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &=~ (1<12));}while(0)
#define SPI2_PCLK_RESET()			do{(RCC->APB1RSTR |= (1<<14)); (RCC->APB2RSTR &=~ (1<14));}while(0)
#define SPI3_PCLK_RESET()			do{(RCC->APB1RSTR |= (1<<15)); (RCC->APB2RSTR &=~ (1<15));}while(0)

/*
 * Reset macros for SPIx peripheral
 */
#define I2C1_PCLK_RESET()			do{(RCC->APB1RSTR |= (1<<21)); (RCC->APB2RSTR &=~ (1<21));}while(0)
#define I2C2_PCLK_RESET()			do{(RCC->APB1RSTR |= (1<<22)); (RCC->APB2RSTR &=~ (1<22));}while(0)
#define I2C3_PCLK_RESET()			do{(RCC->APB1RSTR |= (1<<23)); (RCC->APB2RSTR &=~ (1<23));}while(0)

/*
 * Reset macros for USARTx peripheral
 */
#define USART1_PCLK_RESET()			do{(RCC->APB2RSTR |= (1<<4)); (RCC->APB2RSTR &=~ (1<4));}while(0)
#define USART2_PCLK_RESET()			do{(RCC->APB1RSTR |= (1<<17)); (RCC->APB2RSTR &=~ (1<17));}while(0)
#define USART3_PCLK_RESET()			do{(RCC->APB1RSTR |= (1<<18)); (RCC->APB2RSTR &=~ (1<18));}while(0)
#define USART6_PCLK_RESET()			do{(RCC->APB2RSTR |= (1<<5)); (RCC->APB2RSTR &=~ (1<5));}while(0)
/*
 * Clock disable macros for I2Cx peripheral
 */
#define I2C1_PCLK_DIS()			(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DIS()			(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DIS()			(RCC->APB1ENR &= ~(1<<22))

/*
 * Clock disable macros for SPIx peripheral
 */
#define SPI1_PCLK_DIS()			(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DIS()			(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DIS()			(RCC->APB1ENR &= ~(1<<15))

/*
 * Clock disable macros for USARTx peripheral
 */
#define USART1_PCLK_DIS()			(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DIS()			(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DIS()			(RCC->APB1ENR &= ~(1<<18))
#define USART6_PCLK_DIS()			(RCC->APB2ENR &= ~(1<<5))
/*
 * Clock disable macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DIS()			(RCC->APB2ENR &= ~(1<<14))

/*
 * Clock disable for TIM6
 */
#define TIM6_PCLK_DIS()				(RCC->APB1ENR &= ~(1<<4))

#define PORT_NUM_SYSEXTICR(x)		(	(x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
										(x == GPIOE)?4:\
										(x == GPIOF)?5:\
										(x == GPIOG)?6:\
										(x == GPIOH)?7:\
										(x == GPIOI)?8:0	)



/*
 * NVIC corresponding to EXTI
 */
#define IRQ_NUM_EXTI0		6
#define IRQ_NUM_EXTI1		7
#define IRQ_NUM_EXTI2		8
#define IRQ_NUM_EXTI3		9
#define IRQ_NUM_EXTI4		10
#define IRQ_NUM_EXTI9_5		23
#define IRQ_NUM_EXTI15_10	40

/*
 * NVIC corresponding to SPI
 */
#define IRQ_NUM_SPI1		35
#define IRQ_NUM_SPI2		36
#define IRQ_NUM_SPI3		51

/*
 * NVIC corresponding to I2C
 */
#define IRQ_NUM_I2C1_EV		31
#define IRQ_NUM_I2C1_ER		32

/*
 * NVIC corresponding to USART
 */
#define IRQ_NUM_USART1		37
#define IRQ_NUM_USART2		38
#define IRQ_NUM_USART3		39
#define IRQ_NUM_USART6		71

/*
 * Some useful macros
 */
#define ENABLE 1
#define	DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_SET ENABLE
#define GPIO_RESET DISABLE
#define FLAG_RESET	RESET
#define FLAG_SET	SET

/*
 * Bit macros SPI_CR1
 */
#define  SPI_CR1_CPHA		0
#define  SPI_CR1_CPOL		1
#define  SPI_CR1_MSTR		2
#define  SPI_CR1_BR0_2		3
#define  SPI_CR1_SPE		6
#define  SPI_CR1_LSBF		7
#define  SPI_CR1_SSI		8
#define  SPI_CR1_SSM		9
#define  SPI_CR1_RXO		10
#define  SPI_CR1_DFF		11
#define  SPI_CR1_CRCNE		12
#define  SPI_CR1_CRCEN		13
#define  SPI_CR1_BIDIOE		14
#define  SPI_CR1_BIDIMO		15

/*
 * Bit macros SPI_CR2
 */
#define SPI_CR2_SSOE		2
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7


/*
 * Bit macros SPI_SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8


/*
 * Bit macros of I2C_CR1
 */
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		14

/*
 * Bit macros of I2C_CR2
 */
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

/*
 * Bit macros of I2C_OAR1
 */
#define I2C_OAR1_ADD7_1		1

/*
 * Bit macros of I2C_CCR
 */
#define I2C_CCR_CCR11_0		0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15

/*
 * Bit macros of I2C_SR1
 */
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14

/*
 * Bit macros of I2C_SR2
 */
#define I2C_SR2_MSL			0
#define I2C_SR2_TRA			2

/*
 * Bit macros for USART_CR1
 */
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_M			12
#define USART_CR1_OVER8		15

/*
 * Bit macros for USART_CR2
 */
#define USART_CR2_STOP		12

/*
 * Bit macros for USART_CR3
 */
#define USART_CR3_RTSE		8
#define USART_CR3_CTSE		9

/*
 * Bit macros for USART_SR
 */
#define USART_SR_RXNE		5
#define USART_SR_TC			6
#define USART_SR_TXE		7


#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_tim_driver.h"
#include "stm32f4xx_spi_driver.h"
#include "stm32f4xx_i2c_driver.h"
#include "stm32f4xx_usart_driver.h"
#include "stm32f4xx_rcc_driver.h"


#endif /* INC_STM32F4XX_H_ */
