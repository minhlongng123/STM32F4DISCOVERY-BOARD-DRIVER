/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: Jun 3, 2021
 *      Author: Minh Long Nguyen
 */

#ifndef INC_STM32F4XX_SPI_DRIVER_H_
#define INC_STM32F4XX_SPI_DRIVER_H_

#include "stm32f4xx.h"

/*
 * SPI Device Mode
 */
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0

/*
 * SPI Bus configuration
 */
#define SPI_BUS_CONFIG_FD			1
#define SPI_BUS_CONFIG_HD			2
#define SPI_BUS_CONFIG_SIMPLEX_RX	3

/*
 * SPI clock speed
 */
#define SPI_BUS_SPEED_DIV2			0
#define SPI_BUS_SPEED_DIV4			1
#define SPI_BUS_SPEED_DIV8			2
#define SPI_BUS_SPEED_DIV16			3
#define SPI_BUS_SPEED_DIV32			4
#define SPI_BUS_SPEED_DIV64			5
#define SPI_BUS_SPEED_DIV128		6
#define SPI_BUS_SPEED_DIV256		7

/*
 * SPI Data Frame Format
 */
#define SPI_DFF_8BITS	0
#define SPI_DFF_16BITS	1

/*
 * SPI CPHA
 */
#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0

/*
 * SPI CPOL
 */
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

/*
 * SPI SSM
 */
#define SPI_SSM_DIS		0
#define SPI_SSM_EN		1

/*
 * SPI SSI
 */
#define SPI_SSI_DIS		0
#define SPI_SSI_EN		1

/*
 * Flag
 */
#define SPI_TXE_FLAG	(1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG	(1<<SPI_SR_RXNE)
#define SPI_BUSY_FLAG	(1<<SPI_SR_BSY)

/*
 * SPI application state
 */
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

/*
 * SPI application event
 */
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3

/*
 * SPI configuration
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
	uint8_t SPI_SSI;
}SPI_per_config_t;

/*
 * SPI handler
 */
typedef struct
{
	SPI_RegPer_t *pSPI;
	SPI_per_config_t SPIConfig;
	uint8_t *pTXBuffer;				//To store TX buffer address
	uint8_t *pRXBuffer;				//To store RX buffer address
	uint32_t TXlen;					//To store TX len
	uint32_t RXlen;					//To store RX len
	uint32_t TXstate;				//To store TX state
	uint32_t RXstate;				//To store RX state
}SPI_handler_t;

uint8_t SPI_Flag_status(SPI_RegPer_t *pSPI, uint32_t flagName);

void SPI_Init(SPI_handler_t *pSPIHandler);
void SPI_DeInit(SPI_RegPer_t *pSPIx);
void SPI_PeriClockControl(SPI_RegPer_t *pSPIx, uint8_t EnOrDis);

void SPI_SendData(SPI_RegPer_t *pSPIx, uint8_t *pTXBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegPer_t *pSPIx, uint8_t *pRXBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_handler_t *pSPIHandler, uint8_t *pTXBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_handler_t *pSPIHandler, uint8_t *pRXBuffer, uint32_t len);

void SPI_IRQNumberConfig(uint8_t IRQNumber, uint8_t EnOrDis);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_handler_t *pHandler);
void SPI_SSIcontrol(SPI_RegPer_t *pSPI, uint8_t EnOrDis);
void SPI_SSOEcontrol(SPI_RegPer_t *pSPI, uint8_t EnOrDis);
void SPI_Implementation(SPI_RegPer_t *pSPI, uint8_t EnOrDis);
void SPI_ClearOVRFlag(SPI_handler_t *pSPIhandler);
void SPI_CloseTransmission(SPI_handler_t *pSPIhandler);
void SPI_CloseReception(SPI_handler_t *pSPIhandler);

//Application callback
void SPI_ApplicationEventCallBack(SPI_handler_t *pSPIhandler, uint8_t event);

#endif /* INC_STM32F4XX_SPI_DRIVER_H_ */
