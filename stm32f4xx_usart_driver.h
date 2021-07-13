/*
 * stm32f4xx_usart_driver.h
 *
 *  Created on: Jul 11, 2021
 *      Author: Minh Long Nguyen
 */

#ifndef INC_STM32F4XX_USART_DRIVER_H_
#define INC_STM32F4XX_USART_DRIVER_H_

#include"stm32f4xx.h"

/*
 * Configuration structure of USART
 */
typedef struct
{
	uint8_t USART_Mode;
	uint8_t USART_Baud;
	uint8_t USART_Stopbits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityManage;
	uint8_t USART_HWFlowManage;
}USART_Config_t;

/*
 * Handle structure of USART
 */
typedef struct
{
	USART_RegPer_t *pUSART;
	USART_Config_t USART_Config;
}USART_handler_t;

/*
 * USART mode
 */
#define USART_MODE_ONLY_TX	0
#define USART_MODE_ONLY_RX	1
#define USART_MODE_TXRX		2

/*
 * USART baud rate
 */
#define USART_STD_BAUD_1200		1200
#define USART_STD_BAUD_2400		2400
#define USART_STD_BAUD_9600		9600
#define USART_STD_BAUD_19200	19200
#define USART_STD_BAUD_38400	38400
#define USART_STD_BAUD_57600	57600
#define USART_STD_BAUD_115200	115200
#define USART_STD_BAUD_230400	230400
#define USART_STD_BAUD_460800	460800
#define USART_STD_BAUD_921600	921600
#define USART_STD_BAUD_2M		2000000
#define USART_STD_BAUD_3M		3000000

/*
 * USART Parity control
 */
#define USART_PARITY_DISABLE	0
#define USART_PARITY_EN_EVEN	1
#define USART_PARITY_EN_ODD		2

/*
 * USART Word length
 */
#define USART_WORDLEN_8BITS		0
#define USART_WORDLEN_9BITS		1

/*
 * USART number of Stop bits
 */
#define USART_STOPBITS_1		0
#define USART_STOPBITS_0_5		1
#define USART_STOPBITS_2		2
#define USART_STOPBITS_1_5		3

/*
 * USART HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE			0
#define USART_HW_FLOW_CTRL_CTS			1
#define USART_HW_FLOW_CTRL_RTS			2
#define USART_HW_FLOW_CTRL_CTS_RTS		3

/*
 * USART flag name
 */
#define USART_FLAG_TXE		(1<<USART_SR_TXE)
#define USART_FLAG_RXNE		(1<<USART_SR_RXNE)
#define USART_FLAG_TC		(1<<USART_SR_TC)


//API for USART peripheral

void USART_Init(USART_handler_t *pUSARTHandler);
void USART_DeInit(USART_RegPer_t *pUSARTx);

void USART_PeriClockControl(USART_RegPer_t *pUSARTx, uint8_t EnOrDis);
void USART_PeripheralControl(USART_RegPer_t *pUSARTx, uint8_t EnOrDis);

void USART_SendData(USART_handler_t *pUSARTHandler, uint8_t *pTXbuffer, uint32_t len);
void USART_ReceiveData(USART_handler_t *pUSARTHandler, uint8_t *pRXbuffer, uint32_t len);

uint8_t USART_SendDataIT(USART_handler_t *pUSARTHandler, uint8_t *pTXbuffer, uint32_t len);
uint8_t USART_ReceiveDataIT(USART_handler_t *pUSARTHandler, uint8_t *pTXbuffer, uint32_t len);

uint8_t USART_Flag_status(USART_RegPer_t *pUSART, uint32_t flagName);
void USART_ClearFlag(USART_RegPer_t *pUSART, uint16_t flagname);
void USART_SetBaudRate(USART_RegPer_t *pUSART, uint32_t BaudRate);

void USART_IRQNumberConfig(uint8_t IRQNumber, uint8_t EnOrDis);
void USART_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void USART_IRQHandling(USART_handler_t *pUSARTHandler);

#endif /* INC_STM32F4XX_USART_DRIVER_H_ */
