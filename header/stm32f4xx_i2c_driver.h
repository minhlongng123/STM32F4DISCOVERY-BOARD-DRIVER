/*
 * stm32f4xx_i2c_driver.h
 *
 *  Created on: Jun 30, 2021
 *      Author: Minh Long Nguyen
 */

#include "stm32f4xx.h"

#ifndef INC_STM32F4XX_I2C_DRIVER_H_
#define INC_STM32F4XX_I2C_DRIVER_H_

/*
 * Flag
 */
#define I2C_FLAG_SB		(1<<I2C_SR1_SB)
#define I2C_FLAG_BTF	(1<<I2C_SR1_BTF)
#define I2C_FLAG_TXE	(1<<I2C_SR1_TXE)
#define I2C_FLAG_RXNE	(1<<I2C_SR1_RXNE)
#define I2C_FLAG_ADDR	(1<<I2C_SR1_ADDR)

/*
 * Configuration structure
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handling structure
 */
typedef struct
{
	I2C_RegPer_t *pI2C;
	I2C_Config_t I2C_Config;
	//variable used to store data in interrupt mode
	uint8_t *pTXbuffer;			//address of the TX buffer
	uint8_t *pRXbuffer;			//address of the RX buffer
	uint32_t TXlen;				//length of TX buffer
	uint32_t RXlen;				//length of RX buffer
	uint8_t TXRXStatus;			//status of I2C application
	uint8_t DevAddr;			//device address
	uint32_t RXsize;			//size of RX buffer
	uint8_t Rs;					//repeated start
}I2C_handler_t;

/*
 * I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM   	100000
#define I2C_SCL_SPEED_FM2K 	200000
#define I2C_SCL_SPEED_FM4K 	400000

/*
 * I2C_ACKControl
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*
 * I2C repeated start
 */
#define I2C_RS_EN	1
#define I2C_RS_DIS	0

/*
 * Application status
 */
#define I2C_READY			0
#define I2C_BUSY_IN_RX		1
#define I2C_BUSY_IN_TX		2

/*
 * I2C application event
 */
#define I2C_EV_TX_CMPLT		0
#define I2C_EV_RX_CMPLT		1
#define I2C_EV_TX_STOP		2
#define I2C_EV_RX_STOP		3
#define I2C_EV_DATA_REQ		4
#define I2C_EV_DATA_RCV		5
#define I2C_ERROR_AF		6

//API for I2C peripheral

void I2C_Init(I2C_handler_t *pI2CHandler);
void I2C_DeInit(I2C_RegPer_t *pI2Cx);
void I2C_PeriClockControl(I2C_RegPer_t *pI2Cx, uint8_t EnOrDis);
void I2C_PeripheralControl(I2C_RegPer_t *pI2Cx, uint8_t EnOrDis);

void I2C_MasterSendData(I2C_handler_t *pI2CHandler, uint8_t *pTXBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Rs);
void I2C_MasterReceiveData(I2C_handler_t *pI2CHandler, uint8_t *pRXBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Rs);

uint8_t I2C_MasterSendDataIT(I2C_handler_t *pI2CHandler, uint8_t *pTXBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Rs);
uint8_t I2C_MasterReceiveDataIT(I2C_handler_t *pI2CHandler, uint8_t *pRXBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Rs);

void I2C_SlaveSendData(I2C_RegPer_t *pI2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegPer_t *pI2C);

void I2C_CloseTransmission(I2C_handler_t *pI2CHandler);
void I2C_CloseReception(I2C_handler_t *pI2CHandler);

void I2C_IRQNumberConfig(uint8_t IRQNumber, uint8_t EnOrDis);
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_handler_t *pI2CHandler);
void I2C_ER_IRQHandling(I2C_handler_t *pI2CHandler);

uint8_t I2C_Flag_status(I2C_RegPer_t *pI2C, uint32_t flagName);

void I2C_Implementation(I2C_RegPer_t *pI2C, uint8_t EnOrDis);
void I2C_CloseTransmission(I2C_handler_t *pI2Chandler);
void I2C_CloseReception(I2C_handler_t *pI2Chandler);
void I2C_ACK_control(I2C_RegPer_t *pI2C, uint8_t EnOrDis);
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegPer_t *pI2C, uint8_t EnOrDis);

//Application callback
void I2C_ApplicationEventCallBack(I2C_handler_t *pI2Chandler, uint8_t event);


#endif /* INC_STM32F4XX_I2C_DRIVER_H_ */
