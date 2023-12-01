#ifndef __I2C_SOFT_H
#define __I2C_SOFT_H

#include "stm32f4xx.h"

#define I2CPORT		GPIOB		//定义IO接口
#define I2C_SCL		GPIO_Pin_8	//定义IO接口
#define I2C_SDA		GPIO_Pin_9	//定义IO接口

void I2CStart(void);
void I2CStop(void);
unsigned char I2CWaitAck(void);
void I2CSendAck(void);
void I2CSendNotAck(void);
void I2CSendByte(unsigned char cSendByte);
unsigned char I2CReceiveByte(void);
void I2CInit(void);

#endif
