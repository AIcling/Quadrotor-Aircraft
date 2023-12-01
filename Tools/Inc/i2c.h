#ifndef __I2C_H
#define __I2C_H	 
#include "sys.h"
//宏定义变量便于更改
#define I2CPORT		GPIOB		//定义IO接口
#define I2C_SCL		GPIO_Pin_8	//定义IO接口
#define I2C_SDA		GPIO_Pin_9	//定义IO接口
#define I2C_SCL_SOURCE GPIO_PinSource8
#define I2C_SDA_SOURCE GPIO_PinSource9

#define HostAddress	0x0A	//总线主机的器件地址
#define  I2C_Speed	100000	//总线速度（不高于400000）


void I2C_Configuration(void);
void I2C_SEND_BUFFER(uint8_t SlaveAddr, uint8_t WriteAddr, uint8_t* pBuffer, uint16_t NumByteToWrite);
void I2C_SEND_BYTE(uint8_t SlaveAddr,uint8_t writeAddr,uint8_t pBuffer);
void I2C_READ_BUFFER(uint8_t SlaveAddr,uint8_t readAddr,uint8_t* pBuffer,uint16_t NumByteToRead);
uint8_t I2C_READ_BYTE(uint8_t SlaveAddr,uint8_t readAddr);
uint8_t I2C_READ_BYTE2(u8 SlaveAddr,u8 readAddr);
uint8_t MY_CHECK_EVENT(uint16_t SR1,uint16_t SR2);
#endif
