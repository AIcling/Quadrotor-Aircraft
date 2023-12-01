#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "stm32f4xx.h"
#include <stdint.h>
void uart_init(void);
void USARTSendStr(USART_TypeDef* USARTx, char* str);
void PoseData_Init(void);
void Send_Qnum(void);
void Send_Elua(void);
// void Send_PPM(void);

typedef struct{
 uint8_t HEAD;
 uint8_t UADDR;
 uint8_t ID;
 uint8_t LEN;
 int16_t DATA[4];
 uint8_t FUS;
 uint8_t SUM_CHECK;
 uint8_t ADD_CHECK;
} DATA_Frame;
extern DATA_Frame MPU_Data;
#endif
