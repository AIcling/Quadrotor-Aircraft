#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f4xx.h"

void Motor_Init(void);
void ESC_Calibration(void);
void Set_Speed(uint8_t motor,uint16_t speed);


#endif