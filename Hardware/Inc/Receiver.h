#ifndef __RECEIVER_H__
#define __RECEIVER_H__

#include "stm32f4xx.h"
void My_Receiver_Init(void);
extern uint8_t TIM1_CapPri;
extern uint8_t TIM1_UpdPri;
extern uint16_t PPM_Data[9];
extern int ppm_idx;

#endif