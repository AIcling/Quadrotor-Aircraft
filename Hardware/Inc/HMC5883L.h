#ifndef      __HMC5883L_H__
#define      __HMC5883L_H__

#include <stdint.h>
#define HMC5883L_ADD 0x3C
#define MPU6050_ADD	0xD0

#include "i2c_soft.h"
#include "delay.h"
#include "MPU6050.h"

void HMC5883L_Init(void);
void HMC5883L_Read(uint16_t *data);
void HMC588CL_ReadData(int16_t *data);
void Ellipsoid_Cal(void);
void Ellipsoid_Fitting(void);
extern int16_t HMC5883L_data[3];
extern float Rx,Ry,Rz,Ox,Oy,Oz;




#endif


