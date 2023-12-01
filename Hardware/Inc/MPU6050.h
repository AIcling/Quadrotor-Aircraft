#ifndef      __MPU6050_H__
#define      __MPU6050_H__

#include "sys.h"
#include "i2c.h"
#include "delay.h"


#define MPU6050_ADD	0xD0	//器件地址（AD0悬空或低电平时地址是0xD0，为高电平时为0xD2，7位地址：1101 000x）

#define MPU6050_RA_SMPLRT_DIV       0x19 
#define MPU6050_RA_CONFIG           0x1A 
#define MPU6050_RA_GYRO_CONFIG      0x1B 
#define MPU6050_RA_ACCEL_CONFIG     0x1C 
#define MPU6050_RA_PWR_MGMT_1       0x6B

#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48

void MPU6050_Init(void);
void MPU6050_READ(int16_t* n);   
void MPU6050_SendByte(u8 writeAddr,u8 pBuffer);
void MPU6050_ReadBuffer(u8 readAddr,u8* pBuffer,u16 NumByteToRead);

extern int16_t MPU6050_data[7];



#endif


