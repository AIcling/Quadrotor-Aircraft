#include "delay.h"
#include "stm32f4xx.h"                  // Device header
#include "MPU6050.h"
#include "OLED.h"
#include <stdint.h>
#include "i2c_soft.h"
int16_t MPU6050_data[7];
void MPU6050_Init(void){  //初始化MPU6050
	MPU6050_SendByte(0x6B,0x80);//解除休眠状态
	//  Delay_ms(1000); //等待器件就绪
	// MPU6050_SendByte(0x6C,0xC0);
	MPU6050_SendByte(0x6B,0x00);//8MZ内部晶振
	MPU6050_SendByte(0x19,0x01);//陀螺仪采样率
	MPU6050_SendByte(0x1A,0x06);//中断和低通滤波器
	MPU6050_SendByte(0x1B,0x18);//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
	MPU6050_SendByte(0x1C,0x00);//配置加速度传感器工作在4G模式
	
	//I2C_SEND_BYTE(0xD0,0x6A,0x00);//与HMC5883L有关
	//I2C_SEND_BYTE(0xD0,0x37,0x02); 
}
void MPU6050_SendByte(u8 writeAddr,u8 pBuffer){
	I2CStart();
    I2CSendByte(MPU6050_ADD);
	while(!I2CWaitAck());
	I2CSendByte(writeAddr);
	while(!I2CWaitAck());
	I2CSendByte(pBuffer);
	while(!I2CWaitAck());
	I2CStop();
}

void MPU6050_READ(int16_t* n){ //读出X、Y、Z三轴加速度/陀螺仪原始数据 //n[0]是AX
	u8 i;
    u8 t[14];
    MPU6050_ReadBuffer(0x3B,t,14); //读出连续的数据地址，包括了加速度和陀螺仪和温度共14字节
    for(i=0; i<7; i++) 	//整合加速度、温度、陀螺仪
      n[i]=((t[2*i] << 8)+t[2*i+1]); 
	
//	t[0]=I2C_READ_BYTE(0xD0,0x3B);             
//	t[1]=I2C_READ_BYTE(0xD0,0x3C);      
//	n[0]=((t[0]<<8)+t[1]);   
	
}
void MPU6050_ReadBuffer(u8 readAddr,u8* pBuffer,u16 NumByteToRead){
	I2CStart();
	I2CSendByte(MPU6050_ADD);	
	 while(!I2CWaitAck());
	I2CSendByte(readAddr);
	 while(!I2CWaitAck());
	I2CStart();
	I2CSendByte(MPU6050_ADD|0x01);
	 while(!I2CWaitAck());
	while(NumByteToRead){
		if(NumByteToRead==1){
			*pBuffer = I2CReceiveByte();
			I2CSendNotAck();
			I2CStop();
			break;
		}else{
		*pBuffer = I2CReceiveByte();
		I2CSendAck();
		pBuffer++;
		NumByteToRead--;
		}
	}
}                                    




