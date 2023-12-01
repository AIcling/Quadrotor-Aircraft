#include "MPU6050.h"
#include "i2c_soft.h"
#include "stm32f4xx.h"                  // Device header
#include "HMC5883L.h"
#include "i2c.h"
#include "OLED.h"
#include "Matrix.h"
#include <math.h>
#include <stdint.h>
#include <reent.h>
int16_t HMC5883L_data[3] = {0};
void HMC5883L_SendData(u8 writeAddr,u8 pBuffer);
u8 HMC5883L_Status(void);
void HMC5883L_Init(void)
{
/*

	I2C_SEND_BYTE(0xD0,0x6A,0x00);
	I2C_SEND_BYTE(0xD0,0x37,0x02);
	I2C_SEND_BYTE(0x3C,0x00,0x30);
	I2C_SEND_BYTE(0x3C,0x01,0x00);
	I2C_SEND_BYTE(0x3C,0x02,0x00);
*/
   MPU6050_SendByte(0x6A, 0x00);
   MPU6050_SendByte(0x37,0x02);
//    HMC5883L_SendData(0x00, 0x30);
//    HMC5883L_SendData(0x01, 0x00);
//    HMC5883L_SendData(0x02, 0x00);
   HMC5883L_SendData(0x00, 0x3c);
   HMC5883L_SendData(0x01, 0x02);
   HMC5883L_SendData(0x02, 0x00);
}


void HMC5883L_Read(uint16_t *data)
{
	uint8_t i;
    uint8_t t[6]; 
	uint8_t addr=0x03;
//	I2C_READ_BUFFER(0x3C,0x03,t,6); 
//		for(i=0; i<3; i++) 	
//		data[i]=((t[2*i] << 8)+t[2*i+1]);
	
//	t[0]=I2C_READ_BYTE(0x3C,0x03);
//	t[1]=I2C_READ_BYTE(0x3C,0x04);
//	data[0]=(t[0]<<8)+t[1];
//	t[2]=I2C_READ_BYTE(0x3C,0x05);
//	t[3]=I2C_READ_BYTE(0x3C,0x06);
//	data[1]=(t[2]<<8)+t[3];
//	t[4]=I2C_READ_BYTE(0x3C,0x07);
//	t[5]=I2C_READ_BYTE(0x3C,0x08);
//	data[2]=(t[4]<<8)+t[5];
	
	while(!(I2C_READ_BYTE(0x3C,0x09)&0x01));
	for(i=0; i<3; i++) 	
	{
		t[2*i]=I2C_READ_BYTE(0x3C,addr);
		t[2*i+1]=I2C_READ_BYTE(0x3C,addr+1);
		data[i]=((t[2*i] << 8)+t[2*i+1]);
		addr+=2;
	}	
}

void HMC5883L_SendData(u8 writeAddr,u8 pBuffer){
	I2CStart();
	I2CSendByte(HMC5883L_ADD);
	while(!I2CWaitAck());
	I2CSendByte(writeAddr);
	while(!I2CWaitAck());
	I2CSendByte(pBuffer);
	while(!I2CWaitAck());
	I2CStop();	
}
void HMC588CL_ReadData(int16_t *data){
	int i =6;
    u8 t[6]; 	
	u8* pBuffer = t;
	while(!(HMC5883L_Status()&0x01));
	I2CStart();
	I2CSendByte(HMC5883L_ADD);	
	 while(!I2CWaitAck());
	I2CSendByte(0x03);
	 while(!I2CWaitAck());
	I2CStart();
	I2CSendByte(HMC5883L_ADD|0x01);
	 while(!I2CWaitAck());
	 while(i--){
		if(i==1){
			*pBuffer = I2CReceiveByte();
			I2CSendNotAck();
			I2CStop();
			break;
		}else{
		*pBuffer = I2CReceiveByte();
		I2CSendAck();
		pBuffer++;
		// i--;
		}		
	 }	
	 for(int j=0;j<3;j++){
		data[j] = (t[2*j]<<8) + t[2*j+1];
	 }
}
u8 HMC5883L_Status(void){
	I2CStart();
	I2CSendByte(HMC5883L_ADD);
	while(!I2CWaitAck());
	I2CSendByte(0x09);  
	 while(!I2CWaitAck());
	I2CStart();
	I2CSendByte(HMC5883L_ADD|0x01);
	 while(!I2CWaitAck());
	 u8 buffer = I2CReceiveByte();
	 I2CSendNotAck();
	 I2CStop();	 
	 return buffer;	
}
float Rx=0,Ry=0,Rz=0,Ox=0,Oy=0,Oz=0;
void Ellipsoid_Cal(void){
	HMC588CL_ReadData(HMC5883L_data);
	float mx = HMC5883L_data[0]/1090.0f;
	float my = HMC5883L_data[1]/1090.0f;
	float mz = HMC5883L_data[2]/1090.0f;
	float _x_2 = -mx *mx;
	float y_2 = my * my;
	float z_2 = mz * mz;
	float array[6][6] = {
		{y_2*y_2 , y_2*z_2 , y_2*mx , y_2*my , y_2*mz , y_2},
		{z_2*y_2 , z_2*z_2 , z_2*mx , z_2*my , z_2*mz , z_2},
		{mx*y_2 , mx*z_2 , mx*mx , mx*my , mx*mz , mx},
		{y_2*my , my*z_2 , my*mx , my*my , my*mz , my},
		{mz*y_2 , mz*z_2 , mz*mx , mz*my , z_2 , mz},
		{y_2 , z_2 , mx , my , mz , 1}
	};
	matrix_inver(array);
	// 	float array[36]= {
	// 	y_2*y_2 , y_2*z_2 , y_2*mx , y_2*my , y_2*mz , y_2,
	// 	z_2*y_2 , z_2*z_2 , z_2*mx , z_2*my , z_2*mz , z_2,
	// 	mx*y_2 , mx*z_2 , mx*mx , mx*my , mx*mz , mx,
	// 	y_2*my , my*z_2 , my*mx , y_2 , my*mz , my,
	// 	mz*y_2 , mz*z_2 , mz*mx , mz*my , z_2 , mz,
	// 	y_2 , z_2 , mx , my , mz , 1
	// };
	// inv(array,6);

	float tmp[6] = {0};
	float K_T[6] = {y_2,z_2,mx,my,mz,1};
	int row=0;
	while(row<6){
		for(int j=0;j<6;j++){
			tmp[row] += array[row][j] * K_T[j];
		}
		row++;
	}
	float A = tmp[0] * _x_2;
	float B = tmp[1] * _x_2;
	float C = tmp[2] * _x_2;
	float D = tmp[3] * _x_2;
	float E = tmp[4] * _x_2;
	float F = tmp[5] * _x_2;
	Ox += -C/2;
	Oy += -D/(2*A);
	Oz += -E/(2*B);
	Rx += sqrt(Ox*Ox + A*Oy*Oy + B*Oz*Oz - F);
	Ry += sqrt(Rx*Rx/A);
	Rz += sqrt(Rx*Rx/B);

}
void Ellipsoid_Fitting(void){
	for(int i=0;i<200;i++) {Ellipsoid_Cal();}
	Ox = Ox/200;
	Oy = Oy/200;
	Oz = Oz/200;
	Rx = Rx/200;
	Ry = Ry/200;
	Rz = Rz/200;
}































