#include "stm32f4xx.h"                  // Device header
#include "OLED.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "i2c.h"
#include "stm32f4xx_rcc.h"
#include "Serial.h"
#include "IMU.h"

#include "stm32f4xx_usart.h"
#include "sysTick.h"
#include "usart.h"
#include "delay.h"
#include "sys.h"
#include "i2c_soft.h"
#include <stdint.h>
#include <string.h>
#include "Receiver.h"
#include "Motor.h"
#include "HMC5883L.h"
#include "Pid.h"

//#include "ucos_ii.h"
//#include "os_trace_events.h"
void Send_PPM(void);
void Send_Pose(void);
u32 betime = 0, aftime = 0;
char sendbuffer[50];
u32 betick= 0, aftick = 0;
int times = 100;
int main(){
	SystemInit();
	SysTick->LOAD = 0xffffff;
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; 
	I2CInit();
	MPU6050_Init();
	HMC5883L_Init();
	Delay_ms(5);
	//HMC5883L_Motify();
	get_MPU6050err();
	for(int i=0;i<5;i++) HMC588CL_ReadData(HMC5883L_data);
	uart_init();
	My_Receiver_Init();
	Motor_Init();
	PoseData_Init();
	 betime = SysTick->VAL;
	 betick = SysTick->VAL;
	while(1){
		MPU6050_READ(MPU6050_data);
		HMC588CL_ReadData(HMC5883L_data);
		aftime = SysTick->VAL;
		if((betime - aftime)<0) betime += SysTick->LOAD;
		float intertime = (betime-aftime)/2000000.0f;
		betime = aftime;
		CalAngle(&angle, intertime);
		aftick = SysTick->VAL;
		 
		 if((betick- aftick)<0) betick += SysTick->LOAD;
		//姿态上传和PID反馈控制的时间控制在0.025s/次
		 
		 if(betick-aftick>50000){
			// Send_PPM();
			  Send_Elua();
			  PID_Controller_DL();
			// Send_Pose();
			betick = aftick;
		//  }
	}
}




}
void Send_PPM(void){
	char ppmbuffer[100];
	sprintf(ppmbuffer,"%d %d %d %d %d %d %d %d %d\n",PPM_Data[0],PPM_Data[1],PPM_Data[2],PPM_Data[3],PPM_Data[4],PPM_Data[5],PPM_Data[6],PPM_Data[7],PPM_Data[8]);
	USARTSendStr(USART1,ppmbuffer);
}
void get_MPU6050err(void){
  float Gyro_Xerrsum,Gyro_Yerrsum,Gyro_Zerrsum = 0;
  for(int i=0;i<times;i++){
	MPU6050_READ(MPU6050_data);
    Gyro_Xerrsum += MPU6050_data[4];
	Gyro_Yerrsum += MPU6050_data[5];
	Gyro_Zerrsum += MPU6050_data[6];
  }
  Gyro_Xerr = Gyro_Xerrsum/times;
  Gyro_Yerr = Gyro_Yerrsum/times;
  Gyro_Zerr = Gyro_Zerrsum/times;
}
void Send_Pose(void){
	char sendbuffer[50];
	sprintf(sendbuffer,"%.3d %.3d %.3d\n",(int)(angle.pitch*57.3),(int)(angle.roll*57.3),(int)(angle.yaw*57.3));
	USARTSendStr(USART1, sendbuffer);
}
