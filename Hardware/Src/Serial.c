#include "IMU.h"
#include "Receiver.h"
#include "stm32f4xx.h"
#include "Serial.h"
#include <stdint.h>
void MY_UART_Send(uint8_t *string, uint8_t size);
void uart_init(){
GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); 

RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1);


//USART1 PB6 PB7
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ; 
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
GPIO_Init(GPIOB,&GPIO_InitStructure); 

USART_InitStructure.USART_BaudRate = 9600;
USART_InitStructure.USART_WordLength = USART_WordLength_8b;
USART_InitStructure.USART_StopBits = USART_StopBits_1;
USART_InitStructure.USART_Parity = USART_Parity_No;
USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;
USART_InitStructure.USART_Mode = USART_Mode_Tx;
USART_Init(USART1, &USART_InitStructure); 
USART_Cmd(USART1, ENABLE); 


}


#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int putc(int ch)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

void USARTSendStr(USART_TypeDef* USARTx, char* str)
{
	uint16_t i=0;
	
	while( *(str+i) != '\0' )
	{
		/*
		USART_SendData( USARTx, *(str+i));
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET );
		*/
			
		USART_SendData(USARTx, *(str+i) );
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);  //不需要手动清零 再次写入TDR时会自动清零	
		i++;
	}
	
	while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET ); //判断 str 是否已被（全部！！！）发送出去
}

// DATA_Frame MPU_Data = {0xAA,0xFF,0x04,0x09,0,0,0,0,0x01,0,0,};
// void Send_Pose(void){
// 	MPU_Data.DATA[0] = QNum.q0*10000;
// 	MPU_Data.DATA[1] = QNum.q1*10000;
// 	MPU_Data.DATA[2] = QNum.q2*10000;
// 	MPU_Data.DATA[3] = QNum.q3*10000;
//     uint8_t sum,add = 0;
// 	sum = 0xAA+0xFF+0x04+0x09+0x01;
// 	for(i=0;)

// }
uint8_t Frame[13] = {0};             //发送的数据帧
void PoseData_Init(void){
	Frame[0] = 0xAA;                 //帧头
	Frame[1] = 0xFF;                 //目标地址
	Frame[2] = 0x03;                 // 功能码 0x03表示发送欧拉角
	Frame[3] = 0x07;                 //数据长度
	Frame[10] = 0x01;                // 融合状态 （我也不知道是什么，随便给的1试了几个数字都没什么影响）
}
// void Send_Qnum(void){
// 	uint16_t Q0 = QNum.q0*10000;
// 	uint16_t Q1 = QNum.q1*10000;
// 	uint16_t Q2 = QNum.q2*10000;
// 	uint16_t Q3 = QNum.q3*10000;
// 	Frame[4] =  Q0 & 0xFF;
// 	Frame[5] =  (Q0 >> 8) & 0xFF;
// 	Frame[6] =  Q1 & 0xFF;
// 	Frame[7] =  (Q1 >> 8) & 0xFF;	
// 	Frame[8] =  Q2 & 0xFF;
// 	Frame[9] =  (Q2 >> 8) & 0xFF;		
// 	Frame[10] =  Q3 & 0xFF;
// 	Frame[11] =  (Q3 >> 8) & 0xFF;	
// 	Frame[13] = 0;
// 	Frame[14] = 0;
	// for(int i=0;i<=12;i++){
	// 	Frame[13] += Frame[i];
	// 	Frame[14] += Frame[13];
	// }
// 	MY_UART_Send(Frame,15);
// }

void Send_Elua(void){
	int16_t roll = angle.roll * 100 *57.3;       //三个角 要乘以100 注意是有符号数
	int16_t pitch = angle.pitch * 100*57.3;   
	int16_t yaw = angle.yaw * 100*57.3;
	Frame[4] = roll & 0xFF;                     //用两个字节储存 低字节在前 高字节在后
	Frame[5] = (roll>>8) & 0xFF;
	Frame[6] = pitch & 0xFF;
	Frame[7] = (pitch>>8) & 0xFF;	
	Frame[8] = yaw & 0xFF;
	Frame[9] = (yaw>>8) & 0xFF;
	Frame[11] = 0;                               //和校验
	Frame[12] = 0;		                         //附加校验
	for(int i=0;i<=10;i++){
		Frame[11] += Frame[i];
		Frame[12] += Frame[11];
	}
	MY_UART_Send(Frame,13);
}
void MY_UART_Send(uint8_t *string, uint8_t size){
    while(size){
        while(!(USART1->SR & USART_SR_TXE));
        USART1->DR = *string;
        string++;
        size--;
    }
}



