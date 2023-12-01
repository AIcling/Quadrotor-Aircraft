#include "i2c.h"

void I2C_Configuration(void){ //I2C初始化
	//I2C接口初始化
		GPIO_InitTypeDef  GPIO_InitStructure;	
		I2C_InitTypeDef  I2C_InitStructure;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);       
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //启动I2C功能 
		GPIO_InitStructure.GPIO_Pin = I2C_SCL|I2C_SDA; //选择端口号                      
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //选择IO接口工作方式为复用输出      
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //设置IO接口速度    
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//开漏输出
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//无上拉
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_I2C1);  //这两个的配置必须有，没有则无法输出
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_I2C1);
		
		I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;//设置为I2C模式
		I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
		I2C_InitStructure.I2C_OwnAddress1 = HostAddress; //主机地址（从机不得用此地址）
		I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;//允许应答
		I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; //7位地址模式
		I2C_InitStructure.I2C_ClockSpeed = I2C_Speed; //总线速度设置 	
		I2C_Init(I2C1,&I2C_InitStructure);
		I2C_Cmd(I2C1,ENABLE);//开启I2C					
}

void I2C_SEND_BUFFER(u8 SlaveAddr,u8 WriteAddr,u8* pBuffer,u16 NumByteToWrite){ //I2C发送数据串（器件地址，内部地址，寄存器，数量）
	I2C_GenerateSTART(I2C1,ENABLE);//产生起始位
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); //清除EV5
	I2C_Send7bitAddress(I2C1,SlaveAddr,I2C_Direction_Transmitter);//发送器件地址
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//清除EV6
	I2C_SendData(I2C1,WriteAddr); //内部功能地址
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));//移位寄存器非空，数据寄存器已空，产生EV8，发送数据到DR既清除该事件
	while(NumByteToWrite--){ //循环发送数据	
		I2C_SendData(I2C1,*pBuffer); //发送数据
		pBuffer++; //数据指针移位
		while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));//清除EV8
	}
	I2C_GenerateSTOP(I2C1,ENABLE);//产生停止信号
}

void I2C_SEND_BYTE(u8 SlaveAddr,u8 writeAddr,u8 pBuffer){ //I2C发送一个字节（从地址，内部地址，内容）
	I2C_GenerateSTART(I2C1,ENABLE); //发送开始信号
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); //等待完成	 
	I2C_Send7bitAddress(I2C1,SlaveAddr, I2C_Direction_Transmitter); //发送从器件地址及状态（写入）
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); //等待完成	
	I2C_SendData(I2C1,writeAddr); //发送从器件内部寄存器地址
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); //等待完成	
	I2C_SendData(I2C1,pBuffer); //发送要写入的内容
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); //等待完成	
	I2C_GenerateSTOP(I2C1,ENABLE); //发送结束信号
	
//	//产生开始信号
//	I2C1->CR1|=0x01<<8;
//	while(!MY_CHECK_EVENT(0x0001,0x0003));
//	//因为是写，所以不用给地址加一
//	I2C1->DR=SlaveAddr;
//	while(!MY_CHECK_EVENT(0x0082,0x0007));
//	I2C1->DR = writeAddr;
//	while(!MY_CHECK_EVENT(0x0084,0x0007));
//	I2C1->DR = pBuffer;
//	while(!MY_CHECK_EVENT(0x0084,0x0007));
//	//产生结束信号
//	I2C1->CR1|=0x01<<9;
	
}

void I2C_READ_BUFFER(u8 SlaveAddr,u8 readAddr,u8* pBuffer,u16 NumByteToRead){ //I2C读取数据串（器件地址，寄存器，内部地址，数量）
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C1,ENABLE);//开启信号
	
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));	//清除 EV5
	I2C_Send7bitAddress(I2C1,SlaveAddr, I2C_Direction_Transmitter); //写入器件地址
	
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//清除 EV6
	I2C_Cmd(I2C1,ENABLE);
	I2C_SendData(I2C1,readAddr); //发送读的地址
	
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); //清除 EV8
	I2C_GenerateSTART(I2C1,ENABLE); //开启信号
	
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); //清除 EV5
	I2C_Send7bitAddress(I2C1,SlaveAddr,I2C_Direction_Receiver); //将器件地址传出，主机为读
	
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); //清除EV6
	while(NumByteToRead){
		if(NumByteToRead == 1){ //只剩下最后一个数据时进入 if 语句
			I2C_AcknowledgeConfig(I2C1,DISABLE); //最后有一个数据时关闭应答位
			I2C_GenerateSTOP(I2C1,ENABLE);	//最后一个数据时使能停止位
		}
		if(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)){ //读取数据
			*pBuffer = I2C_ReceiveData(I2C1);//调用库函数将数据取出到 pBuffer
			pBuffer++; //指针移位
			NumByteToRead--; //字节数减 1 
		}
	}
	I2C_AcknowledgeConfig(I2C1,ENABLE);
	
}

u8 I2C_READ_BYTE(u8 SlaveAddr,u8 readAddr){ //I2C读取一个字节
	u8 a;
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C1,ENABLE);
	
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1,SlaveAddr, I2C_Direction_Transmitter); 
	
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_Cmd(I2C1,ENABLE);
	I2C_SendData(I2C1,readAddr);
	
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTART(I2C1,ENABLE);
	
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1,SlaveAddr, I2C_Direction_Receiver);
	
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	
	I2C_AcknowledgeConfig(I2C1,DISABLE); //最后有一个数据时关闭应答位
	I2C_GenerateSTOP(I2C1,ENABLE);


	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
	a = I2C_ReceiveData(I2C1);
	I2C_AcknowledgeConfig(I2C1,ENABLE);
		
	return a;
}

uint8_t MY_CHECK_EVENT(uint16_t SR1,uint16_t SR2)
{
	//先读取SR1，只有ADDR被置位并且STOPF被清除才可以读取SR2的值
	if( (I2C1->SR1&0x02 ) || !(I2C1->SR1&0x10) )
	{
		if( ( (I2C1->SR1 & SR1)==SR1 ) && ( (I2C1->SR2 & SR2)==SR2 ) )
			return 1;
		else
			return 0;
	}
	else
	{
		return 0;
	}
}
