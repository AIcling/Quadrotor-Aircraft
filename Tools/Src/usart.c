#include "stm32f4xx.h"
#include "usart.h"

//////////////////////////////////////////////////////////////////
  
// #if 1
// #pragma import(__use_no_semihosting)             
               
// struct __FILE 
// { 
// 	int handle; 
// }; 

// FILE __stdout;       

// void _sys_exit(int x) 
// { 
// 	x = x; 
// } 

// int fputc(int ch, FILE *f)
// { 	
// 	while((USART6->SR&0X40)==0);//  
// 	USART6->DR = (u8) ch;      
// 	return ch;
// }
// #endif
 

//static void NVIC_Configuration(void) {
//#if EN_USART2_RX
//    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  //¿ªÆôÏà¹ØÖÐ¶Ï
//    NVIC_InitTypeDef NVIC_InitStructure;

    // Usart1 NVIC(ÖÐ¶Ï¿ØÖÆÆ÷) ÅäÖÃ
 //   NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;  //´®¿Ú1ÖÐ¶ÏÍ¨µÀ
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;            //ÇÀÕ¼ÓÅÏÈ¼¶1
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;            //×ÓÓÅÏÈ¼¶1
//    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;       // IRQÍ¨µÀÊ¹ÄÜ
//    NVIC_Init(&NVIC_InitStructure);                                      //¸ù¾ÝÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯NVIC¼Ä´æÆ÷¡¢

//#endif
//}/

void USART_Config1(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
	
	
		
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2 | GPIO_Pin_3;
	
	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	

	GPIO_Init(GPIOA,&GPIO_InitStructure);
	

	

	USART_InitStructure.USART_BaudRate=9600;
	
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	
	USART_InitStructure.USART_Parity=USART_Parity_No;
	
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	
	USART_Init(USART2,&USART_InitStructure);
	
	
//	NVIC_Configuration();
	

	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	

	USART_Cmd(USART2,ENABLE);
}

void USART_Config2(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6);
	


	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	


	USART_InitStructure.USART_BaudRate=9600;

	USART_InitStructure.USART_WordLength=USART_WordLength_8b;

	USART_InitStructure.USART_StopBits=USART_StopBits_1;

	USART_InitStructure.USART_Parity=USART_Parity_No;

	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;

	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;

	USART_Init(USART6,&USART_InitStructure);
	

//	NVIC_Configuration();
	

	USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);
	

	USART_Cmd(USART6,ENABLE);
}


void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch )
{

	USART_SendData(pUSARTx,ch);

	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}


unsigned char USARTGetByte(USART_TypeDef * pUSARTx,unsigned char* GetData)
{   	   
        if(USART_GetFlagStatus(pUSARTx, USART_FLAG_RXNE) == RESET)
        {  return 0;
		}
        *GetData = USART_ReceiveData(pUSARTx); 
        return 1;
}

void USARTSendString(USART_TypeDef * pUSARTx,char* String)
{   
		for(int i = 0;String[i];i++)
				Usart_SendByte(pUSARTx,String[i]);
}


void USARTTest(USART_TypeDef * pUSARTx)
{
       unsigned char i = 0;

       while(1)
       {    
		 while(USARTGetByte(pUSARTx,&i))
        {
         Usart_SendByte(pUSARTx,i);
        }      
       }     

}


