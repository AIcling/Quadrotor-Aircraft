#include "Receiver.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include <stdint.h>

uint16_t PPM_Data[9] ={0};
 int ppm_idx = 0;
uint8_t TIM1_CapPri = 3;
uint8_t TIM1_UpdPri = 4;
void Receiver_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;//PA8
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;//
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	
	TIM_InternalClockConfig(TIM1);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	//因为高级定时器有很多要配置的东西，所以用这个函数先把我们不用的
	//东西配置为缺省值，再更改我们需要配置的选项
	TIM_TimeBaseStructInit( & TIM_TimeBaseInitStructure);
	
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM
	TIM_TimeBaseInitStructure.TIM_Period = 65536-1;  //这个值，只要是用于输入捕获测频率，就尽量给到最大
	TIM_TimeBaseInitStructure.TIM_Prescaler = 2-1;  //以1Mhz计数
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM1, & TIM_TimeBaseInitStructure);
	
	TIM_ICInitTypeDef TIM_ICInitStruct;
	TIM_ICInitStruct.TIM_Channel=TIM_Channel_1;
	TIM_ICInitStruct.TIM_ICFilter=0xF;
	TIM_ICInitStruct.TIM_ICPolarity=TIM_ICPolarity_Rising;
	TIM_ICInitStruct.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	TIM_ICInitStruct.TIM_ICSelection=TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM1,&TIM_ICInitStruct);
	
	//其实接收机不需要这个通道，因为我只需测量上升沿就够了
	TIM_ICInitStruct.TIM_Channel=TIM_Channel_2;
	TIM_ICInitStruct.TIM_ICFilter=0xF;
	TIM_ICInitStruct.TIM_ICPolarity=TIM_ICPolarity_Falling;
	TIM_ICInitStruct.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	TIM_ICInitStruct.TIM_ICSelection=TIM_ICSelection_IndirectTI;
	TIM_ICInit(TIM1,&TIM_ICInitStruct);
	
	//选择输入触发源
	TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);
	TIM_SelectSlaveMode(TIM1,TIM_SlaveMode_Reset);
	
	//以下部分为中断配置
	TIM_ITConfig(TIM1,TIM_IT_CC1,ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);
	
	//上电
	TIM_Cmd(TIM1,ENABLE);
	
}



uint32_t Receiver_Data(void)
{
	return (TIM_GetCapture1(TIM1)+1);
}

void My_Receiver_Init(void)
{
  //我的思路是利用ppm信号20ms一次的特点对CNT进行清零
  //引脚复用
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;//PA8
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;//
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
 //然后是TIM时基单元初始化
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	RCC_ClocksTypeDef ClkFre;
	RCC_GetClocksFreq(&ClkFre);
	uint32_t PCLK2 = ClkFre.PCLK2_Frequency;
	TIM1->PSC = (PCLK2/1000000)-1;      //每0.001ms计数一次
	TIM1->ARR = 2100;         //2.1ms发生上溢，产生更新中断，一路ppm信号完
	TIM1->CCMR1 |= 0x01;      //CC1输入，IC1->TI1
	TIM1->CCER |= 0x01 | 0x00;//使能捕获，上升沿触发
	TIM1->DIER |= 0x01 |0x02; //使能输入捕获中断和更新中断
	TIM1->CR1 |= 0x01;        //使能计数器
//中断设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC->IP[TIM1_CC_IRQn] = TIM1_CapPri<<0x04;
	NVIC->IP[TIM1_UP_TIM10_IRQn] = TIM1_UpdPri<<0x04;
	NVIC->ISER[TIM1_CC_IRQn>>5] |= 0x01 << (TIM1_CC_IRQn&0x1F);
	NVIC->ISER[TIM1_UP_TIM10_IRQn>>5] |= 0x01 << (TIM1_UP_TIM10_IRQn&0x1F); 
}
void TIM1_CC_IRQHandler(void){
	// if(ppm_idx<8){
	PPM_Data[ppm_idx++] = TIM1->CCR1;
	// if(PPM_Data[ppm_idx-1]>3000) ppm_idx =0;
	// if( PPM_Data[ppm_idx] > 20) ppm_idx =0;
	// ppm_idx ++;
	// }
	TIM1->CNT = 0;
}

void TIM1_UP_TIM10_IRQHandler(void){
	 ppm_idx = 0;
	 TIM1->CNT = 0;
	TIM1->SR &= ~0x01;   //清零更新中断标志
}

