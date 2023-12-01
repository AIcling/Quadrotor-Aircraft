#include "Motor.h"
#include "delay.h"

void Motor_Init(void)
{
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);//开启外设时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//开启外设时钟
	/*  配置GPIO的模式和IO口 */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;// 四个通道全部初始化
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;//配置为复用模式
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化GPIO
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);//配置PC6复用功能
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);//配置PC7复用功能
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);//配置PC8复用功能
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);//配置PC9复用功能
	//TIM3定时器初始化	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_Period = 19999; //重载值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 199;//预分频系数
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//不指定时钟分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//重复计数值为0
	TIM_TimeBaseInit(TIM3, & TIM_TimeBaseInitStructure);//初始化时基单元
	
	//PWM配置	  
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);//先将无需特别设置的部分设为缺省值
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;//选择PWM1模式
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;//选择极性
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//PWM输出使能
	TIM_OCInitStructure.TIM_Pulse=0;//先将CCR设置为0
	
	//分别初始化四个通道
	TIM_OC1Init(TIM3,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM3,&TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM3,&TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
}
void ESC_Calibration(void)
{	
	TIM_SetCompare1(TIM3, 2000);//10%
	TIM_SetCompare2(TIM3, 2000);
	TIM_SetCompare3(TIM3, 2000);
	TIM_SetCompare4(TIM3, 2000);
	Delay_ms(4000);
	TIM_SetCompare1(TIM3, 1000);//5%
	TIM_SetCompare2(TIM3, 1000);
	TIM_SetCompare3(TIM3, 1000);
	TIM_SetCompare4(TIM3, 1000);
	Delay_ms(4000);
}

//电机调速部分，传入的值表示电机满速的百分之多少
void Set_Speed(uint8_t motor,uint16_t speed)
{
	if(speed<10||speed>20)//电机速度取值范围为10-20，10为停止转动，20为满速转动
	{
		return;
	}
	if(motor==1)//一号电机
	{
		TIM_SetCompare1(TIM3,speed);//利用库函数修改CCR值
	}
	else if(motor==2)//二号电机
	{
		TIM_SetCompare2(TIM3,speed);
	}
	else if(motor==3)//三号电机
	{
		TIM_SetCompare3(TIM3,speed);
	}
	else if(motor==4)//四号电机
	{
		TIM_SetCompare4(TIM3,speed);
	}
}