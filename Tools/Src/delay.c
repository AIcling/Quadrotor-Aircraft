#include "delay.h"
#include "sys.h"
#include <stdint.h>
////////////////////////////////////////////////////////////////////////////////// 	 
//Èç¹ûÊ¹ÓÃOS,Ôò°üÀ¨ÏÂÃæµÄÍ·ÎÄ¼þ£¨ÒÔucosÎªÀý£©¼´¿É.
#if SYSTEM_SUPPORT_OS
//#include "includes.h"					//Ö§³ÖOSÊ±£¬Ê¹ÓÃ	  
#endif
#undef SYSTEM_SUPPORT_OS


static u8  fac_us=0;							//usÑÓÊ±±¶³ËÊý			   
static u16 fac_ms=0;							//msÑÓÊ±±¶³ËÊý,ÔÚosÏÂ,´ú±íÃ¿¸ö½ÚÅÄµÄmsÊý
	
#if SYSTEM_SUPPORT_OS							//Èç¹ûSYSTEM_SUPPORT_OS¶¨ÒåÁË,ËµÃ÷ÒªÖ§³ÖOSÁË(²»ÏÞÓÚUCOS).

#ifdef 	OS_CRITICAL_METHOD						//OS_CRITICAL_METHOD¶¨ÒåÁË,ËµÃ÷ÒªÖ§³ÖUCOSII				
#define delay_osrunning		OSRunning			//OSÊÇ·ñÔËÐÐ±ê¼Ç,0,²»ÔËÐÐ;1,ÔÚÔËÐÐ
#define delay_ostickspersec	OS_TICKS_PER_SEC	//
#define delay_osintnesting 	OSIntNesting		//ÖÐ¶ÏÇ¶Ì×¼¶±ð,¼´ÖÐ¶ÏÇ¶Ì×´ÎÊý
#endif

//Ö§³ÖUCOSIII
#ifdef 	CPU_CFG_CRITICAL_METHOD					//CPU_CFG_CRITICAL_METHOD¶¨ÒåÁË,ËµÃ÷ÒªÖ§³ÖUCOSIII	
#define delay_osrunning		OSRunning			//OSÊÇ·ñÔËÐÐ±ê¼Ç,0,²»ÔËÐÐ;1,ÔÚÔËÐÐ
#define delay_ostickspersec	OSCfg_TickRate_Hz	//OSÊ±ÖÓ½ÚÅÄ,¼´Ã¿Ãëµ÷¶È´ÎÊý
#define delay_osintnesting 	OSIntNestingCtr		//ÖÐ¶ÏÇ¶Ì×¼¶±ð,¼´ÖÐ¶ÏÇ¶Ì×´ÎÊý
#endif


//us¼¶ÑÓÊ±Ê±,¹Ø±ÕÈÎÎñµ÷¶È(·ÀÖ¹´ò¶Ïus¼¶ÑÓ³Ù)
void delay_osschedlock(void)
{
#ifdef CPU_CFG_CRITICAL_METHOD   			//Ê¹ÓÃUCOSIII
	OS_ERR err; 
	OSSchedLock(&err);						//UCOSIIIµÄ·½Ê½,½ûÖ¹µ÷¶È£¬·ÀÖ¹´ò¶ÏusÑÓÊ±
#else										//·ñÔòUCOSII
	OSSchedLock();							//UCOSIIµÄ·½Ê½,½ûÖ¹µ÷¶È£¬·ÀÖ¹´ò¶ÏusÑÓÊ±
#endif
}

//us¼¶ÑÓÊ±Ê±,»Ö¸´ÈÎÎñµ÷¶È
void delay_osschedunlock(void)
{	
#ifdef CPU_CFG_CRITICAL_METHOD   			//Ê¹ÓÃUCOSIII
	OS_ERR err; 
	OSSchedUnlock(&err);					//UCOSIIIµÄ·½Ê½,»Ö¸´µ÷¶È
#else										//·ñÔòUCOSII
	OSSchedUnlock();						//UCOSIIµÄ·½Ê½,»Ö¸´µ÷¶È
#endif
}

//µ÷ÓÃOS×Ô´øµÄÑÓÊ±º¯ÊýÑÓÊ±
//ticks:ÑÓÊ±µÄ½ÚÅÄÊý
void delay_ostimedly(u32 ticks)
{
#ifdef CPU_CFG_CRITICAL_METHOD
	OS_ERR err; 
	OSTimeDly(ticks,OS_OPT_TIME_PERIODIC,&err);//UCOSIIIÑÓÊ±²ÉÓÃÖÜÆÚÄ£Ê½
#else
	OSTimeDly(ticks);						//UCOSIIÑÓÊ±
#endif 
}
 
//systickÖÐ¶Ï·þÎñº¯Êý,Ê¹ÓÃOSÊ±ÓÃµ½
void SysTick_Handler(void)
{	
	if(delay_osrunning==1)					//OS¿ªÊ¼ÅÜÁË,²ÅÖ´ÐÐÕý³£µÄµ÷¶È´¦Àí
	{
		OSIntEnter();						//½øÈëÖÐ¶Ï
		OSTimeTick();       				//µ÷ÓÃucosµÄÊ±ÖÓ·þÎñ³ÌÐò               
		OSIntExit();       	 				//´¥·¢ÈÎÎñÇÐ»»ÈíÖÐ¶Ï
	}
}
#endif
			   
//³õÊ¼»¯ÑÓ³Ùº¯Êý
//µ±Ê¹ÓÃOSµÄÊ±ºò,´Ëº¯Êý»á³õÊ¼»¯OSµÄÊ±ÖÓ½ÚÅÄ
//SYSTICKµÄÊ±ÖÓ¹Ì¶¨ÎªAHBÊ±ÖÓµÄ1/8
//SYSCLK:ÏµÍ³Ê±ÖÓÆµÂÊ
void delay_init(u8 SYSCLK)
{
#if SYSTEM_SUPPORT_OS 						//Èç¹ûÐèÒªÖ§³ÖOS.
	u32 reload;
#endif
// 	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); 
	fac_us=SYSCLK/8;						//²»ÂÛÊÇ·ñÊ¹ÓÃOS,fac_us¶¼ÐèÒªÊ¹ÓÃ
#if SYSTEM_SUPPORT_OS 						//Èç¹ûÐèÒªÖ§³ÖOS.
	reload=SYSCLK/8;						//Ã¿ÃëÖÓµÄ¼ÆÊý´ÎÊý µ¥Î»ÎªM	   
	reload*=1000000/delay_ostickspersec;	//¸ù¾Ýdelay_ostickspersecÉè¶¨Òç³öÊ±¼ä
											//reloadÎª24Î»¼Ä´æÆ÷,×î´óÖµ:16777216,ÔÚ168MÏÂ,Ô¼ºÏ0.7989s×óÓÒ	
	fac_ms=1000/delay_ostickspersec;		//´ú±íOS¿ÉÒÔÑÓÊ±µÄ×îÉÙµ¥Î»	   
	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;   	//¿ªÆôSYSTICKÖÐ¶Ï
	SysTick->LOAD=reload; 					//Ã¿1/delay_ostickspersecÃëÖÐ¶ÏÒ»´Î	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; 	//¿ªÆôSYSTICK    
#else
	fac_ms=(u16)fac_us*1000;				//·ÇOSÏÂ,´ú±íÃ¿¸ömsÐèÒªµÄsystickÊ±ÖÓÊý   
#endif
}								    

#if SYSTEM_SUPPORT_OS 						//Èç¹ûÐèÒªÖ§³ÖOS.
//ÑÓÊ±nus
//nus:ÒªÑÓÊ±µÄusÊý.	
//nus:0~204522252(×î´óÖµ¼´2^32/fac_us@fac_us=21)	    								   
void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;				//LOADµÄÖµ	    	 
	ticks=nus*fac_us; 						//ÐèÒªµÄ½ÚÅÄÊý 
	delay_osschedlock();					//×èÖ¹OSµ÷¶È£¬·ÀÖ¹´ò¶ÏusÑÓÊ±
	told=SysTick->VAL;        				//¸Õ½øÈëÊ±µÄ¼ÆÊýÆ÷Öµ
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//ÕâÀï×¢ÒâÒ»ÏÂSYSTICKÊÇÒ»¸öµÝ¼õµÄ¼ÆÊýÆ÷¾Í¿ÉÒÔÁË.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//Ê±¼ä³¬¹ý/µÈÓÚÒªÑÓ³ÙµÄÊ±¼ä,ÔòÍË³ö.
		}  
	};
	delay_osschedunlock();					//»Ö¸´OSµ÷¶È											    
}  
//ÑÓÊ±nms
//nms:ÒªÑÓÊ±µÄmsÊý
//nms:0~65535
void delay_ms(u16 nms)
{	
	if(delay_osrunning&&delay_osintnesting==0)//Èç¹ûOSÒÑ¾­ÔÚÅÜÁË,²¢ÇÒ²»ÊÇÔÚÖÐ¶ÏÀïÃæ(ÖÐ¶ÏÀïÃæ²»ÄÜÈÎÎñµ÷¶È)	    
	{		 
		if(nms>=fac_ms)						//ÑÓÊ±µÄÊ±¼ä´óÓÚOSµÄ×îÉÙÊ±¼äÖÜÆÚ 
		{ 
   			delay_ostimedly(nms/fac_ms);	//OSÑÓÊ±
		}
		nms%=fac_ms;						//OSÒÑ¾­ÎÞ·¨Ìá¹©ÕâÃ´Ð¡µÄÑÓÊ±ÁË,²ÉÓÃÆÕÍ¨·½Ê½ÑÓÊ±    
	}
	delay_us((u32)(nms*1000));				//ÆÕÍ¨·½Ê½ÑÓÊ±
}
#else  //²»ÓÃucosÊ±
//ÑÓÊ±nus
//nusÎªÒªÑÓÊ±µÄusÊý.	
//×¢Òâ:nusµÄÖµ,²»Òª´óÓÚ798915us(×î´óÖµ¼´2^24/fac_us@fac_us=21)
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*2; 				//Ê±¼ä¼ÓÔØ	  		 
	SysTick->VAL=0x00;        				//Çå¿Õ¼ÆÊýÆ÷
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ; //¿ªÊ¼µ¹Êý 	 
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//µÈ´ýÊ±¼äµ½´ï   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; //¹Ø±Õ¼ÆÊýÆ÷
	SysTick->VAL =0X00;       				//Çå¿Õ¼ÆÊýÆ÷ 
}

void delay_xms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;			//Ê±¼ä¼ÓÔØ(SysTick->LOADÎª24bit)
	SysTick->VAL =0x00;           			//Çå¿Õ¼ÆÊýÆ÷
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //¿ªÊ¼µ¹Êý 
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//µÈ´ýÊ±¼äµ½´ï   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //¹Ø±Õ¼ÆÊýÆ÷
	SysTick->VAL =0X00;     		  		//Çå¿Õ¼ÆÊýÆ÷	  	    
} 
//ÑÓÊ±nms 
//nms:0~65535
void delay_ms(u16 nms)
{	 	 
	u32 temp;	    	 
	SysTick->LOAD=nms*168000; 				//Ê±¼ä¼ÓÔØ	  		 
	SysTick->VAL=0x00;        				//Çå¿Õ¼ÆÊýÆ÷
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ; //¿ªÊ¼µ¹Êý 	 
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//µÈ´ýÊ±¼äµ½´ï   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; //¹Ø±Õ¼ÆÊýÆ÷
	SysTick->VAL =0X00;       				//Çå¿Õ¼ÆÊýÆ÷ 
} 
#endif

void Delay_us(u32 nus){
	u32 told = SysTick->VAL, tload = SysTick->LOAD, tnew = SysTick->VAL;
	u32 cnt = nus*2;
	// SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; 
	while((told-tnew+tload) % tload < cnt){
		tnew = SysTick->VAL;
	}

}
void Delay_ms(u32 nms){
	while(nms--){
		Delay_us(1000);
	}
}