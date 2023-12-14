#include "stm32f4xx.h"

#include "Pid.h"
#include "IMU.h"
#include "Motor.h"
#include "Receiver.h"

#include <stdio.h>

int16_t Motor1_PWM;
int16_t Motor2_PWM;
int16_t Motor3_PWM;
int16_t Motor4_PWM;

int16_t		Oil_PWM;
int16_t 	Pitch_PWM;
int16_t 	Roll_PWM;
int16_t 	Yaw_PWM;

PID_Ctrl PID;

float Kp = 3.5f;
float Ki = 0.00f;
float Kd = 30.0f;

float Kp_IN = 2.0f;
float Ki_IN = 0.0f;
float Kd_IN = 25.0f;
float Kp_OUT = 1.0f;
float Ki_OUT = 0.0f;
float Kd_OUT = 3.0f;

/*在该文件中，需要实现PID反馈控制*/
/*角度为我们的预设值，该预设值需要由遥控器数据算出来*/

/*这里，我们暂时将角度的范围设定为+-30(Pitch和Roll角就是这个范围)*/
/*Yaw角可以根据遥控器的转动幅度决定其变化的快慢*/

/*该函数用于计算PID输出*/
//首先解析遥控器的数据，依据该数据算出Oil_PWM
//之后根据其他通道的数据，换算出目标角度
//然后再依据目标角度与当前角度的差值，计算PID输出
//也即计算Pitch_PWM  Roll_PWM  Yaw_PWM
//然后进行叠加，最后根据叠加数据来调整电机转速

//遥控器通道说明如下
//左舵上下:飞机前进，后退			(Pitch_PWM)PPM2 上1000，下2000
//左舵左右:飞机左倾右倾，			(Roll_PWM)PPM4  左1000，右2000
//右舵上下:飞机垂直上下运动 		(Oil_PWM)PPM3   上2000，下1000
//右舵左右:飞机机头向左或向右转动	(Yaw_PWM)PPM1	左1000，右2000
//还要选一个通道来直接关闭电机，自行选择
//我们可以选用通道5(PPM5)或者通道7(PPM7)
//往下为1000，往上为2000，中间为1500

//电机实际大概会在1700的时候会带动飞机起飞，把该值与1500相对应
//从而实现一个映射关系


//PB6:一号电机  PB7:二号电机   PB8:三号电机   PB9:四号电机

//
//	PB6   PB7    一   二
//	   \ /	       \ /	
//     / \         / \
//	PB8	  PB9    三	  四
//

/**
  * @brief 这是单环PID控制器，是使用的PID结构体来实现，
  *		   
  * @args
  * @returnval
  */


void PID_Controller(void)
{
	Oil_PWM = PPM_Data[3];
	PID.Pitch_Tar = (PPM_Data[2]-1500)/33.333f;
	PID.Roll_Tar  = (PPM_Data[4]-1500)/33.333f;
	
	PID.Pitch_Err_Last = PID.Pitch_Err_Now;				//更新上一次的误差
	PID.Pitch_Err_Now = angle.pitch - PID.Pitch_Tar;	//更新这一次的误差
	PID.Pitch_Err_All += PID.Pitch_Err_Now;				//对误差求和
	Pitch_PWM = (int16_t)( Kp*PID.Pitch_Err_Now + Ki*PID.Pitch_Err_All + Kd*(PID.Pitch_Err_Now - PID.Pitch_Err_Last) );//计算输出
	
	//一下过程与计算Pitch的输出同理
	PID.Roll_Err_Last = PID.Roll_Err_Now;
	PID.Roll_Err_Now = angle.roll - PID.Roll_Tar;
	PID.Roll_Err_All += PID.Roll_Err_Now;
	Roll_PWM = (int16_t)( Kp*PID.Roll_Err_Now + Ki*PID.Roll_Err_All + Kd*(PID.Roll_Err_Now - PID.Roll_Err_Last) );
	
	
	Motor1_PWM = Oil_PWM - Pitch_PWM + Roll_PWM;
	Motor2_PWM = Oil_PWM - Pitch_PWM - Roll_PWM;
	Motor3_PWM = Oil_PWM + Pitch_PWM + Roll_PWM;
	Motor4_PWM = Oil_PWM + Pitch_PWM - Roll_PWM;//叠加输出
	
	if(Motor1_PWM<1000)//防止出现非法范围内的CCR值
	{
		Motor1_PWM=1000;
	}
	if(Motor2_PWM<1000)
	{
		Motor2_PWM=1000;
	}
	if(Motor3_PWM<1000)
	{
		Motor3_PWM=1000;
	}
	if(Motor4_PWM<1000)
	{
		Motor4_PWM=1000;
	}
	
	if(Motor1_PWM>2000)
	{
		Motor1_PWM=2000;
	}
	if(Motor2_PWM>2000)
	{ 
		Motor2_PWM=2000;
	}
	if(Motor3_PWM>2000)
	{
		Motor3_PWM=2000;
	}
	if(Motor4_PWM>2000)
	{
		Motor4_PWM=2000;
	}
	Set_Speed(MOTOR1,(uint16_t)Motor1_PWM);
	Set_Speed(MOTOR2,(uint16_t)Motor2_PWM);
	Set_Speed(MOTOR3,(uint16_t)Motor3_PWM);
	Set_Speed(MOTOR4,(uint16_t)Motor4_PWM);//最终输出到电机
	return ;
}


/**
  * @brief 这是双环PID控制器，由于单环是使用的PID结构体来实现，为了减少改动，现在外环仍然采取PID结构体，
  *		 但内环的相关参数由静态变量来替代
  * @args
  * @returnval
  */



void PID_Controller_DL(void)
{
	Oil_PWM = PPM_Data[3];
	PID.Pitch_Tar = (PPM_Data[2]-1500)/33.333f;
	PID.Roll_Tar  = (PPM_Data[4]-1500)/33.333f;
	
	float temp;
	
	//PID的内环参数,使用静态变量
	static float P_Err_Last=0,P_Err_Sum=0,P_Err_Last_In=0,P_Err_Sum_In=0;
	float P_Err_Now;
	P_Err_Now = angle.pitch - PID.Pitch_Tar;
	
		P_Err_Sum += P_Err_Now;
	
		temp = Kp_IN * P_Err_Now + Ki_IN * P_Err_Sum + Kd_IN * ( P_Err_Now - P_Err_Last );
		P_Err_Last =  P_Err_Now;
		P_Err_Now = Wx - temp;
		temp = Kp_OUT * P_Err_Now + Ki_OUT * P_Err_Sum_In + Kd_OUT * ( P_Err_Now - P_Err_Last_In );
		P_Err_Last_In = P_Err_Now;
		Pitch_PWM = -(int16_t)temp;
	
	//同上
	static float R_Err_Last=0,R_Err_Sum=0,R_Err_Last_In=0,R_Err_Sum_In=0;
	float R_Err_Now;
	R_Err_Now = angle.roll - PID.Roll_Tar;
	
		R_Err_Sum += R_Err_Now;
	
		temp = Kp_IN * R_Err_Now + Ki_IN * R_Err_Sum + Kd_IN * ( R_Err_Now - R_Err_Last );
		R_Err_Last =  R_Err_Now;
		R_Err_Now = Wy - temp;
		temp = Kp_OUT * R_Err_Now + Ki_OUT * R_Err_Sum_In + Kd_OUT * ( R_Err_Now - R_Err_Last_In );
		R_Err_Last_In = R_Err_Now;
		Roll_PWM = -(int16_t)temp;
	
	
	
	//同上
	static float Y_Err_Last=0,Y_Err_Sum=0,Y_Err_Last_In=0,Y_Err_Sum_In=0;
	float Y_Err_Now;
	
	Y_Err_Now = angle.yaw - 0.00f;
	Y_Err_Sum += Y_Err_Now;
	
	temp = Kp_IN * Y_Err_Now + Ki_IN * Y_Err_Sum + Kd_IN * ( Y_Err_Now - Y_Err_Last );
	Y_Err_Last_In = Y_Err_Now;
	Yaw_PWM = -(int16_t)(0.05f *temp);
	
	Motor1_PWM = Oil_PWM + Roll_PWM - Pitch_PWM + Yaw_PWM;
	Motor2_PWM = Oil_PWM - Roll_PWM - Pitch_PWM - Yaw_PWM;
	Motor3_PWM = Oil_PWM + Roll_PWM + Pitch_PWM - Yaw_PWM;
	Motor4_PWM = Oil_PWM - Roll_PWM + Pitch_PWM + Yaw_PWM;
	
	

	if(Motor1_PWM<1000)//防止出现非法范围内的CCR值
	{
		Motor1_PWM=1000;
	}
	if(Motor2_PWM<1000)
	{
		Motor2_PWM=1000;
	}
	if(Motor3_PWM<1000)
	{
		Motor3_PWM=1000;
	}
	if(Motor4_PWM<1000)
	{
		Motor4_PWM=1000;
	}
	
	if(Motor1_PWM>2000)
	{
		Motor1_PWM=2000;
	}
	if(Motor2_PWM>2000)
	{ 
		Motor2_PWM=2000;
	}
	if(Motor3_PWM>2000)
	{
		Motor3_PWM=2000;
	}
	if(Motor4_PWM>2000)
	{
		Motor4_PWM=2000;
	}
	Set_Speed(MOTOR1,(uint16_t)Motor1_PWM);//调整电机转速
	Set_Speed(MOTOR2,(uint16_t)Motor2_PWM);
	Set_Speed(MOTOR3,(uint16_t)Motor3_PWM);
	Set_Speed(MOTOR4,(uint16_t)Motor4_PWM);
	return ;
}



















