#include "stm32f4xx.h"

#include "Mahony.h"
// #include "Gy86.h"  //引入原始数据的头文件
#include <stdio.h>
#include <math.h>

#define  ACC_RANGE 2.0f
#define  GYRO_RANGE 2000.0f
#define  GRAVITY   9.8f
#define  RAD 0.0174f

Angle MyAngle={0};
IMU MPU6050={0};

void Prepare_RawData(IMU *Trans_Data)
{
	Trans_Data->Acc_X=(float)MPU6050_data[0]-906.4f;
	Trans_Data->Acc_Y=(float)MPU6050_data[1]+187.6f;
	Trans_Data->Acc_Z=(float)MPU6050_data[2]+1295.4f;
	Trans_Data->Gyro_X=(((float)MPU6050_data[4]+49.59f)*GYRO_RANGE*RAD)/32768;
	Trans_Data->Gyro_Y=(((float)MPU6050_data[5]-27.71f)*GYRO_RANGE*RAD)/32768;
	Trans_Data->Gyro_Z=(((float)MPU6050_data[6]-18.70f)*GYRO_RANGE*RAD)/32768;
	return ;
}

float GetInvSqrt(float Sum)
{
	return 1.0f/sqrt(Sum);
}

#define KP 0.8f
#define KI 0.001f
#define T 0.02f
#define HALF_T 0.01f

float exInt=0.0f ,eyInt=0.0f,ezInt=0.0f;
float q0=1.0f, q1=0.0f, q2=0.0f, q3=0.0f;


void Mahony_Update_Attitude(IMU *Trans_Data,Angle *Att_Data)
{
	float Matrix[9]={1,0,0,0,1,0,0,0,1};
	float ax= Trans_Data->Acc_X ,ay=Trans_Data->Acc_Y, az=Trans_Data->Acc_Z;
	float gx= Trans_Data->Gyro_X,gy=Trans_Data->Gyro_Y,gz=Trans_Data->Gyro_Z;
	float vx,vy,vz;
	float ex,ey,ez;
	float norm;
	
	//先提前计算出来，避免在表达式那一块再次计算，提高效率
	float q0q0=q0*q0;
	float q0q1=q0*q1;
	float q0q2=q0*q2;
	float q0q3=q0*q3;
	float q1q1=q1*q1;
	float q1q2=q1*q2;
	float q1q3=q1*q3;
	float q2q2=q2*q2;
	float q2q3=q2*q3;
	float q3q3=q3*q3;
	
	//重力加速度归一化
	norm=GetInvSqrt(ax*ax + ay*ay + az*az);
	ax=ax * norm;
	ay=ay * norm;
	az=az * norm;
	
	//printf("ACC:%.2f %.2f %.2f\r\n",ax,ay,az);
	
	//这是归一化之后的结果
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = 1-2*q1q1-2*q2q2;
	//vz = q0q0-q1q1-q2q2+q3q3;//这两种写法是等价的
	
	//printf("GYR:%.2f %.2f %.2f\r\n",gx,gy,gz);
	//printf("four:%.2f %.2f %.2f %.2f\r\n",q0,q1,q2,q3);
	
	//利用叉乘算出误差
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	//printf("ERR:%.2f %.2f %.2f\r\n",ex,ey,ez);
	
	//对误差进行积分
	exInt = exInt + ex * KI ;
	eyInt = eyInt + ey * KI ;
	ezInt = ezInt + ez * KI ;
	
	//将误差补偿至角速度上
	gx = gx + KP*ex + exInt;
	gy = gy + KP*ey + eyInt;
	gz = gz + KP*ez + ezInt;
	
	//printf("GYR2:%.2f %.2f %.2f\r\n",gx,gy,gz);
	//printf("\r\n\r\n\r\n");
	
	//更新四元数
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*HALF_T;
	q1 = q1 + ( q0*gx + q2*gz - q3*gy)*HALF_T;
	q2 = q2 + ( q0*gy - q1*gz + q3*gx)*HALF_T;
	q3 = q3 + ( q0*gz + q1*gy - q2*gx)*HALF_T;
	
	//由于计算精度问题，原本被归一化的数据会越来越不规范（偏离了归一化时的情况）
	//所以需要在每次更新四元数过后再手动归一化一次
	norm = GetInvSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
	
	//更新旋转矩阵
	Matrix[0]=q0q0 + q1q1 - q2q2 - q3q3;
	Matrix[1]=2.0f*(q1q2 + q0q3);
	Matrix[2]=2.0f*(q1q3 - q0q2);
	Matrix[3]=2.0f*(q1q2 - q0q3);
	Matrix[4]=q0q0 - q1q1 + q2q2 - q3q3;
	Matrix[5]=2.0f*(q2q3 + q0q1);
	Matrix[6]=2.0f*(q1q3 + q0q2);
	Matrix[7]=2.0f*(q2q3 - q0q1);
	Matrix[8]=q0q0-q1q1-q2q2+q3q3;
	
	Matrix[0]-=0.001f;//消除编译后的警告
	
	//提取旋转矩阵中我们需要的数据，进行解算
	float g1,g2,g3,g4,g5;
	g1=2.0f * (q1*q3 - q0*q2);
	g2=2.0f * (q0*q1 + q2*q3);
	g3=(q0*q0-q1*q1-q2*q2+q3*q3);
	g4=2.0f * (q1*q2 + q0*q3);
	g5=q0*q0 + q1*q1 -q2*q2 - q3*q3;
	
	Att_Data->pitch  = -asin(g1)    * 57.296f;
	Att_Data->roll = atan2(g2,g3) * 57.296f;
	Att_Data->yaw   = atan2(g4,g5) * 57.296f;
	
	return ;
}


#define BETA  0.02


void Madgwick_Update_Attitude(IMU *Trans_Data,Angle *Att_Data)
{
	float ax = Trans_Data->Acc_X ,ay=Trans_Data->Acc_Y, az=Trans_Data->Acc_Z;
	float gx = Trans_Data->Gyro_X,gy=Trans_Data->Gyro_Y,gz=Trans_Data->Gyro_Z;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0,_2q1,_2q2,_2q3,_4q0,_4q1,_4q2,_8q1,_8q2,q0q0,q1q1,q2q2,q3q3;
	float e0,e1,e2,e3;
	float norm;
	
	qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
	qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy);
	qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx);
	qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx);
	
	norm = GetInvSqrt(ax*ax + ay*ay + az*az);
	ax=ax * norm;
	ay=ay * norm;
	az=az * norm;
	
	_2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;
	
	e0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    e1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    e2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    e3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
	
	norm = GetInvSqrt(e0*e0 + e1*e1 + e2*e2 + e3*e3); 
    e0 = e0 * norm;
    e1 = e1 * norm;
    e2 = e2 * norm;
    e3 = e3 * norm;
	
	qDot1 -= BETA * e0;
    qDot2 -= BETA * e1;
    qDot3 -= BETA * e2;
    qDot4 -= BETA * e3;
	
	q0 += qDot1 * T;
    q1 += qDot2 * T;
    q2 += qDot3 * T;
    q3 += qDot4 * T;
	
	norm = GetInvSqrt( q0*q0 + q1*q1 + q2*q2 + q3*q3 );
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
	
	float g1,g2,g3,g4,g5;
	g1=2.0f * (q1*q3 - q0*q2);
	g2=2.0f * (q0*q1 + q2*q3);
	g3= q0*q0 - q1*q1 - q2*q2 + q3*q3;
	g4=2.0f * (q1*q2 + q0*q3);
	g5= q0*q0 + q1*q1 - q2*q2 - q3*q3;
	
	Att_Data->roll  = -asin(g1)    * 57.296f;
	Att_Data->pitch = atan2(g2,g3) * 57.296f;
	Att_Data->yaw   = atan2(g4,g5) * 57.296f;
	
	return ;
}











