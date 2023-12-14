#ifndef      __FILTER_H__
#define      __FILTER_H__
#include "MPU6050.h"

typedef struct
{
	float Acc_X;
	float Acc_Y;
	float Acc_Z;
	float Gyro_X;
	float Gyro_Y;
	float Gyro_Z;
}IMU;

typedef struct 
{
	float pitch;
	float roll;
	float yaw;
}Angle;

extern float q0;
extern float q1;
extern float q2;
extern float q3;

extern Angle MyAngle;
extern IMU MPU6050;


void Prepare_RawData(IMU *Trans_Data);
void Mahony_Update_Attitude(IMU *Trans_Data,Angle *Att_Data);
void Madgwick_Update_Attitude(IMU *Trans_Data,Angle *Att_Data);


#endif

