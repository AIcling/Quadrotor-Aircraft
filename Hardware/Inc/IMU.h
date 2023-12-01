#ifndef __IMU_H
#define __IMU_H

struct Qfour{
    float q0;
    float q1;
    float q2;
    float q3;
} ;

struct Angle{
    float pitch;
    float roll;
    float yaw;
};

void CalAngle(struct Angle *angle, float time);
void IMU_Init(void);
extern struct Qfour QNum;
extern struct Angle angle;
extern float Gyro_Xerr,Gyro_Yerr,Gyro_Zerr;
extern float Acc_Xerr,Acc_Yerr,Acc_Yerr;
extern float Xoffset,Yoffset,Zoffset,Kmx,Kmy,Kmz;
void get_MPU6050err(void);
void HMC5883L_Motify(void);
#define Use_Mag 1

#endif