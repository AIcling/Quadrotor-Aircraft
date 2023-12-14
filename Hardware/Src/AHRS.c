#include "IMU.h"
#include "HMC5883L.h"
#include "MPU6050.h"
#include <math.h>
#include "delay.h"
#include "Matrix.h"
const float Pic = 180.0/3.14159624;
const float K = 4000.0/65535;
const float Kg = 4.0/65535;
// const float Alpha = 0.2f;

extern u32 betime, aftime;

struct {
    float x;
    float y;
    float z;
} Gyro;

struct Qfour QNum = {1,0,0,0};
struct Angle angle = {0,0,0};
float e0,e1,e2,e3 ;
float Gyro_Xerr,Gyro_Yerr,Gyro_Zerr = 0.0f;
float Acc_Xerr,Acc_Yerr,Acc_Yerr = 0.0f;
float Wx,Wy,Wz=0.0f;

#ifdef Use_Mag
void CalAngle(struct Angle *angle, float time){
  float Mx = (HMC5883L_data[0]-Ox)/Rx/1090;
  float My = (HMC5883L_data[1]-Oy)/Ry/1090;
  float Mz = (HMC5883L_data[2]-Oz)/Rz/1090;
   Wx = (MPU6050_data[4]*K/Pic);
   Wy = (MPU6050_data[5]*K/Pic);
   Wz = (MPU6050_data[6]*K/Pic);
  float Ax = MPU6050_data[0] * Kg;
  float Ay = MPU6050_data[1] * Kg;
  float Az = MPU6050_data[2] * Kg;
  float _2q0, _2q1, _2q2, _2q3, q0q0,q0q1,q0q2,q0q3,_2q0q2, q1q1, q1q2,q2q2, q2q3,q3q3,_2q2q3,q1q3,_2q0Mx,_2q0My,_2q0Mz,_2q1Mx,hx,hy,_2bx,_2bz,_4bx,_4bz ;
//加速度归一化
  float sum_ac = sqrt(pow(Ax,2)+pow(Ay,2)+pow(Az,2));
  Ax = Ax/sum_ac;
  Ay = Ay/sum_ac;
  Az = Az/sum_ac;
//磁力计
   _2q0Mx = 2.0f * QNum.q0 * Mx;
   _2q0My = 2.0f * QNum.q0 * My;
   _2q0Mz = 2.0f * QNum.q0 * Mz;
   _2q1Mx = 2.0f * QNum.q1 * Mx;
//加速度
		_2q0 = 2.0f * QNum.q0;
		_2q1 = 2.0f * QNum.q1;
		_2q2 = 2.0f * QNum.q2;
		_2q3 = 2.0f * QNum.q3;
		q0q0 = QNum.q0 * QNum.q0;
    q0q1 = QNum.q0 * QNum.q1;
    q0q2 = QNum.q0 * QNum.q2;
    _2q0q2 = 2.0f * QNum.q2 * QNum.q0;
		q1q1 = QNum.q1 * QNum.q1;
    q1q2 = QNum.q1 * QNum.q2;
    q0q3 = QNum.q0 * QNum.q3;
    q1q3 = QNum.q1 * QNum.q3;
		q2q2 = QNum.q2 * QNum.q2;
    q2q3 = QNum.q2 * QNum.q3;
		q3q3 = QNum.q3 * QNum.q3;
    _2q2q3 = 2.0f * QNum.q2 * QNum.q3;
//磁场
    hx = Mx*q0q0 - _2q0My*QNum.q3 + _2q0Mz*QNum.q2 + Mx*q1q1 + My*_2q1*QNum.q2 + Mz*_2q1*QNum.q3 - Mx*q2q2 - Mx*q3q3;
    hy = _2q0Mx * QNum.q3 + My * q0q0 - _2q0Mz * QNum.q1 + _2q1Mx * QNum.q2 - My * q1q1 + My * q2q2 + _2q2 * Mz * QNum.q3 - My * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0Mx * QNum.q2 + _2q0My * QNum.q1 + Mz * q0q0 + _2q1Mx * QNum.q3 - Mz * q1q1 + _2q2 * My * QNum.q3 - Mz * q2q2 + Mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;
   
//加速度矫正误差
		e0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - Ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - Ay) - _2bz * QNum.q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - Mx) + (-_2bx * QNum.q3 + _2bz * QNum.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - My) + _2bx * QNum.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - Mz);
		e1 = _2q3 * (2.0f * q1q3 - _2q0q2 - Ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - Ay) - 4.0f * QNum.q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - Az) + _2bz * QNum.q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - Mx) + (_2bx * QNum.q2 + _2bz * QNum.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - My) + (_2bx * QNum.q3 - _4bz * QNum.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - Mz);
		e2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - Ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - Ay) - 4.0f * QNum.q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - Az) + (-_4bx * QNum.q2 - _2bz *  QNum.q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - Mx) + (_2bx * QNum.q1 + _2bz * QNum.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - My) + (_2bx * QNum.q0 - _4bz * QNum.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - Mz);
		e3 = _2q1 * (2.0f * q1q3 - _2q0q2 - Ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - Ay) + (-_4bx * QNum.q3 + _2bz * QNum.q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - Mx) + (-_2bx *  QNum.q0 + _2bz * QNum.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - My) + _2bx * QNum.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - Mz);
if(e0!=0||e1!=0||e2!=0||e3!=0)
{
float err_sum = sqrt(e0*e0+e1*e1+e2*e2+e3*e3);
e0 = e0/err_sum;
e1 = e1/err_sum;
e2 = e2/err_sum;
e3 = e3/err_sum;
}
float Alpha = 0.1;
//四元数更新
  QNum.q0 = QNum.q0 - 0.006*(Wx*QNum.q1+Wy*QNum.q2+Wz*QNum.q3)- Alpha*e0;
  QNum.q1 = QNum.q1 + 0.006*(Wx*QNum.q0-Wy*QNum.q3+Wz*QNum.q2)- Alpha*e1;
  QNum.q2 = QNum.q2 + 0.006*(Wx*QNum.q3+Wy*QNum.q0-Wz*QNum.q1)- Alpha*e2;
  QNum.q3 = QNum.q3 + 0.006*(Wy*QNum.q1-Wx*QNum.q2+Wz*QNum.q0)- Alpha*e3;
//四元数归一化
  float sum = sqrt(pow(QNum.q0,2)+pow(QNum.q1,2)+pow(QNum.q2,2)+pow(QNum.q3,2));
  QNum.q0 = QNum.q0/sum;
  QNum.q1 = QNum.q1/sum;
  QNum.q2 = QNum.q2/sum;
  QNum.q3 = QNum.q3/sum;

//四元数转欧拉角 Yaw->Pitch->Roll
  angle->pitch =  asin(-2*(QNum.q2*QNum.q3-QNum.q0*QNum.q1));
  angle->yaw = atan((2*(QNum.q1*QNum.q2+QNum.q0*QNum.q3))/(pow(QNum.q0,2)-pow(QNum.q1,2)+pow(QNum.q2,2)-pow(QNum.q3,2)));
  angle->roll = atan((2*(QNum.q1*QNum.q3+QNum.q0*QNum.q2))/(pow(QNum.q0,2)-pow(QNum.q1,2)-pow(QNum.q2,2)+pow(QNum.q3,2)));
}

float Xoffset,Yoffset,Zoffset,Kmx,Kmy,Kmz;
void HMC5883L_Motify(void){
  HMC588CL_ReadData(HMC5883L_data);
  float Xmin,Xmax = HMC5883L_data[0];
  float Ymin,Ymax = HMC5883L_data[1];
  float Zmin,Zmax = HMC5883L_data[2];

  int i = 200;
  while(i--){
    HMC588CL_ReadData(HMC5883L_data);
    if(Xmin>HMC5883L_data[0]) Xmin = HMC5883L_data[0];
    if(Xmax<HMC5883L_data[0]) Xmax = HMC5883L_data[0];
    if(Ymin>HMC5883L_data[1]) Ymin = HMC5883L_data[1];
    if(Ymax<HMC5883L_data[1]) Ymax = HMC5883L_data[1];    
    if(Zmin>HMC5883L_data[2]) Zmin = HMC5883L_data[2];
    if(Zmax<HMC5883L_data[2]) Zmax = HMC5883L_data[2]; 
    Delay_ms(100);    
  }
  Xoffset = (Xmax+Xmin)/2;
  Yoffset = (Ymax+Ymin)/2;
  Zoffset = (Zmax+Zmin)/2;
  Kmx = 2/(Xmax-Xmin);
  Kmy = 2/(Ymax-Ymin);
  Kmz = 2/(Zmax-Zmin);
}
#else
void CalAngle(struct Angle *angle, float time){
  float Wx = ((MPU6050_data[4]-Gyro_Xerr)*K/Pic);
  float Wy = ((MPU6050_data[5]-Gyro_Yerr)*K/Pic);
  float Wz = ((MPU6050_data[6]-Gyro_Zerr)*K/Pic);
  float Ax = MPU6050_data[0] * Kg;
  float Ay = MPU6050_data[1] * Kg;
  float Az = MPU6050_data[2] * Kg;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
  float err_sum = 0;
//加速度归一化
  float sum_ac = sqrt(pow(Ax,2)+pow(Ay,2)+pow(Az,2));
  Ax = Ax/sum_ac;
  Ay = Ay/sum_ac;
  Az = Az/sum_ac;
//加速度
		_2q0 = 2.0f * QNum.q0;
		_2q1 = 2.0f * QNum.q1;
		_2q2 = 2.0f * QNum.q2;
		_2q3 = 2.0f * QNum.q3;
		_4q0 = 4.0f * QNum.q0;
		_4q1 = 4.0f * QNum.q1;
		_4q2 = 4.0f * QNum.q2;
		_8q1 = 8.0f * QNum.q1;
		_8q2 = 8.0f * QNum.q2;
		q0q0 = QNum.q0 * QNum.q0;
		q1q1 = QNum.q1 * QNum.q1;
		q2q2 = QNum.q2 * QNum.q2;
		q3q3 = QNum.q3 * QNum.q3;
//加速度矫正误差
 e0 = _4q0 * q2q2 + _2q2 * Ax + _4q0 * q1q1 - _2q1 * Ay;
 e1 = _4q1 * q3q3 - _2q3 * Ax + 4.0f * q0q0 * QNum.q1 - _2q0 * Ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * Az;
 e2 = 4.0f * q0q0 * QNum.q2 + _2q0 * Ax + _4q2 * q3q3 - _2q3 * Ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * Az;
 e3 = 4.0f * q1q1 * QNum.q3 - _2q1 * Ax + 4.0f * q2q2 * QNum.q3 - _2q2 * Ay;
if(e0!=0||e1!=0||e2!=0||e3!=0)
{
 err_sum = sqrt(e0*e0+e1*e1+e2*e2+e3*e3);
e0 = e0/err_sum;
e1 = e1/err_sum;
e2 = e2/err_sum;
e3 = e3/err_sum;
}
  //  float Alpha = 0.02f+0.02f*err_sum;
   float Alpha = 0.1;
//四元数更新
  QNum.q0 = QNum.q0 - 0.007*(Wx*QNum.q1+Wy*QNum.q2+Wz*QNum.q3- Alpha*e0);
  QNum.q1 = QNum.q1 + 0.007*(Wx*QNum.q0-Wy*QNum.q3+Wz*QNum.q2- Alpha*e1);
  QNum.q2 = QNum.q2 + 0.007*(Wx*QNum.q3+Wy*QNum.q0-Wz*QNum.q1- Alpha*e2);
  QNum.q3 = QNum.q3 + 0.007*(Wy*QNum.q1-Wx*QNum.q2+Wz*QNum.q0- Alpha*e3);
//四元数归一化
  float sum = sqrt(pow(QNum.q0,2)+pow(QNum.q1,2)+pow(QNum.q2,2)+pow(QNum.q3,2));
  QNum.q0 = QNum.q0/sum;
  QNum.q1 = QNum.q1/sum;
  QNum.q2 = QNum.q2/sum;
  QNum.q3 = QNum.q3/sum;

//四元数转欧拉角 Yaw->Pitch->Roll
  // angle->pitch =  asin(-2*(QNum.q2*QNum.q3-QNum.q0*QNum.q1));
  // angle->yaw = atan((2*(QNum.q1*QNum.q2+QNum.q0*QNum.q3))/(pow(QNum.q0,2)-pow(QNum.q1,2)+pow(QNum.q2,2)-pow(QNum.q3,2)));
  // angle->roll = atan((2*(QNum.q1*QNum.q3+QNum.q0*QNum.q2))/(pow(QNum.q0,2)-pow(QNum.q1,2)-pow(QNum.q2,2)+pow(QNum.q3,2)));
  angle->roll = -asin(2*QNum.q1*QNum.q3-2*QNum.q0*QNum.q2);
  angle->pitch = atan((2*QNum.q0*QNum.q1+2*QNum.q2*QNum.q3)/(1-2*QNum.q1*QNum.q1-2*QNum.q2*QNum.q2));
  angle->yaw = atan((2*QNum.q0*QNum.q3-2*QNum.q1*QNum.q2)/(1-2*QNum.q2*QNum.q2-2*QNum.q3*QNum.q3));
}

#endif  //Use_Mag











