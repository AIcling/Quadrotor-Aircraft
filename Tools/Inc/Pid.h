#ifndef      __PID_H__
#define      __PID_H__

#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3
#define MOTOR4 4

typedef struct
{
	float	Pitch_Tar;
	float	Roll_Tar;
	float	Yaw_Tar;
	float	Pitch_Err_Now;
	float	Pitch_Err_Last;
	float	Pitch_Err_All;
	float	Roll_Err_Now;
	float	Roll_Err_Last;
	float	Roll_Err_All;
	float	Yaw_Err_Now;
	float	Yaw_Err_Last;
	float	Yaw_Err_All;
}PID_Ctrl;

extern float Kp;
extern float Ki;
extern float Kd;


extern float Kp_IN;
extern float Ki_IN;
extern float Kd_IN;
extern float Kp_OUT;
extern float Ki_OUT;
extern float Kd_OUT;


void PID_Controller(void);

void PID_Controller_DL(void);



#endif

