#ifndef _Trance_H_
#define _Trance_H_


#include "include.h"
//#include "Sys.h"

 struct _CCD_PID
{
	float data;
	float data_old;
	
	float error;
	float error_old;
	float merrer;
	float devir;
	
	float CCD_death;
	float CCD_death_old;
	
	float angle_death;
	float roll_old;
	float Roll_speed;
	float Roll_speed_old;
	float Roll_V_death;
	
	float KP_out;
	float KI_out;
	float KD_out;
	
	float PID_out;
	
	float width;
	float shell_error;
	float shell_Pout;
	
};
 struct _CCD_PID_H
{
	float data;
	float data_old;
	
	float error;
	float error_old;
	float merrer;
	float devir;
	
	float CCD_death;
	float CCD_death_old;
	
	float angle_death;
	float roll_old;
	float Roll_speed;
	float Roll_speed_old;
	float Roll_V_death;
	
	float KP_out;
	float KI_out;
	float KD_out;
	
	float PID_out;
	
	float width;
	float shell_error;
	float shell_Pout;
	
};
 struct _camera_Tance
{
	float data;
	float data_V;
	float data_old;
	
	float error_H;
	float error_H_old;
	float merrer_H;
	float devir_H;
	float error_V;
	float error_V_old;
	float merrer_V;
	float devir_V;
	
	float angle_death;

	
	float KP_out_H;
	float KI_out_H;
	float KD_out_H;
	
	float KP_out_V;
	float KI_out_V;
	float KD_out_V;
	
	float PID_out_H;
	float PID_out_V;
	
	float width;
	int count;
	int flag;
	
	float shell_error;
	float shell_Pout;
	
	u8 direction;
	u8 take_off_flag;
	u8 trance_flag;
	u8 trance_flag_old;
	u8 sound;
};


struct _Duty
{
	u8 flag;
	u8 mode ;
	u8 duty_mode;
	u8 fly_mode ;
	
	float time;
	float time_cycle;
	
};
extern struct _CCD_PID  CCD_CTRL_PID;
extern struct _CCD_PID_H CCD_CTRL_PID_H;
extern struct _camera_Tance camera_Trance;
extern struct _Duty Duty; 


 void CCD_ctrl(u8 mid,float T,u8 mode);

void direction_control(float T,u8 duty_mode);
void mode_0_duty(float T,u8 mode);
void mode_1_duty(float T,u8 mode);
void mode_2_duty(float T,u8 mode);
void feed_error(int Ori_H_error,int Ori_V_error, int *H_error,int *V_error,float Hight);
void camera_shell_pid(void);


#endif



