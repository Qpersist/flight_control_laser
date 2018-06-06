#ifndef _ULTRASONIC_H
#define _ULTRASONIC_H

#include "stm32f4xx.h"
#include "ms5611.h"


#define DEAD_BAND(value,mid,ban) (((value)<(mid)-(ban))?((value)+(ban)):(((value)>(mid)+(ban))?((value)-(ban)):(mid)))


 struct _tache
{
	float hight_pid_out;
	
	float hight_pid_error;
	float hight_pid_merrer;
	float hight_pid_deriv;
	
	float hight_pid_KP_out;
	float hight_pid_KI_out;
	float hight_pid_KD_out;
	float hight_pid_KD_out_old;
	
	float hight_pid_error_old;
	float set_hight;
};
struct _hight_ctrl
{
		struct _tache core;
		struct _tache shell;

};

extern struct _hight_ctrl hight_PID_ctrl;
void Ultrasonic_Init(void);
void Ultra_Duty(void);
void Ultra_Get(u8);
void hight_ctrl(int exp_hight);
void hubu_Ctrl(void);

extern s8 ultra_start_f;

extern _height_st ultra;

#endif


