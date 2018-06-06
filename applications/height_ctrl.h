#ifndef __HEIGHT_CTRL_H
#define __HEIGHT_CTRL_H

#include "stm32f4xx.h"

#include "parameter.h"

//typedef xyz_f_t _xyz_f_t;
#define _xyz_f_t xyz_f_t

#define THR_TAKE_OFF_LIMIT 550
/*************************************/

#define TT                    0.002f //油门控制周期2.5ms，
#define Ultrasonic_MAX_Height 2500   //超声波最高有效高度，单位是mm
#define Ultrasonic_MIN_Height 200    //超声波最低有效高度，单位是mm

extern float height_ctrl_out;
enum
{
	unknow=0,
	ziwen,
	ULTRASONIC_High
};


 struct _st_height_pid_v
{
	float err;
	float err_old;
	float err_d;
	float err_i;
	float pid_out;

};

 struct _st_height_pid
{
	float kp;
	float kd;
	float ki;

};

typedef struct
{
	struct _st_height_pid_v ultra_ctrl;
    struct _st_height_pid ultra_pid;
	struct _st_height_pid ultra_wz_speed_pid;
	
	struct _st_height_pid wz_speed_pid;
	struct _st_height_pid_v wz_speed_pid_v;

	struct _st_height_pid_v baro_ctrl;
	struct _st_height_pid baro_pid;
	struct _st_height_pid baro_wz_speed_pid;
	
	
}_HT_HAWK;

struct Flag
{
	u8 FlightMode;
};
extern struct Flag flag;

/*****************************************************/
#define ACC_SPEED_NUM 50


typedef struct
{
	float m_acc;
	float m_speed;
	float m_height;
	float fusion_acc;
	float fusion_speed;
	float fusion_height;

}_hc_value_st;
extern _hc_value_st hc_value;

float auto_take_off_land(float dT,u8 ready);
float Height_Ctrl(float T,float thr,u8 ready,float en);
void h_pid_init(void);



void HT_Baro_PID_Init(void);
void HT_WZ_Speed_PID_Init(void);
void HT_Ultra_PID_Init(void);
void LockForKeepHigh(float THROTTLE);
void Ultra_dataporcess(float T);
void height_speed_ctrl(float T,float thr_keepheight,float exp_z_speed,float h_speed);
void Ultra_Ctrl(float T,float thr);
void HT_Height_Ctrl(float T,float thr);



#endif

