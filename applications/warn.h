#ifndef _warn_H
#define _warn_H

#include "include.h"



#define KEY0 		GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_2) //PE4
#define KEY1 		GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_3)	//PE3 
#define KEY2 		GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_4) //PE2


struct _Switch
{
	u8 mode;
	u8 count_mode ;
	u8 mode_key;
	
	u8 flag;
	u8 count_unlock;
	
	float time;
	
};

extern struct _Switch Switch,Switch2;


void light_sound_init(void);
void set_light_sound(u8 mode);
void switch_init(void);
 void Auto_switch_2(float T);
void CALIBRATE_duty(float T);


#endif




