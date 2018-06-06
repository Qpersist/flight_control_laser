#include "fly_mode.h"
#include "rc.h"
#include "ano_of.h"


u8 mode_value[10];
u8 mode_state,mode_state_old;

u8 change_Hight_data;
int AUX1_data,AUX1_data_old;
int AUX2_data,AUX2_data_old;
u8 change_CCD_data =0;
void mode_check(float *ch_in,u8 *mode_value)
{

	if(Switch2.flag ==0 )
	{
		if(*(ch_in+AUX1) <-200)
		{
			mode_state = 0;//0;
		}
		else if(*(ch_in+AUX1) >200  )
		{
			mode_state = 2;
		}
		else
		{
			mode_state = 1;
		}
		
		AUX2_data = *(ch_in+AUX2);
		if((AUX2_data > (AUX2_data_old+500)) && OF_ALT2<=30)
		{
			change_Hight_data =1;
		}

		if(AUX2_data < AUX2_data_old-500)
			change_Hight_data =0; 
		
		AUX2_data_old = AUX2_data;
		
		if(*(ch_in+AUX3) <0)
			change_CCD_data = 0;
		else 
			change_CCD_data = 1;
	
	
	}
	 else
	 {
	AUX1_data =*(ch_in+AUX1);
	
		if(AUX1_data < AUX1_data_old -100) 
		{
			Switch2.count_mode = 0; 
			Switch2.count_unlock= 0;
			Switch2.flag = 0;
		}
	 AUX1_data_old  =  AUX1_data;
	}
	//=========== GPS、气压定高 ===========
	if(mode_state == 0 )
	{
		*(mode_value+GPS) = *(mode_value+BARO) = 0;
	}
	else
	{
		*(mode_value+GPS) = *(mode_value+BARO) = 1;
	}
	
//	//=========== 返航模式 ===========
//	if(fly_ready )
//	{
//		if(( mode_state == 2 && mode_state_old != 2) || rc_lose == 1)
//		{

//			*(mode_value+BACK_HOME) = 1;
//			

//		}
//		else if(mode_state != 2)
//		{
//			*(mode_value+BACK_HOME) = 0;
//		}
//	}
//	else
//	{
//		*(mode_value+BACK_HOME) = 0;
//	}
	
	
 
	//===========   ===========
	mode_state_old = mode_state; //历史模式
	flag.FlightMode = mode_state;
}
