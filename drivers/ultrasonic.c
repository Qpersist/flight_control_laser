#include "include.h"
#include "ultrasonic.h"
#include "usart.h"
#include "mymath.h"
#include "time.h"
#include "imu.h"
#include "fly_mode.h"


void Ultrasonic_Init()
{
//  Uart5_Init(9600);			//串口5初始化，函数参数为波特率
	
}

s8 ultra_start_f;
s8 ultra_start_ok;
void Ultra_Duty()
{
	u8 temp[3];

	ultra.h_dt = 0.05f; //50ms一次
/*
		UART5->DR = 0xe8;   //ks103地址（可设置）
		while( (UART5->SR & USART_FLAG_TXE) == 0 );
		
		UART5->DR = 0x02;   //++++
		while( (UART5->SR & USART_FLAG_TXE) == 0 );

		UART5->DR = 0xbc;  //70ms,带温度补偿
		while( (UART5->SR & USART_FLAG_TXE) == 0 );
*/	
	#if defined(USE_KS103)
		temp[0] = 0xe8;
		temp[1] = 0x02;
		temp[2] = 0xbc;
		Uart2_Send(temp ,3);
	#elif defined(USE_US100)
		temp[0] = 0x55;
		Uart2_Send(temp ,1);
		//Usart1_Send(temp ,1);
	#endif
///////////////////////////////////////////////
		ultra_start_f = 1;

		if(ultra.measure_ot_cnt<200) //200ms
		{
			ultra.measure_ot_cnt += ultra.h_dt *1000;
		}
		else
		{
			ultra.measure_ok = 0;//超时，复位
		}
}

u16 ultra_distance_old;

_height_st ultra;

void AD_Ultra_GET(u8 channel)
{
	uint16_t distance;
	distance = Get_Adc(channel) *  0.75 * 3.3/5;
	ultra.relative_height  = distance;
	
	ultra_distance_old = ultra.relative_height;
}

void Ultra_Get(u8 com_data)
{
	static u8 ultra_tmp;
	
	if( ultra_start_f == 1 )
	{
		ultra_tmp = com_data;
		ultra_start_f = 2;
	}
	else if( ultra_start_f == 2 )
	{
		ultra.height =  ((ultra_tmp<<8) + com_data)/10;
		
		if(ultra.height < 500) // 5米范围内认为有效，跳变值约10米.
		{
//			ultra.relative_height = ultra.height;
			ultra.measure_ok = 1;
			ultra.start_ok=1;
		}
		else
		{
			ultra.height = ultra_distance_old;
			ultra.measure_ok = 2; //数据超范围
		}

		ultra_start_f = 0;
	}
	ultra.measure_ot_cnt = 0; //清除超时计数（喂狗）
	
	ultra.h_delta = ultra.height - ultra_distance_old;

/**********************
滤波
	
*************************/	
	if((ultra.h_delta > 50)||(ultra.h_delta < -50))
	{
		ultra.height = ultra_distance_old;
	}
	if((ultra.h_delta > 20) && (ultra.h_delta < 50))
	{
		ultra.relative_height = ultra_distance_old +20;
	}
	else if((ultra.h_delta < -20) && (ultra.h_delta > -50))
	{
		ultra.relative_height = ultra_distance_old -20;
	}
	
	ultra.relative_height = LPF_1st(ultra_distance_old,ultra.height,0.5);
	ultra.h_delta = ultra.relative_height - ultra_distance_old;
	ultra_distance_old = ultra.relative_height;
	
	if(change_Hight_data == 1)
	{
		hight_PID_ctrl.shell.hight_pid_error = hight_PID_ctrl.shell.set_hight - ultra.relative_height;
		hight_PID_ctrl.shell.hight_pid_merrer += (hight_PID_ctrl.shell.set_hight - ultra.relative_height) * 0.05f;
		hight_PID_ctrl.shell.hight_pid_deriv = (-ultra.h_delta) / 0.05f ; 
	}
	
	
}
extern u8 mode_state;
extern float thr_value,thr_value_old;
struct _hight_ctrl hight_PID_ctrl;
void hight_ctrl(int exp_hight)
{
static float h_delta;

/*********************************************/	
	if(mode_state == 0 || mode_state == 1) //手动油门
	{
		hight_PID_ctrl.shell.set_hight= GY_530.relative_height;
		hight_PID_ctrl.shell.hight_pid_merrer= 0 ;
	}
	else if(mode_state == 2 )
	{
//		hight_PID_ctrl.hight_pid_error = hight_PID_ctrl.set_hight - ultra.relative_height;
//		if((hight_PID_ctrl.hight_pid_error > hight_PID_ctrl.hight_error_1 + 10) )
//			hight_PID_ctrl.hight_pid_error = hight_PID_ctrl.hight_pid_error  + 10;
//		if(hight_PID_ctrl.hight_pid_error < hight_PID_ctrl.hight_error_1 - 10)
//			hight_PID_ctrl.hight_pid_error = hight_PID_ctrl.hight_pid_error - 10 ;
//		hight_PID_ctrl.hight_pid_KP_out = pid_setup.groups.hc_height.kp *  hight_PID_ctrl.hight_pid_error;
//		
//		hight_PID_ctrl.hight_pid_merrer += hight_PID_ctrl.hight_pid_error;
//		hight_PID_ctrl.hight_pid_merrer = LIMIT(hight_PID_ctrl.hight_pid_merrer,0,100);
//		hight_PID_ctrl.hight_pid_KI_out = pid_setup.groups.hc_height.ki * hight_PID_ctrl.hight_pid_merrer;
//		
//		hight_PID_ctrl.hight_pid_deriv = hight_PID_ctrl.hight_pid_error - hight_PID_ctrl.hight_error_1;
//		hight_PID_ctrl.hight_pid_KD_out = pid_setup.groups.hc_height.kd * hight_PID_ctrl.hight_pid_deriv;
//		hight_PID_ctrl.hight_error_1 = hight_PID_ctrl.hight_pid_error;
//			
//		hight_PID_ctrl.hight_pid_out = hight_PID_ctrl.hight_pid_KP_out + hight_PID_ctrl.hight_pid_KI_out
//																	+ hight_PID_ctrl.hight_pid_KD_out;
//		
//		
//		
//		hight_PID_ctrl.hight_pid_KP_out = pid_setup.groups.hc_height.kp * (hight_PID_ctrl.hight_pid_error - hight_PID_ctrl.hight_error_1);
//		
//		hight_PID_ctrl.hight_pid_KI_out = pid_setup.groups.hc_height.ki * hight_PID_ctrl.hight_pid_error;
//		
//		hight_PID_ctrl.hight_pid_KD_out = pid_setup.groups.hc_height.kd * (hight_PID_ctrl.hight_pid_error
//																			- 2*hight_PID_ctrl.hight_error_1
//																			+ hight_PID_ctrl.hight_error_2 );
//		
//		hight_PID_ctrl.hight_pid_out = hight_PID_ctrl.hight_pid_KP_out + hight_PID_ctrl.hight_pid_KI_out
//																	+ hight_PID_ctrl.hight_pid_KD_out;
//		hight_PID_ctrl.hight_error_2 = hight_PID_ctrl.hight_error_1;
//		hight_PID_ctrl.hight_error_1 = hight_PID_ctrl.hight_pid_error; 
//pid_setup.groups.hc_sp.kp
		
		
//		//高度外环
//		hight_PID_ctrl.shell.hight_pid_error = hight_PID_ctrl.shell.set_hight - ultra.relative_height;
//		hight_PID_ctrl.shell.hight_pid_KP_out= pid_setup.groups.hc_height.kp* hight_PID_ctrl.shell.hight_pid_error;
//		
//		hight_PID_ctrl.shell.hight_pid_merrer +=  hight_PID_ctrl.shell.hight_pid_error ;
//		hight_PID_ctrl.shell.hight_pid_merrer= LIMIT(hight_PID_ctrl.shell.hight_pid_merrer,0,50);
//		hight_PID_ctrl.shell.hight_pid_KI_out= pid_setup.groups.hc_height.ki* hight_PID_ctrl.shell.hight_pid_merrer;
//		
//		hight_PID_ctrl.shell.hight_pid_deriv = hight_PID_ctrl.shell.hight_pid_error-hight_PID_ctrl.shell.hight_pid_error_old;
//		hight_PID_ctrl.shell.hight_pid_KD_out= pid_setup.groups.hc_height.kd* hight_PID_ctrl.shell.hight_pid_deriv;
//		hight_PID_ctrl.shell.hight_pid_error_old= hight_PID_ctrl.shell.hight_pid_error;
//		
//		hight_PID_ctrl.shell.hight_pid_out = hight_PID_ctrl.shell.hight_pid_KP_out + hight_PID_ctrl.shell.hight_pid_KI_out
//																	+ hight_PID_ctrl.shell.hight_pid_KD_out;
//		
//		
//		//高度内环
//		hight_PID_ctrl.core.hight_pid_error = -(mpu6050.Acc.z- 4096)/10.0f + hight_PID_ctrl.shell.hight_pid_out;
//		hight_PID_ctrl.core.hight_pid_KP_out= pid_setup.groups.hc_sp.kp* hight_PID_ctrl.core.hight_pid_error;
//		
//		hight_PID_ctrl.core.hight_pid_merrer +=  hight_PID_ctrl.core.hight_pid_error ;
//		hight_PID_ctrl.core.hight_pid_merrer= LIMIT(hight_PID_ctrl.core.hight_pid_merrer,-50,50);
//		hight_PID_ctrl.core.hight_pid_KI_out= pid_setup.groups.hc_sp.ki* hight_PID_ctrl.core.hight_pid_merrer;
//		
//		hight_PID_ctrl.core.hight_pid_deriv = mpu6050.Acc.z-hight_PID_ctrl.core.hight_pid_error_old;
//		hight_PID_ctrl.core.hight_pid_KD_out= pid_setup.groups.hc_sp.kd* hight_PID_ctrl.core.hight_pid_deriv;
//		hight_PID_ctrl.core.hight_pid_error_old= mpu6050.Acc.z;
//		
//		hight_PID_ctrl.core.hight_pid_out = hight_PID_ctrl.core.hight_pid_KP_out + hight_PID_ctrl.core.hight_pid_KI_out
//																	+ hight_PID_ctrl.core.hight_pid_KD_out;
/******************************************************/

//		hight_PID_ctrl.shell.hight_pid_error = exp_hight - GY_530.relative_height;			
		hight_PID_ctrl.shell.hight_pid_KP_out= pid_setup.groups.hc_height.kp* hight_PID_ctrl.shell.hight_pid_error;
		
//		hight_PID_ctrl.shell.hight_pid_merrer +=  hight_PID_ctrl.shell.hight_pid_error ;
		hight_PID_ctrl.shell.hight_pid_merrer= LIMIT(hight_PID_ctrl.shell.hight_pid_merrer,-50,50);
		hight_PID_ctrl.shell.hight_pid_KI_out= pid_setup.groups.hc_height.ki* hight_PID_ctrl.shell.hight_pid_merrer;
		
//		h_delta = (hight_PID_ctrl.shell.hight_pid_error-hight_PID_ctrl.shell.hight_pid_error_old) / 0.05f;	
//		hight_PID_ctrl.shell.hight_pid_deriv = (-ultra.h_delta) / 0.05f ;
		hight_PID_ctrl.shell.hight_pid_KD_out= pid_setup.groups.hc_height.kd* hight_PID_ctrl.shell.hight_pid_deriv;

		
		hight_PID_ctrl.shell.hight_pid_out = hight_PID_ctrl.shell.hight_pid_KP_out + hight_PID_ctrl.shell.hight_pid_KI_out
																	+ hight_PID_ctrl.shell.hight_pid_KD_out;
																	
	}
	hight_PID_ctrl.shell.hight_pid_out= LIMIT(hight_PID_ctrl.shell.hight_pid_out,-100,100);

	
}



void hubu_Ctrl(void)
{
	static float vel;
			if(mode_state == 0 || mode_state == 1) //手动油门
			{
				hight_PID_ctrl.shell.set_hight = ultra.relative_height;
				vel = 0;
				
				hight_PID_ctrl.shell.hight_pid_merrer = 0;
				hight_PID_ctrl.shell.hight_pid_error_old = ultra.relative_height; 
				thr_value =thr_value_old;
			}
			else if(mode_state == 2 )
			{
//				if((thr_value_old<thr_value-100)||(thr_value_old>thr_value+100)) 
//					hight_PID_ctrl.shell.set_hight += (thr_value_old-thr_value)/30000.0f;
				
				hight_PID_ctrl.shell.hight_pid_error = (hight_PID_ctrl.shell.set_hight -ultra.relative_height) ;//单位从米到厘米
				//死区
				hight_PID_ctrl.shell.hight_pid_error = DEAD_BAND(hight_PID_ctrl.shell.hight_pid_error,0,5);
				hight_PID_ctrl.shell.hight_pid_KP_out= pid_setup.groups.hc_height.kp* hight_PID_ctrl.shell.hight_pid_error;
				
				hight_PID_ctrl.shell.hight_pid_merrer +=  hight_PID_ctrl.shell.hight_pid_error ;
				hight_PID_ctrl.shell.hight_pid_merrer= LIMIT(hight_PID_ctrl.shell.hight_pid_merrer,-50,50);
				hight_PID_ctrl.shell.hight_pid_KI_out= pid_setup.groups.hc_height.ki* hight_PID_ctrl.shell.hight_pid_merrer;
				
				vel += (mpu6050.Acc.z - GY_530.ACC_z_zero) / 4096 ;//0.05*100
				float baroVel = (ultra.relative_height - hight_PID_ctrl.shell.hight_pid_error_old) * 10000;//20*100
				//死区
				vel = DEAD_BAND(vel,0,10);
				//baroVel = DEAD_BAND(baroVel,0,10);
				hight_PID_ctrl.shell.hight_pid_error_old = ultra.relative_height	;
				vel = vel * pid_setup.groups.ctrl3.kp+ baroVel * (1 - pid_setup.groups.ctrl3.kp);
	
				hight_PID_ctrl.core.hight_pid_KD_out= pid_setup.groups.hc_sp.kd* vel;
				
				hight_PID_ctrl.shell.hight_pid_out = hight_PID_ctrl.shell.hight_pid_KP_out + hight_PID_ctrl.shell.hight_pid_KI_out
																	+ hight_PID_ctrl.shell.hight_pid_KD_out;
			 }
}
	
/*	
	if( (value < (mid) - (ban) )
	{
	    return value + ban;	
	}
	else if( (value) > (mid) + (ban) )
	{
	    return value - ban;
	    		
	}
	else
	{
	    return mid;	
	}
*/


