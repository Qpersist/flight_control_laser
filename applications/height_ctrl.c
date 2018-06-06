/******************** (C) COPYRIGHT 2016 ANO Tech ********************************
  * ����   �������ƴ�
 * �ļ���  ��height_ctrl.c
 * ����    ���߶ȿ���
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/
#include "height_ctrl.h"
#include "anotc_baro_ctrl.h"
#include "mymath.h"
#include "filter.h"
#include "rc.h"
#include "PID.h"
#include "ctrl.h"
#include "include.h"
#include "fly_mode.h"
#include <stdlib.h>
#include "include.h"
#include "VL53L0.h"


float	set_height_e,set_height_em,
			set_speed_t,set_speed,exp_speed,fb_speed,
			exp_acc,fb_acc,fb_speed,fb_speed_old;

_hc_value_st hc_value;


u8 thr_take_off_f = 0;
u8 auto_take_off,auto_land;
float height_ref;

float auto_take_off_land(float dT,u8 ready)
{
	static u8 back_home_old;
	static float thr_auto;
	
	if(ready==0)
	{
		height_ref = hc_value.fusion_height;
		auto_take_off = 0;
	}
	
	if(Thr_Low == 1 && fly_ready == 0)
	{
		if(mode_value[BACK_HOME] == 1 && back_home_old == 0) //���֮ǰ�����ҽ���֮ǰ���Ƿ���ģʽ��������ģʽ
		{
				if(auto_take_off==0)  //��һ�����Զ���ɱ��0->1
				{
					auto_take_off = 1;
				}
		}
	}
	
	switch(auto_take_off)
	{
		case 1:
		{
			if(thr_take_off_f ==1)
			{
				auto_take_off = 2;
			}
			break;
		}
		case 2:
		{
			if(hc_value.fusion_height - height_ref>500)
			{
				if(auto_take_off==2) //�Ѿ������Զ����
				{
					auto_take_off = 3;
				}
			}
		
		}
		default:break;
	}

	

	
	if(auto_take_off == 2)
	{
		thr_auto = 200;
	}
	else if(auto_take_off == 3)
	{
		thr_auto -= 200 *dT;
	
	}
	
	thr_auto = LIMIT(thr_auto,0,300);
	
	back_home_old = mode_value[BACK_HOME]; //��¼ģʽ��ʷ
		
	return (thr_auto);
}
	


_PID_arg_st h_acc_arg;
_PID_arg_st h_speed_arg;
_PID_arg_st h_height_arg;

_PID_val_st h_acc_val;
_PID_val_st h_speed_val;
_PID_val_st h_height_val;

void h_pid_init()
{
	h_acc_arg.kp = 0.01f ;				//����ϵ��
	h_acc_arg.ki = 0.02f  *pid_setup.groups.hc_sp.kp;				//����ϵ��
	h_acc_arg.kd = 0;				//΢��ϵ��
	h_acc_arg.k_pre_d = 0 ;	
	h_acc_arg.inc_hz = 0;
	h_acc_arg.k_inc_d_norm = 0.0f;
	h_acc_arg.k_ff = 0.05f;

	h_speed_arg.kp = 1.5f *pid_setup.groups.hc_sp.kp;				//����ϵ��
	h_speed_arg.ki = 0.0f *pid_setup.groups.hc_sp.ki;				//����ϵ��
	h_speed_arg.kd = 0.0f;				//΢��ϵ��
	h_speed_arg.k_pre_d = 0.10f *pid_setup.groups.hc_sp.kd;
	h_speed_arg.inc_hz = 20;
	h_speed_arg.k_inc_d_norm = 0.8f;
	h_speed_arg.k_ff = 0.5f;	
	
	h_height_arg.kp = 1.5f *pid_setup.groups.hc_height.kp;				//����ϵ��
	h_height_arg.ki = 0.0f *pid_setup.groups.hc_height.ki;				//����ϵ��
	h_height_arg.kd = 0.05f *pid_setup.groups.hc_height.kd;				//΢��ϵ��
	h_height_arg.k_pre_d = 0.01f ;
	h_height_arg.inc_hz = 20;
	h_height_arg.k_inc_d_norm = 0.5f;
	h_height_arg.k_ff = 0;	
	
}

float thr_set,thr_pid_out,thr_out,thr_take_off,tilted_fix;

float en_old;
u8 ex_i_en_f,ex_i_en;

float Height_Ctrl(float T,float thr,u8 ready,float en)
{
	static u8 step,speed_cnt,height_cnt;
	
	if(ready == 0)
	{
		ex_i_en = ex_i_en_f = 0;
		en = 0;
		thr_take_off = 0;
		thr_take_off_f = 0;
	}
	
	switch(step)
	{
		case 0:
		{

			//step = 1;
			break;
		}
		case 1:
		{

			
			step = 2;
			break;
		}
		case 2:
		{
		
			step = 3;
			break;
		}
		case 3:
		{
			
			step = 4;
			break;
		}	
		case 4:
		{
			
			step = 0;
			break;
		}	
		default:break;	
	
	}
	/*�����г��ν��붨��ģʽ�л�����*/
	if(ABS(en - en_old) > 0.5f)//�ӷǶ����л�������
	{
		if(thr_take_off<10)//δ�����������
		{
			if(thr_set > -150)
			{
				thr_take_off = 400;
				
			}
		}
		en_old = en;
	}
	
	/*���߿���*/
	//h_pid_init();
	
	thr_set = my_deathzoom_2(my_deathzoom((thr - 500),0,40),0,10);
	
	if(thr_set>0)
	{
		set_speed_t = thr_set/450 * MAX_VERTICAL_SPEED_UP;
		
		if(thr_set>100)
		{
			ex_i_en_f = 1;
			
			if(!thr_take_off_f)
			{
				thr_take_off_f = 1; //�û�������Ҫ���
				thr_take_off = 350; //ֱ�Ӹ�ֵ һ��
				
			}
		}
	}
	else
	{
		if(ex_i_en_f == 1)
		{
			ex_i_en = 1;
		}
		set_speed_t = thr_set/450 * MAX_VERTICAL_SPEED_DW;
	}
	
	set_speed_t = LIMIT(set_speed_t,-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP);
	
	//exp_speed =my_pow_2_curve(exp_speed_t,0.45f,MAX_VERTICAL_SPEED);
	LPF_1_(10.0f,T,my_pow_2_curve(set_speed_t,0.25f,MAX_VERTICAL_SPEED_DW),set_speed);
	
	set_speed = LIMIT(set_speed,-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP);
	
/////////////////////////////////////////////////////////////////////////////////	
	baro_ctrl(T,&hc_value); //�߶����ݻ�ȡ�� ��ѹ������
	
/////////////////////////////////////////////////////////////////////////////////		
	//����߶����ɼ��˲���
	set_height_em += (set_speed - hc_value.m_speed) *T;
	set_height_em = LIMIT(set_height_em,-5000 *ex_i_en,5000 *ex_i_en);
	
	set_height_e += (set_speed - 1.05f *hc_value.fusion_speed) *T;
	set_height_e = LIMIT(set_height_e,-5000 *ex_i_en,5000 *ex_i_en);
	
	LPF_1_(0.05f,T,set_height_em,set_height_e);
	
	
/////////////////////////////////////////////////////////////////////////////////		
/////////////////////////////////////////////////////////////////////////////////
	if(en < 0.1f)
	{
		exp_speed = hc_value.fusion_speed;
		exp_acc = hc_value.fusion_acc;
	}
/////////////////////////////////////////////////////////////////////////////////	
	float acc_i_lim;
	acc_i_lim = safe_div(150,h_acc_arg.ki,0);
	
	fb_speed_old = fb_speed;
	fb_speed = hc_value.fusion_speed;
	fb_acc = safe_div(fb_speed - fb_speed_old,T,0);
	
	thr_pid_out = PID_calculate( T,            //����
														exp_acc,				//ǰ��
														exp_acc,				//����ֵ���趨ֵ��
														fb_acc,			//����ֵ
														&h_acc_arg, //PID�����ṹ��
														&h_acc_val,	//PID���ݽṹ��
														acc_i_lim*en			//integration limit�������޷�
														 );			//���		

	//step_filter(1000 *T,thr_pid_out,thr_pid_out_dlim);
	
	//�������
	if(h_acc_val.err_i > (acc_i_lim * 0.2f))
	{
		if(thr_take_off<THR_TAKE_OFF_LIMIT)
		{
			thr_take_off += 150 *T;
			h_acc_val.err_i -= safe_div(150,h_acc_arg.ki,0) *T;
		}
	}
	else if(h_acc_val.err_i < (-acc_i_lim * 0.2f))
	{
		if(thr_take_off>0)
		{
			thr_take_off -= 150 *T;
			h_acc_val.err_i += safe_div(150,h_acc_arg.ki,0) *T;
		}
	}
	
	thr_take_off = LIMIT(thr_take_off,0,THR_TAKE_OFF_LIMIT); //һ��
	
	
	//���Ų���
	tilted_fix = safe_div(1,LIMIT(reference_v.z,0.707f,1),0); //45���ڲ���
	
	//�������
	thr_out = (thr_pid_out + tilted_fix *(thr_take_off) );
	
	thr_out = LIMIT(thr_out,0,1000);
	

	
/////////////////////////////////////////////////////////////////////////////////	
	static float dT,dT2;
	dT += T;
	speed_cnt++;
	if(speed_cnt>=10) //u8  20ms
	{

		exp_acc = PID_calculate( dT,            //����
														exp_speed,				//ǰ��
														(set_speed + exp_speed),				//����ֵ���趨ֵ��
														hc_value.fusion_speed,			//����ֵ
														&h_speed_arg, //PID�����ṹ��
														&h_speed_val,	//PID���ݽṹ��
														500 *en			//integration limit�������޷�
														 );			//���	
		
		exp_acc = LIMIT(exp_acc,-3000,3000);
		
		//integra_fix += (exp_speed - hc_value.m_speed) *dT;
		//integra_fix = LIMIT(integra_fix,-1500 *en,1500 *en);
		
		//LPF_1_(0.5f,dT,integra_fix,h_speed_val.err_i);
		
		dT2 += dT;
		height_cnt++;
		if(height_cnt>=10)  //200ms 
		{
			/////////////////////////////////////

		 exp_speed = PID_calculate( dT2,            //����
																0,				//ǰ��
																0,				//����ֵ���趨ֵ��
																-set_height_e,			//����ֵ
																&h_height_arg, //PID�����ṹ��
																&h_height_val,	//PID���ݽṹ��
																1500 *en			//integration limit�������޷�
																 );			//���	
			
			exp_speed = LIMIT(exp_speed,-300,300);
			/////////////////////////////////////
			dT2 = 0;
			height_cnt = 0;
		}
		
		speed_cnt = 0;
		dT = 0;				
	}		
/////////////////////////////////////////////////////////////////////////////////	
	if(step==0)
	{
		step = 1;
	}
	
	if(en < 0.1f)
	{
		return (thr);
	}
	else
	{
		return (thr_out);
	}
}

/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/





/***************���ز��ֵĶ��� δ��ʵ�ֶ��� ����**********************/
struct Flag flag;

/******************�߶ȿ��Ʊ���********************/
float height_ctrl_out;
float wz_acc;
float keepheight_thr;
/*-------------------------------------------------*/

_HT_HAWK HT_HAWK;

/******************����������********************/
#define ULTRA_SPEED 		 300    // mm/s
#define ULTRA_INT        300    // ���ַ���
u8 lock=0;
extern  s8 ultra_start_ok;;
extern float US100_Alt_delta;
float exp_height_speed,exp_height;
float ultra_speed,ultra_speed_acc;
float ultra_dis_lpf;
float ultra_ctrl_out;

/*-------------------------------------------------*/

/******************�ٶȻ�����********************/
extern float wz_speed;
float wz_acc_mms2;
float tempacc_lpf;
u8 lock_spd_crl=0;

/*-------------------------------------------------*/

/******************��ѹ�Ʊ���********************/
float baro_height;
extern float baro_height_old;
#define BARO_SPEED 		 300    // mm/s
#define BARO_INT        300    // ���ַ���

u8 lock_BARO=0;//����������ǰ�Լ�������
#define BARO_SPEED_NUM 10
float baro_speed_arr[BARO_SPEED_NUM + 1];


float acc_speed_arr[ACC_SPEED_NUM + 1];

float baro_dis_lpf,baro_dis_kalman;
float baro_ctrl_out;


u16 baro_cnt[2];
u16 acc_cnt[2];
float baro_speed,baro_speed_lpf;
float baro_height;
float Pressure_groud;
/*-------------------------------------------------*/


void LockForKeepHigh(float THROTTLE)
{
    if (flag.FlightMode==ULTRASONIC_High && 	lock ==0)
		{
			HT_HAWK.wz_speed_pid.kp= pid_setup.groups.hc_sp.kp;//��PID
			HT_HAWK.wz_speed_pid.ki= pid_setup.groups.hc_sp.ki;
			HT_HAWK.wz_speed_pid.kd= pid_setup.groups.hc_sp.kd;
			
			exp_height=ultra_dis_lpf;//����������߶�ֵ�����˲�����
			wz_acc=0;
			HT_HAWK.wz_speed_pid_v.err_i=0;
			ultra_speed=0;//�������㣬���ҳ�����ʱԭ���ۻ����ٶȻ���󴫵ݸ���ǰ����ɵ���
			HT_HAWK.ultra_ctrl.err_i=0;
			HT_HAWK.ultra_ctrl.err_old=0;
			lock=1;
			hight_PID_ctrl.shell.set_hight  = ultra.relative_height;
			
    }
//		else if(flag.FlightMode==MANUAL_High || flag.FlightMode==ATMOSPHERE_High)
	else if(flag.FlightMode==1)
		{lock=0;}


			//��ѹ����ʱ����
//		if (flag.FlightMode==ATMOSPHERE_High && lock_BARO==0)
//		{
//			  wz_speed_pid.kp=baro_wz_speed_pid.kp;//��PID
//		    wz_speed_pid.ki=baro_wz_speed_pid.ki;
//		    wz_speed_pid.kd=baro_wz_speed_pid.kd;
//        exp_height=baro_dis_lpf;
//			  wz_acc=0;
//				baro_speed=0;
//				baro_speed_lpf=0;
//			  baro_ctrl.err_i=0;
//				baro_ctrl.err_old=0;				
//			  lock_BARO=1;				
//    }
//		else if(flag.FlightMode==MANUAL_High || flag.FlightMode==ULTRASONIC_High)
//		{lock_BARO=0;}					
}

void Ultra_dataporcess(float T)
{
	float ultra_sp_tmp,ultra_dis_tmp;	
		ultra_dis_tmp = HT_Moving_Median(1,5,ultra.relative_height);  //�Գ����������ľ�������ƶ���λֵ�˲�
	
		if( ultra_dis_tmp < Ultrasonic_MAX_Height )
		{	
			if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 100 )
			{	
				ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 4.0f *3.14f *T ) ) ) *(ultra_dis_tmp - ultra_dis_lpf) ;
			}
			else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 200 )
			{			
				ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 2.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
			}
			else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 400 )
			{
				ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 1.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
			}
			else
			{
				ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 0.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
			}
		}
		
		//ע�ⳬ���������� �����ʱЧ�����⣬���ⷴ��������
		ultra_sp_tmp = HT_Moving_Median(0,5,ultra.h_delta/T); //�Գ���������������֮�����������ٶ���ֵ�˲� ultra_delta/T;

		if( ultra_dis_tmp < Ultrasonic_MAX_Height )//С��3��,ע�������һ�������أ�
		{
			if( ABS(ultra_sp_tmp) < 100 )//�˶��ٶ�С��100mm/s
			{
				ultra_speed += ( 1 / ( 1 + 1 / ( 4 *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
			}//ultra_speed�ᴫ�ݸ��ٶȻ�����Ϊ��ǰ�ٶȵķ���,ԭ����4
			else
			{
				ultra_speed += ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
			}//ϵ��Խ������ԽС��ԭ����1
		}
	
}


void height_speed_ctrl(float T,float thr_keepheight,float exp_z_speed,float h_speed)
{

		//wz_speed�����ڲ��볬�����߶�D����
		HT_HAWK.wz_speed_pid_v.err = HT_HAWK.wz_speed_pid.kp *( exp_z_speed - wz_speed );//
	
		HT_HAWK.wz_speed_pid_v.err_d = HT_HAWK.wz_speed_pid.kd * (-tempacc_lpf) *T;//(wz_speed_pid_v.err - wz_speed_pid_v.err_old);
		//wz_speed_pid_v.err_i += wz_speed_pid.ki *wz_speed_pid_v.err *T;
		HT_HAWK.wz_speed_pid_v.err_i += HT_HAWK.wz_speed_pid.ki *( exp_z_speed - h_speed ) *T;//�����ٶ���ʵ���ٶ�������
	
		HT_HAWK.wz_speed_pid_v.err_i = LIMIT(HT_HAWK.wz_speed_pid_v.err_i,-Thr_Weight *200,Thr_Weight *200);
	

	
		HT_HAWK.wz_speed_pid_v.pid_out = thr_keepheight + Thr_Weight *LIMIT((HT_HAWK.wz_speed_pid.kp *exp_z_speed 
											+ HT_HAWK.wz_speed_pid_v.err + HT_HAWK.wz_speed_pid_v.err_d 
													+ HT_HAWK.wz_speed_pid_v.err_i),-300,300);//����ԭ��û�г�wz_speed_pid.kp
                               

	
		HT_HAWK.wz_speed_pid_v.err_old = HT_HAWK.wz_speed_pid_v.err; 
}

void Ultra_Ctrl(float T,float thr)
{
		exp_height_speed = ULTRA_SPEED *my_deathzoom_2(thr - 500,0,100)/300.0f; //20�����������Լ�������Ŷ������ſ��Ƹ߶�+-ULTRA_SPEEDmm / s
	
		exp_height_speed = LIMIT(exp_height_speed ,-ULTRA_SPEED,ULTRA_SPEED);
		
		if( exp_height > Ultrasonic_MAX_Height )//�����������߶��ȶ���Χ��ִ��
		{
			if( exp_height_speed > 0 )
			{
				exp_height_speed = 0;
			}
		}
		else if( exp_height < Ultrasonic_MIN_Height )
		{
			if( exp_height_speed < 0 )
			{
				exp_height_speed = 0;
			}
		}
		
		exp_height += exp_height_speed *T;//�ۻ������߶ȣ���Ϊ�����ٶȿ��ܸı���

		HT_HAWK.ultra_ctrl.err = ( HT_HAWK.ultra_pid.kp*(exp_height - ultra_dis_lpf) );//mm �Ը߶�������Kp
		

			
		HT_HAWK.ultra_ctrl.err_i += HT_HAWK.ultra_pid.ki *HT_HAWK.ultra_ctrl.err *T;//�Ը߶�������
			
		HT_HAWK.ultra_ctrl.err_i = LIMIT(HT_HAWK.ultra_ctrl.err_i,-Thr_Weight *ULTRA_INT,Thr_Weight *ULTRA_INT);//�����޷����ں�������Ȩ��	
					//����D���ں��˼��ٶȼ�����õ��ľ���	
//		userdata2[7]=0.4f *(ultra_ctrl.err_old - ultra_ctrl.err);
		
		HT_HAWK.ultra_ctrl.err_d = HT_HAWK.ultra_pid.kd *( 0.5f *(-wz_speed*T) + 0.5f *(HT_HAWK.ultra_ctrl.err - HT_HAWK.ultra_ctrl.err_old) );
			

		
		HT_HAWK.ultra_ctrl.pid_out = HT_HAWK.ultra_ctrl.err +HT_HAWK.ultra_ctrl.err_i + HT_HAWK.ultra_ctrl.err_d;	
			
		HT_HAWK.ultra_ctrl.pid_out = LIMIT(HT_HAWK.ultra_ctrl.pid_out,-500,500);
	
		if(flag.FlightMode!=ULTRASONIC_High)
			HT_HAWK.ultra_ctrl.pid_out=0;//������������ڲ��Լ��ٶȼƶ���Ч��,����Ȼ���ٶȷ���		
		
		ultra_ctrl_out = HT_HAWK.ultra_ctrl.pid_out;
	
		HT_HAWK.ultra_ctrl.err_old = HT_HAWK.ultra_ctrl.err;
}


void HT_Height_Ctrl(float T,float thr)
{
	static u8 height_ctrl_start_f;
	static float thr_lpf;
	static float height_thr;
	static float wz_acc_temp,wz_acc1;
	static float hc_speed_i,wz_speed_0;
	static float h_speed;
	
	
	switch( height_ctrl_start_f )
	{
		case 0:
	
				if( mpu6050.Acc.z > 4000 )//ע�����ָˮƽ��̬�µ�ZֵҪ������󣬱�ʾˮƽ��ֱ    +-2G ��4096
				{
					  height_ctrl_start_f = 1;
				}
				break;
		
		case 1:
			
				LockForKeepHigh(thr_lpf);//һ���е��ظ�ģʽ���������浱ǰ�ľ���ֵ������ֵ��������㡣
				if((flag.FlightMode==ULTRASONIC_High && lock==1 &&lock_spd_crl==0))
				{
						keepheight_thr=thr_lpf;
						wz_speed_0=0;
						hc_speed_i=0;
						hc_speed_i=0;
						wz_speed=0;
						HT_HAWK.wz_speed_pid_v.err_old=0;
						HT_HAWK.wz_speed_pid_v.err=0;
						HT_HAWK.wz_speed_pid_v.err_i=0;
						lock=2;
						lock_BARO=2;
						lock_spd_crl=1;
				}
				else if(flag.FlightMode== 1)
				{
						lock=0;
						lock_BARO=0;
						lock_spd_crl=0;//�л��ֶ�ģʽ�����㣬��ʹ�´ν��붨��ģʽ����˳�������ٶ��ڻ�	
				}
			
				height_thr = LIMIT( thr , 0, 700 );
				thr_lpf += ( 1 / ( 1 + 1 / ( 2.0f *3.14f *T ) ) ) *( height_thr - thr_lpf );//������ֵ��ͨ�˲�����
				//userdata1[0]=	thr_lpf;//������
				/*����ĵ�ͨ�˲����ڲ��ԶԱ�Ч��������û��ѡ��*/				
		    //wz_acc += ( 1 / ( 1 + 1 / ( 8 *3.14f *T ) ) ) *my_deathzoom( (V.z *(sensor.acc.averag.z- sensor.acc.quiet.z) + V.x *sensor.acc.averag.x + V.y *sensor.acc.averag.y - wz_acc),100 );//
		    //wz_acc_mms2 = (float)(wz_acc/8192.0f) *9800 ;
				//���ٶȵľ�̬����Ƕ�ȡ��EEPROM���ݣ�����ÿ�η��ж����ܲ�һ������ÿ�����ǰУ׼һ����ã����Ե���У׼Z��ģ�������Ƴ���		
				
				//test = V.z *(sensor.acc.averag.z  - sensor.acc.quiet.z) + V.x *sensor.acc.averag.x + V.y *sensor.acc.averag.y;
				wz_acc_temp = sqrt(abs((int)mpu6050.Acc.x*(int)mpu6050.Acc.x) + abs(mpu6050.Acc.y*mpu6050.Acc.y) + abs(mpu6050.Acc.z*mpu6050.Acc.z)) - GY_530.ACC_z_zero;
						
				Moving_Average( acc_speed_arr,ACC_SPEED_NUM,acc_cnt ,wz_acc_temp,&wz_acc1 );		
				
	
				tempacc_lpf= (float)(wz_acc1/4096.0f) *9800;//9800 *T;������+-4G��8G��65535/8g=8192 g�����ٶȣ�mm/s2����ÿƽ����				
				
				if(abs(tempacc_lpf)<50)tempacc_lpf=0;//������������
						

							
				wz_speed_0 += tempacc_lpf *T;//���ٶȼƻ��ֳ��ٶ�				
								
				if( ultra.start_ok == 1 )////������ɶģʽ��ֻҪ�����˳��������ݾͽ�������
				{	
					 Ultra_dataporcess(15.0f*TT);
					 ultra.start_ok=2;
				}
				
//				if(baro_start_f==1)//������ɶģʽ��ֻҪ��������ѹ���ݾͽ�������
//				{ 
//					 Baro_dataporcess(8.0f*TT);			 //8.0f*TT��ô��ʱ�����һ����ѹ������
//					 userdata2[10]=baro_height;//Baro_Height_Source
//					 baro_dis_delta=baro_height-baro_dis_old;
//					 baro_dis_old=baro_height;//��ѹ���ٶȿ��Լ����Ż�������Ƚϴֲ�			 
//		       //Moving_Average( (float)( baro_dis_delta/(8.0f*TT)),baro_speed_arr,BARO_SPEED_NUM,baro_cnt ,&baro_speed ); //��λmm/s
//					 userdata1[11]=baro_speed_lpf=0.4* baro_dis_delta/(8.0f*TT);//baro_speed�������ϵ����������ֵ		 
//					 baro_start_f=2;
//				}	
		
				if(flag.FlightMode==ULTRASONIC_High)
				{
					 h_speed=ultra_speed;
					 wz_speed_0 = 0.9f	*wz_speed_0 + 0.1f*h_speed;//��������ֱ�ٶȻ����˲�
				}				
//				else if(flag.FlightMode==ATMOSPHERE_High)
//				{
//					 h_speed=baro_speed_lpf;
//					 wz_speed_0 = 0.99	*wz_speed_0 + 0.01*h_speed;//��ѹ�ƴ�ֱ�ٶȻ����˲���ϵ���ɵ�
//				}	

				//h_speed�Ǹ߶Ȼ������ٶȻ���ʵ��߶ȷ����ٶȡ��������Ǵ���ģ���ѹ���ٶȲ��ɿ���

				hc_speed_i += 0.4f *T *( h_speed - wz_speed );//�ٶ�ƫ����֣�������0.4ϵ��
				hc_speed_i = LIMIT( hc_speed_i, -500, 500 );//�����޷�	

				
				
				wz_speed_0 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( h_speed - wz_speed_0  ) ;//0.1ʵ���ٶ��������ٶ�����ٶ�
				
				
				 wz_speed = wz_speed_0 + hc_speed_i;//�����������ٶ�+�����޷�������ʽ�ٶȻ���

//			  if( flag.FlightMode == ATMOSPHERE_High)
//				{
//						if(baro_start_f==2)//˵�����������ұ������ƶ���ֵ�˲�ʹ�ù���
//						{
//							baro_start_f = -1;
//							Baro_Ctrl(8.0f*TT,thr_lpf);
//						}	
//						height_speed_ctrl(T,keepheight_thr,baro_ctrl_out,baro_speed_lpf);							
//				}

				if( flag.FlightMode == ULTRASONIC_High)
				{
						height_speed_ctrl(T,keepheight_thr,ultra_ctrl_out,ultra_speed);//ϵ��ԭ����0.4
							
						if( ultra.start_ok == 2 )//���������ݸ������ұ�������
						{				
								Ultra_Ctrl(15.0f*TT,thr_lpf);//realtime������Ϊ TT ��ģ�#define TT 0.0025//��������2.5ms
								ultra.start_ok = -1;
						}
				}
		
	      /*******************************************���Ƹ߶����********************************************/
				if(flag.FlightMode==ULTRASONIC_High )//ע�����ģʽ
				{		
					  height_ctrl_out = HT_HAWK.wz_speed_pid_v.pid_out;
				}
				else
				{
					  height_ctrl_out = thr;
				}
	
			
				break; 	
				default: break;
	} 
	
	
	
}

void HT_Ultra_PID_Init(void)
{
		HT_HAWK.ultra_pid.kp = pid_setup.groups.hc_height.kp;//���������ߵĸ߶�λ�û�PID���ᱻEEPROM������¸�ֵ��  //0.15
		HT_HAWK.ultra_pid.ki = pid_setup.groups.hc_height.ki;
		HT_HAWK.ultra_pid.kd = pid_setup.groups.hc_height.kd;	//2.5
}

void HT_WZ_Speed_PID_Init(void)
{
		HT_HAWK.ultra_wz_speed_pid.kp = 0.25;// //���������ߵ��ٶȻ�PID���ᱻEEPROM������¸�ֵ��  //
		HT_HAWK.ultra_wz_speed_pid.ki = 0.08; //0.12
		HT_HAWK.ultra_wz_speed_pid.kd = 8;
	
		HT_HAWK.baro_wz_speed_pid.kp = 0.1;// ��ѹ���ߵ��ٶȻ�PID���ᱻEEPROM������¸�ֵ��
		HT_HAWK.baro_wz_speed_pid.ki = 0.06; 
		HT_HAWK.baro_wz_speed_pid.kd = 8;//1.4*10���ڼ���ʽ������10����ɾ��
	
		HT_HAWK.wz_speed_pid.kp = 0.1;//�ٶȻ�Ĭ��PID�������ò��������¸�ֵ
		HT_HAWK.wz_speed_pid.ki = 0.08;
		HT_HAWK.wz_speed_pid.kd = 8;
}

void HT_Baro_PID_Init(void)
{
		HT_HAWK.baro_pid.kp = 0.15;//��ѹ���ߵĸ߶�λ�û�PID���ᱻEEPROM������¸�ֵ��
		HT_HAWK.baro_pid.ki = 0;
		HT_HAWK.baro_pid.kd = 1.5;	//2.5
}
