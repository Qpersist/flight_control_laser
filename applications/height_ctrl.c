/******************** (C) COPYRIGHT 2016 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：height_ctrl.c
 * 描述    ：高度控制
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
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
		if(mode_value[BACK_HOME] == 1 && back_home_old == 0) //起飞之前，并且解锁之前，非返航模式拨到返航模式
		{
				if(auto_take_off==0)  //第一步，自动起飞标记0->1
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
				if(auto_take_off==2) //已经触发自动起飞
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
	
	back_home_old = mode_value[BACK_HOME]; //记录模式历史
		
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
	h_acc_arg.kp = 0.01f ;				//比例系数
	h_acc_arg.ki = 0.02f  *pid_setup.groups.hc_sp.kp;				//积分系数
	h_acc_arg.kd = 0;				//微分系数
	h_acc_arg.k_pre_d = 0 ;	
	h_acc_arg.inc_hz = 0;
	h_acc_arg.k_inc_d_norm = 0.0f;
	h_acc_arg.k_ff = 0.05f;

	h_speed_arg.kp = 1.5f *pid_setup.groups.hc_sp.kp;				//比例系数
	h_speed_arg.ki = 0.0f *pid_setup.groups.hc_sp.ki;				//积分系数
	h_speed_arg.kd = 0.0f;				//微分系数
	h_speed_arg.k_pre_d = 0.10f *pid_setup.groups.hc_sp.kd;
	h_speed_arg.inc_hz = 20;
	h_speed_arg.k_inc_d_norm = 0.8f;
	h_speed_arg.k_ff = 0.5f;	
	
	h_height_arg.kp = 1.5f *pid_setup.groups.hc_height.kp;				//比例系数
	h_height_arg.ki = 0.0f *pid_setup.groups.hc_height.ki;				//积分系数
	h_height_arg.kd = 0.05f *pid_setup.groups.hc_height.kd;				//微分系数
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
	/*飞行中初次进入定高模式切换处理*/
	if(ABS(en - en_old) > 0.5f)//从非定高切换到定高
	{
		if(thr_take_off<10)//未计算起飞油门
		{
			if(thr_set > -150)
			{
				thr_take_off = 400;
				
			}
		}
		en_old = en;
	}
	
	/*定高控制*/
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
				thr_take_off_f = 1; //用户可能想要起飞
				thr_take_off = 350; //直接赋值 一次
				
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
	baro_ctrl(T,&hc_value); //高度数据获取： 气压计数据
	
/////////////////////////////////////////////////////////////////////////////////		
	//计算高度误差（可加滤波）
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
	
	thr_pid_out = PID_calculate( T,            //周期
														exp_acc,				//前馈
														exp_acc,				//期望值（设定值）
														fb_acc,			//反馈值
														&h_acc_arg, //PID参数结构体
														&h_acc_val,	//PID数据结构体
														acc_i_lim*en			//integration limit，积分限幅
														 );			//输出		

	//step_filter(1000 *T,thr_pid_out,thr_pid_out_dlim);
	
	//起飞油门
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
	
	thr_take_off = LIMIT(thr_take_off,0,THR_TAKE_OFF_LIMIT); //一半
	
	
	//油门补偿
	tilted_fix = safe_div(1,LIMIT(reference_v.z,0.707f,1),0); //45度内补偿
	
	//油门输出
	thr_out = (thr_pid_out + tilted_fix *(thr_take_off) );
	
	thr_out = LIMIT(thr_out,0,1000);
	

	
/////////////////////////////////////////////////////////////////////////////////	
	static float dT,dT2;
	dT += T;
	speed_cnt++;
	if(speed_cnt>=10) //u8  20ms
	{

		exp_acc = PID_calculate( dT,            //周期
														exp_speed,				//前馈
														(set_speed + exp_speed),				//期望值（设定值）
														hc_value.fusion_speed,			//反馈值
														&h_speed_arg, //PID参数结构体
														&h_speed_val,	//PID数据结构体
														500 *en			//integration limit，积分限幅
														 );			//输出	
		
		exp_acc = LIMIT(exp_acc,-3000,3000);
		
		//integra_fix += (exp_speed - hc_value.m_speed) *dT;
		//integra_fix = LIMIT(integra_fix,-1500 *en,1500 *en);
		
		//LPF_1_(0.5f,dT,integra_fix,h_speed_val.err_i);
		
		dT2 += dT;
		height_cnt++;
		if(height_cnt>=10)  //200ms 
		{
			/////////////////////////////////////

		 exp_speed = PID_calculate( dT2,            //周期
																0,				//前馈
																0,				//期望值（设定值）
																-set_height_e,			//反馈值
																&h_height_arg, //PID参数结构体
																&h_height_val,	//PID数据结构体
																1500 *en			//integration limit，积分限幅
																 );			//输出	
			
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





/***************恒拓部分的定高 未能实现定高 放弃**********************/
struct Flag flag;

/******************高度控制变量********************/
float height_ctrl_out;
float wz_acc;
float keepheight_thr;
/*-------------------------------------------------*/

_HT_HAWK HT_HAWK;

/******************超声波变量********************/
#define ULTRA_SPEED 		 300    // mm/s
#define ULTRA_INT        300    // 积分幅度
u8 lock=0;
extern  s8 ultra_start_ok;;
extern float US100_Alt_delta;
float exp_height_speed,exp_height;
float ultra_speed,ultra_speed_acc;
float ultra_dis_lpf;
float ultra_ctrl_out;

/*-------------------------------------------------*/

/******************速度环变量********************/
extern float wz_speed;
float wz_acc_mms2;
float tempacc_lpf;
u8 lock_spd_crl=0;

/*-------------------------------------------------*/

/******************气压计变量********************/
float baro_height;
extern float baro_height_old;
#define BARO_SPEED 		 300    // mm/s
#define BARO_INT        300    // 积分幅度

u8 lock_BARO=0;//定高锁定当前以及清零用
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
			HT_HAWK.wz_speed_pid.kp= pid_setup.groups.hc_sp.kp;//变PID
			HT_HAWK.wz_speed_pid.ki= pid_setup.groups.hc_sp.ki;
			HT_HAWK.wz_speed_pid.kd= pid_setup.groups.hc_sp.kd;
			
			exp_height=ultra_dis_lpf;//这个超声波高度值经过滤波处理
			wz_acc=0;
			HT_HAWK.wz_speed_pid_v.err_i=0;
			ultra_speed=0;//若不清零，则且超声波时原来累积的速度会错误传递给当前，造成掉高
			HT_HAWK.ultra_ctrl.err_i=0;
			HT_HAWK.ultra_ctrl.err_old=0;
			lock=1;
			hight_PID_ctrl.shell.set_hight  = ultra.relative_height;
			
    }
//		else if(flag.FlightMode==MANUAL_High || flag.FlightMode==ATMOSPHERE_High)
	else if(flag.FlightMode==1)
		{lock=0;}


			//气压计暂时不用
//		if (flag.FlightMode==ATMOSPHERE_High && lock_BARO==0)
//		{
//			  wz_speed_pid.kp=baro_wz_speed_pid.kp;//变PID
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
		ultra_dis_tmp = HT_Moving_Median(1,5,ultra.relative_height);  //对超声波测量的距离进行移动中位值滤波
	
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
		
		//注意超声波测量的 距离的时效性问题，避免反复被计算
		ultra_sp_tmp = HT_Moving_Median(0,5,ultra.h_delta/T); //对超声波测距出来两次之间距离差计算的速度中值滤波 ultra_delta/T;

		if( ultra_dis_tmp < Ultrasonic_MAX_Height )//小于3米,注意这里，万一超出了呢？
		{
			if( ABS(ultra_sp_tmp) < 100 )//运动速度小于100mm/s
			{
				ultra_speed += ( 1 / ( 1 + 1 / ( 4 *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
			}//ultra_speed会传递给速度环，作为当前速度的反馈,原来是4
			else
			{
				ultra_speed += ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
			}//系数越大作用越小，原来是1
		}
	
}


void height_speed_ctrl(float T,float thr_keepheight,float exp_z_speed,float h_speed)
{

		//wz_speed被用于参与超声波高度D运算
		HT_HAWK.wz_speed_pid_v.err = HT_HAWK.wz_speed_pid.kp *( exp_z_speed - wz_speed );//
	
		HT_HAWK.wz_speed_pid_v.err_d = HT_HAWK.wz_speed_pid.kd * (-tempacc_lpf) *T;//(wz_speed_pid_v.err - wz_speed_pid_v.err_old);
		//wz_speed_pid_v.err_i += wz_speed_pid.ki *wz_speed_pid_v.err *T;
		HT_HAWK.wz_speed_pid_v.err_i += HT_HAWK.wz_speed_pid.ki *( exp_z_speed - h_speed ) *T;//期望速度与实际速度误差积分
	
		HT_HAWK.wz_speed_pid_v.err_i = LIMIT(HT_HAWK.wz_speed_pid_v.err_i,-Thr_Weight *200,Thr_Weight *200);
	

	
		HT_HAWK.wz_speed_pid_v.pid_out = thr_keepheight + Thr_Weight *LIMIT((HT_HAWK.wz_speed_pid.kp *exp_z_speed 
											+ HT_HAWK.wz_speed_pid_v.err + HT_HAWK.wz_speed_pid_v.err_d 
													+ HT_HAWK.wz_speed_pid_v.err_i),-300,300);//积分原来没有乘wz_speed_pid.kp
                               

	
		HT_HAWK.wz_speed_pid_v.err_old = HT_HAWK.wz_speed_pid_v.err; 
}

void Ultra_Ctrl(float T,float thr)
{
		exp_height_speed = ULTRA_SPEED *my_deathzoom_2(thr - 500,0,100)/300.0f; //20这里具体根据自己起飞油门定，油门控制高度+-ULTRA_SPEEDmm / s
	
		exp_height_speed = LIMIT(exp_height_speed ,-ULTRA_SPEED,ULTRA_SPEED);
		
		if( exp_height > Ultrasonic_MAX_Height )//超出超声波高度稳定范围则不执行
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
		
		exp_height += exp_height_speed *T;//累积期望高度，因为期望速度可能改变了

		HT_HAWK.ultra_ctrl.err = ( HT_HAWK.ultra_pid.kp*(exp_height - ultra_dis_lpf) );//mm 对高度误差乘以Kp
		

			
		HT_HAWK.ultra_ctrl.err_i += HT_HAWK.ultra_pid.ki *HT_HAWK.ultra_ctrl.err *T;//对高度误差积分
			
		HT_HAWK.ultra_ctrl.err_i = LIMIT(HT_HAWK.ultra_ctrl.err_i,-Thr_Weight *ULTRA_INT,Thr_Weight *ULTRA_INT);//积分限幅，融合了油门权重	
					//对于D，融合了加速度计运算得到的距离	
//		userdata2[7]=0.4f *(ultra_ctrl.err_old - ultra_ctrl.err);
		
		HT_HAWK.ultra_ctrl.err_d = HT_HAWK.ultra_pid.kd *( 0.5f *(-wz_speed*T) + 0.5f *(HT_HAWK.ultra_ctrl.err - HT_HAWK.ultra_ctrl.err_old) );
			

		
		HT_HAWK.ultra_ctrl.pid_out = HT_HAWK.ultra_ctrl.err +HT_HAWK.ultra_ctrl.err_i + HT_HAWK.ultra_ctrl.err_d;	
			
		HT_HAWK.ultra_ctrl.pid_out = LIMIT(HT_HAWK.ultra_ctrl.pid_out,-500,500);
	
		if(flag.FlightMode!=ULTRASONIC_High)
			HT_HAWK.ultra_ctrl.pid_out=0;//加入这代码用于测试加速度计定高效果,但仍然有速度反馈		
		
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
	
				if( mpu6050.Acc.z > 4000 )//注意这里，指水平静态下的Z值要比这个大，表示水平垂直    +-2G 即4096
				{
					  height_ctrl_start_f = 1;
				}
				break;
		
		case 1:
			
				LockForKeepHigh(thr_lpf);//一旦切到控高模式，立即保存当前的距离值，油门值、相关清零。
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
						lock_spd_crl=0;//切回手动模式后置零，以使下次进入定高模式是能顺利置零速度内环	
				}
			
				height_thr = LIMIT( thr , 0, 700 );
				thr_lpf += ( 1 / ( 1 + 1 / ( 2.0f *3.14f *T ) ) ) *( height_thr - thr_lpf );//对油门值低通滤波修正
				//userdata1[0]=	thr_lpf;//调试用
				/*下面的低通滤波用于测试对比效果，最终没有选用*/				
		    //wz_acc += ( 1 / ( 1 + 1 / ( 8 *3.14f *T ) ) ) *my_deathzoom( (V.z *(sensor.acc.averag.z- sensor.acc.quiet.z) + V.x *sensor.acc.averag.x + V.y *sensor.acc.averag.y - wz_acc),100 );//
		    //wz_acc_mms2 = (float)(wz_acc/8192.0f) *9800 ;
				//加速度的静态零点是读取的EEPROM数据，但是每次飞行都可能不一样，能每次起飞前校准一次最好，可以单独校准Z轴的，自行设计程序		
				
				//test = V.z *(sensor.acc.averag.z  - sensor.acc.quiet.z) + V.x *sensor.acc.averag.x + V.y *sensor.acc.averag.y;
				wz_acc_temp = sqrt(abs((int)mpu6050.Acc.x*(int)mpu6050.Acc.x) + abs(mpu6050.Acc.y*mpu6050.Acc.y) + abs(mpu6050.Acc.z*mpu6050.Acc.z)) - GY_530.ACC_z_zero;
						
				Moving_Average( acc_speed_arr,ACC_SPEED_NUM,acc_cnt ,wz_acc_temp,&wz_acc1 );		
				
	
				tempacc_lpf= (float)(wz_acc1/4096.0f) *9800;//9800 *T;由于是+-4G共8G，65535/8g=8192 g，加速度，mm/s2毫米每平方秒				
				
				if(abs(tempacc_lpf)<50)tempacc_lpf=0;//简单消除下噪声
						

							
				wz_speed_0 += tempacc_lpf *T;//加速度计积分成速度				
								
				if( ultra.start_ok == 1 )////不管是啥模式，只要更新了超声波数据就进行运算
				{	
					 Ultra_dataporcess(15.0f*TT);
					 ultra.start_ok=2;
				}
				
//				if(baro_start_f==1)//不管是啥模式，只要更新了气压数据就进行运算
//				{ 
//					 Baro_dataporcess(8.0f*TT);			 //8.0f*TT这么长时间更新一次气压计数据
//					 userdata2[10]=baro_height;//Baro_Height_Source
//					 baro_dis_delta=baro_height-baro_dis_old;
//					 baro_dis_old=baro_height;//气压计速度可以继续优化，这里比较粗糙			 
//		       //Moving_Average( (float)( baro_dis_delta/(8.0f*TT)),baro_speed_arr,BARO_SPEED_NUM,baro_cnt ,&baro_speed ); //单位mm/s
//					 userdata1[11]=baro_speed_lpf=0.4* baro_dis_delta/(8.0f*TT);//baro_speed这里乘以系数以削减该值		 
//					 baro_start_f=2;
//				}	
		
				if(flag.FlightMode==ULTRASONIC_High)
				{
					 h_speed=ultra_speed;
					 wz_speed_0 = 0.9f	*wz_speed_0 + 0.1f*h_speed;//超声波垂直速度互补滤波
				}				
//				else if(flag.FlightMode==ATMOSPHERE_High)
//				{
//					 h_speed=baro_speed_lpf;
//					 wz_speed_0 = 0.99	*wz_speed_0 + 0.01*h_speed;//气压计垂直速度互补滤波，系数可调
//				}	

				//h_speed是高度环传到速度环的实测高度方向速度【但可能是错误的，气压计速度不可靠】

				hc_speed_i += 0.4f *T *( h_speed - wz_speed );//速度偏差积分，乘以了0.4系数
				hc_speed_i = LIMIT( hc_speed_i, -500, 500 );//积分限幅	

				
				
				wz_speed_0 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( h_speed - wz_speed_0  ) ;//0.1实测速度修正加速度算的速度
				
				
				 wz_speed = wz_speed_0 + hc_speed_i;//经过修正的速度+经过限幅的增量式速度积分

//			  if( flag.FlightMode == ATMOSPHERE_High)
//				{
//						if(baro_start_f==2)//说明有新数据且被上面移动均值滤波使用过了
//						{
//							baro_start_f = -1;
//							Baro_Ctrl(8.0f*TT,thr_lpf);
//						}	
//						height_speed_ctrl(T,keepheight_thr,baro_ctrl_out,baro_speed_lpf);							
//				}

				if( flag.FlightMode == ULTRASONIC_High)
				{
						height_speed_ctrl(T,keepheight_thr,ultra_ctrl_out,ultra_speed);//系数原来是0.4
							
						if( ultra.start_ok == 2 )//超声波数据更新了且被运算了
						{				
								Ultra_Ctrl(15.0f*TT,thr_lpf);//realtime是周期为 TT 秒的，#define TT 0.0025//控制周期2.5ms
								ultra.start_ok = -1;
						}
				}
		
	      /*******************************************控制高度输出********************************************/
				if(flag.FlightMode==ULTRASONIC_High )//注意这里，模式
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
		HT_HAWK.ultra_pid.kp = pid_setup.groups.hc_height.kp;//超声波定高的高度位置环PID，会被EEPROM里的重新赋值的  //0.15
		HT_HAWK.ultra_pid.ki = pid_setup.groups.hc_height.ki;
		HT_HAWK.ultra_pid.kd = pid_setup.groups.hc_height.kd;	//2.5
}

void HT_WZ_Speed_PID_Init(void)
{
		HT_HAWK.ultra_wz_speed_pid.kp = 0.25;// //超声波定高的速度环PID，会被EEPROM里的重新赋值的  //
		HT_HAWK.ultra_wz_speed_pid.ki = 0.08; //0.12
		HT_HAWK.ultra_wz_speed_pid.kd = 8;
	
		HT_HAWK.baro_wz_speed_pid.kp = 0.1;// 气压定高的速度环PID，会被EEPROM里的重新赋值的
		HT_HAWK.baro_wz_speed_pid.ki = 0.06; 
		HT_HAWK.baro_wz_speed_pid.kd = 8;//1.4*10，在计算式乘以了10，已删除
	
		HT_HAWK.wz_speed_pid.kp = 0.1;//速度环默认PID，真正用不到被重新赋值
		HT_HAWK.wz_speed_pid.ki = 0.08;
		HT_HAWK.wz_speed_pid.kd = 8;
}

void HT_Baro_PID_Init(void)
{
		HT_HAWK.baro_pid.kp = 0.15;//气压定高的高度位置环PID，会被EEPROM里的重新赋值的
		HT_HAWK.baro_pid.ki = 0;
		HT_HAWK.baro_pid.kd = 1.5;	//2.5
}
