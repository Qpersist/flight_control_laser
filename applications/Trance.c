#include "Trance.h"
#include "ctrl.h"
#include "ultrasonic.h"
#include "fly_mode.h"
#include "PID.h"
#include "ano_of.h"
#include "warn.h"



struct _CCD_PID  CCD_CTRL_PID;
struct _CCD_PID_H CCD_CTRL_PID_H;
struct _camera_Tance camera_Trance;
struct _Duty Duty;

// void CCD_ctrl(u8 mid,float T,u8 mode)
//{

//	if(camera_Trance.take_off_flag == 1) 
//	{
//		CCD_CTRL_PID.KI_out = 5;
//		CCD_CTRL_PID_H.KI_out = 5;
//	}
//	else 
//	{
//		CCD_CTRL_PID.KI_out =0 ;
//		CCD_CTRL_PID_H.KI_out =0;
//	}
//	
////	camera_Trance.PID_out = camera_Trance.KP_out + camera_Trance.KI_out + camera_Trance.KD_out ;
//	
////   camera_Trance.PID_out =  pid_setup.groups.ctrl4.kp * CCD_CTRL_PID.shell_Pout;	
///******************************************/  
//	if(mode == 0)
//	{
//	   CCD_CTRL_PID.KP_out =  pid_setup.groups.ctrl3.kp *  (CCD_CTRL_PID.error + camera_Trance.PID_out_H) ;
////	   CCD_CTRL_PID.KP_out =  pid_setup.groups.ctrl3.kp* CCD_CTRL_PID.shell_Pout ;
////	   CCD_CTRL_PID.KI_out = pid_setup.groups.ctrl3.ki * CCD_CTRL_PID.merrer ;
//	   CCD_CTRL_PID.KD_out = pid_setup.groups.ctrl3.kd * CCD_CTRL_PID.devir ;
//	 
//	   CCD_CTRL_PID.PID_out = CCD_CTRL_PID.KP_out + CCD_CTRL_PID.KI_out + CCD_CTRL_PID.KD_out ;


//		CCD_CTRL_PID_H.KP_out = pid_setup.groups.ctrl3.kp * (CCD_CTRL_PID_H.error + camera_Trance.PID_out_V) ;
////		CCD_CTRL_PID_H.KI_out = pid_setup.groups.ctrl3.ki * CCD_CTRL_PID_H.merrer ;
//		CCD_CTRL_PID_H.KD_out = pid_setup.groups.ctrl3.kd * CCD_CTRL_PID_H.devir ;
//		
//		CCD_CTRL_PID_H.PID_out = CCD_CTRL_PID_H.KP_out + CCD_CTRL_PID_H.KI_out + CCD_CTRL_PID_H.KD_out ;
//	}
//	else if(mode == 1)
//	{
//		CCD_CTRL_PID.KP_out =  pid_setup.groups.ctrl3.kp *  (CCD_CTRL_PID.error + CCD_CTRL_PID.shell_Pout) ;
////	   CCD_CTRL_PID.KP_out =  pid_setup.groups.ctrl3.kp* CCD_CTRL_PID.shell_Pout ;
//	   CCD_CTRL_PID.KI_out = pid_setup.groups.ctrl3.ki * CCD_CTRL_PID.merrer ;
//	   CCD_CTRL_PID.KD_out = pid_setup.groups.ctrl3.kd * CCD_CTRL_PID.devir ;
//	 
//	   CCD_CTRL_PID.PID_out = CCD_CTRL_PID.KP_out + CCD_CTRL_PID.KI_out + CCD_CTRL_PID.KD_out ;


////		CCD_CTRL_PID_H.KP_out = pid_setup.groups.ctrl3.kp * (CCD_CTRL_PID_H.error ) ;
////		CCD_CTRL_PID_H.KI_out = pid_setup.groups.ctrl3.ki * CCD_CTRL_PID_H.merrer ;
////		CCD_CTRL_PID_H.KD_out = pid_setup.groups.ctrl3.kd * CCD_CTRL_PID_H.devir ;
//		
//		CCD_CTRL_PID_H.PID_out = 0 ;
//	}
//	else if(mode == 2)
//	{
//		 CCD_CTRL_PID.KP_out =  pid_setup.groups.ctrl3.kp *  (CCD_CTRL_PID.error + camera_Trance.PID_out_H) ;
////	   CCD_CTRL_PID.KP_out =  pid_setup.groups.ctrl3.kp* CCD_CTRL_PID.shell_Pout ;
////	   CCD_CTRL_PID.KI_out = pid_setup.groups.ctrl3.ki * CCD_CTRL_PID.merrer ;
//	   CCD_CTRL_PID.KD_out = pid_setup.groups.ctrl3.kd * CCD_CTRL_PID.devir ;
//	 
//	   CCD_CTRL_PID.PID_out = CCD_CTRL_PID.KP_out + CCD_CTRL_PID.KI_out + CCD_CTRL_PID.KD_out ;


//		CCD_CTRL_PID_H.KP_out = pid_setup.groups.ctrl3.kp * (CCD_CTRL_PID_H.error + camera_Trance.PID_out_V) ;
////		CCD_CTRL_PID_H.KI_out = pid_setup.groups.ctrl3.ki * CCD_CTRL_PID_H.merrer ;
//		CCD_CTRL_PID_H.KD_out = pid_setup.groups.ctrl3.kd * CCD_CTRL_PID_H.devir ;
//		
//		CCD_CTRL_PID_H.PID_out = CCD_CTRL_PID_H.KP_out + CCD_CTRL_PID_H.KI_out + CCD_CTRL_PID_H.KD_out ;
//	}
//		
//}

///*******************************************************
//    mode_state 						飞行模式，姿态，定高
//    change_CCD_data     			开始循迹标志位
//    camera_Trance.take_off_flag     自动降落标志位	降落时不前进
//    camera_Trance.trance_flag       飞行方向
//	duty_mode :     				飞行模式   下位机结合按键模式
//									0  ：定点 1：前进     2：跟踪  
//    Duty.fly_mode ：			    0 定点，1 前进
//	Duty.time：						按时降落
//    Duty.time_cycle ： 				运行时间
//*******************************************************/

//void direction_control(float T,u8 duty_mode)
//{
////	u8 mode ;
//	
//	if( mode_state ==2 )
//	{

//		if(change_CCD_data ==1)
//		{
//			Duty.time_cycle += T;
//			 if(duty_mode == 0)
//			 {
//				Duty.fly_mode=0 ; 
//				 
//			 }
//			 else if(duty_mode == 1)
//			 {
//			    Duty.fly_mode=1 ;
//			 }
//			 else if(duty_mode == 2 )
//			 {
//				Duty.fly_mode=2 ; 
//			 }
//			
//			CCD_ctrl(mid_line,T,Duty.fly_mode);
//			if(Duty.fly_mode == 0)
//			{
//				except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f );   //30 ;   //max 30度	
//				except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f ); 
//				
//				Duty.time += T;
//				if(Duty.time >= 15.0f && Switch.count_mode== 0 )
//				{
//					camera_Trance.take_off_flag = 1;
//				}
////				if(Duty.time >= 20.0f && Switch.count_mode== 1 )
////				{
////					camera_Trance.take_off_flag = 1;
////				}
//				
//			}
//			else if(Duty.fly_mode ==1)
//			{
//				except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f );   //30 ;   //max 30度	
//				except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( (-70) ,0,30 )/500.0f ); 
//				
//				Duty.time= 0;
//			}
//			else if(Duty.fly_mode == 2)
//			{
//				except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f );   //30 ;   //max 30度	
//				except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f ); 
//	
//				Duty.time += T;
//				if(Duty.time >= 40.0f)
//				{
//					camera_Trance.take_off_flag = 1;
////					camera_Trance.trance_flag = 1;
//				}
//			}
//			

//			
//		}
//		else 
//		{
//			Duty.time_cycle = 0;
//			Duty.time = 0;
//			except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( ( CH_filter[ROL]) ,0,30 )/500.0f );   //30
//			except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( (-CH_filter[PIT]) ,0,30 )/500.0f );  //30
//			
//			CCD_CTRL_PID.PID_out =0 ;
//			CCD_CTRL_PID.shell_Pout = 0;
//			CCD_CTRL_PID.merrer = 0;
//			CCD_CTRL_PID_H.shell_Pout = 0;
//			CCD_CTRL_PID_H.PID_out  = 0;
//			CCD_CTRL_PID_H.merrer = 0;
//			
//			camera_Trance.merrer_H = 0;
//			camera_Trance.merrer_V = 0;
//			camera_Trance.PID_out_H = 0 ;
//			camera_Trance.PID_out_V = 0 ;
//  		}
//	    
//		
//	}
//	else 
//	{
//		Duty.time_cycle = 0;
//			Duty.time = 0;
//		except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( ( CH_filter[ROL]) ,0,30 )/500.0f );   //30
//		except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( (-CH_filter[PIT]) ,0,30 )/500.0f );  //30


//		CCD_CTRL_PID.PID_out =0 ;
//		CCD_CTRL_PID.shell_Pout = 0;
//	    CCD_CTRL_PID.merrer = 0;
//		CCD_CTRL_PID_H.shell_Pout = 0;
//		CCD_CTRL_PID_H.PID_out  = 0;
//		CCD_CTRL_PID_H.merrer = 0;
//		
//		camera_Trance.merrer_H = 0;
//		camera_Trance.merrer_V = 0;
//		camera_Trance.PID_out_H = 0 ;
//		camera_Trance.PID_out_V = 0 ;
//	}
//}

extern u8 CCD_OK_flag;
void camera_shell_pid(void)
{
	if(CCD_OK_flag == 1)
	{
		if( camera_Trance.data >-50 && camera_Trance.data< 50)
			camera_Trance.error_H  =  (float)(camera_Trance.data  - OF_ALT2 * tan(Roll / 57.3f));
		if( camera_Trance.data_V >-50 && camera_Trance.data_V< 50)
			camera_Trance.error_V  =  (float)(camera_Trance.data_V  - OF_ALT2 * tan(Pitch / 57.3f));
		camera_Trance.error_H =  LIMIT(camera_Trance.error_H,-60,60);
		camera_Trance.KP_out_H = pid_setup.groups.ctrl4.kp * camera_Trance.error_H;

		camera_Trance.merrer_H += camera_Trance.error_H;
		camera_Trance.merrer_H = LIMIT(camera_Trance.merrer_H,-50,50);
		camera_Trance.KI_out_H = pid_setup.groups.ctrl4.ki *   camera_Trance.merrer_H;
		
//		camera_Trance.devir_H = (camera_Trance.error_H - camera_Trance.error_H_old);
		camera_Trance.KD_out_H = pid_setup.groups.ctrl4.kd * camera_Trance.devir_H;
//		camera_Trance.error_H_old = camera_Trance.error_H;
/***********************************************************************/
		camera_Trance.KP_out_V = pid_setup.groups.ctrl4.kp * camera_Trance.error_V;

		camera_Trance.merrer_V += camera_Trance.error_V;
		camera_Trance.merrer_V = LIMIT(camera_Trance.merrer_V,-50,50);
		camera_Trance.KI_out_V = pid_setup.groups.ctrl4.ki *   camera_Trance.merrer_V;
		
//		camera_Trance.devir_V = (camera_Trance.error_V - camera_Trance.error_V_old);
		camera_Trance.KD_out_V = pid_setup.groups.ctrl4.kd * camera_Trance.devir_V;
//		camera_Trance.error_V_old = camera_Trance.error_V;

		
		camera_Trance.PID_out_H = camera_Trance.KP_out_H + camera_Trance.KI_out_H + camera_Trance.KD_out_H ; 
		camera_Trance.PID_out_V = camera_Trance.KP_out_V + camera_Trance.KI_out_V + camera_Trance.KD_out_V ;
		CCD_OK_flag=0;
	}
}
void mode_0_duty(float T,u8 mode)
{
	static u8 control_count = 0;
	Duty.time += T;                         //计时器  开始之后15s之后降落       由于启动油门需要时间只有10s左右悬停
	//期望角度
	except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f );   //30 ;   //max 30度	
	except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f ); 
	
	//计算PID  CCD_CTRL_PID.PID_out 横轴方向 CCD_CTRL_PID_H.PID_out 纵轴方向
	CCD_CTRL_PID.KP_out =  pid_setup.groups.ctrl3.kp *  (CCD_CTRL_PID.error) ;
	CCD_CTRL_PID.KD_out = pid_setup.groups.ctrl3.kd * CCD_CTRL_PID.devir ;

	CCD_CTRL_PID.PID_out = CCD_CTRL_PID.KP_out + CCD_CTRL_PID.KI_out + CCD_CTRL_PID.KD_out ;


	CCD_CTRL_PID_H.KP_out = pid_setup.groups.ctrl3.kp * (CCD_CTRL_PID_H.error) ;
	CCD_CTRL_PID_H.KD_out = pid_setup.groups.ctrl3.kd * CCD_CTRL_PID_H.devir ;

	CCD_CTRL_PID_H.PID_out = CCD_CTRL_PID_H.KP_out + CCD_CTRL_PID_H.KI_out + CCD_CTRL_PID_H.KD_out ;
	
	if(Duty.time > 15.0f)
		camera_Trance.take_off_flag = 1;
//	else 
//		hight_PID_ctrl.core.set_hight = 1200; 
}

 
void mode_1_duty(float T,u8 mode)
{
	Duty.time += T;
	
	if(Duty.time <= 3.0f)
	{
		except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f );   //30 ;   //max 30度	
		except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f ); 
		
		CCD_CTRL_PID.KP_out =  pid_setup.groups.ctrl3.kp *  (CCD_CTRL_PID.error ) ;
		CCD_CTRL_PID.KD_out = pid_setup.groups.ctrl3.kd * CCD_CTRL_PID.devir ;

		CCD_CTRL_PID.PID_out = CCD_CTRL_PID.KP_out + CCD_CTRL_PID.KI_out + CCD_CTRL_PID.KD_out ;


		CCD_CTRL_PID_H.KP_out = pid_setup.groups.ctrl3.kp * (CCD_CTRL_PID_H.error ) ;
		CCD_CTRL_PID_H.KD_out = pid_setup.groups.ctrl3.kd * CCD_CTRL_PID_H.devir ;

		CCD_CTRL_PID_H.PID_out = CCD_CTRL_PID_H.KP_out + CCD_CTRL_PID_H.KI_out + CCD_CTRL_PID_H.KD_out ;

	}
	else if(Duty.time <=13.0f)
	{
		if(mode == 1)
		{                        
			except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f );   //30 ;   //max 30度	
			except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( (-80) ,0,30 )/500.0f ); 

			CCD_CTRL_PID.KP_out =  pid_setup.groups.ctrl3.kp *  (CCD_CTRL_PID.error + CCD_CTRL_PID.shell_Pout) ;
			CCD_CTRL_PID.KD_out = pid_setup.groups.ctrl3.kd * CCD_CTRL_PID.devir ;

			CCD_CTRL_PID.PID_out = CCD_CTRL_PID.KP_out + CCD_CTRL_PID.KI_out + CCD_CTRL_PID.KD_out ;

			CCD_CTRL_PID_H.PID_out = 0 ;
			CCD_CTRL_PID.shell_Pout = 0;
		}
		else if(mode == 0)
		{
			except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f );   //30 ;   //max 30度	
			except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f ); 

			CCD_CTRL_PID.KP_out =  pid_setup.groups.ctrl3.kp *  (CCD_CTRL_PID.error ) ;
			CCD_CTRL_PID.KD_out = pid_setup.groups.ctrl3.kd * CCD_CTRL_PID.devir ;
			CCD_CTRL_PID.PID_out = CCD_CTRL_PID.KP_out + CCD_CTRL_PID.KI_out + CCD_CTRL_PID.KD_out ;


			CCD_CTRL_PID_H.KP_out = pid_setup.groups.ctrl3.kp * (CCD_CTRL_PID_H.error ) ;
			CCD_CTRL_PID_H.KD_out = pid_setup.groups.ctrl3.kd * CCD_CTRL_PID_H.devir ;

			CCD_CTRL_PID_H.PID_out = CCD_CTRL_PID_H.KP_out + CCD_CTRL_PID_H.KI_out + CCD_CTRL_PID_H.KD_out ;
		}
	}
	else if(Duty.time >13.0f)
	{
		 camera_Trance.take_off_flag = 1;
		 except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( (50) ,0,30 )/500.0f );   //右停;   //max 30度	  后右
		except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( (40) ,0,30 )/500.0f );
	}
	
	
}

void mode_2_duty(float T,u8 mode)
{
	Duty.time += T;
	
	if(Duty.time <= 3.0f)
	{
		except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f );   //30 ;   //max 30度	
		except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f ); 
		
		CCD_CTRL_PID.KP_out =  pid_setup.groups.ctrl3.kp *  (CCD_CTRL_PID.error ) ;
		CCD_CTRL_PID.KD_out = pid_setup.groups.ctrl3.kd * CCD_CTRL_PID.devir ;

		CCD_CTRL_PID.PID_out = CCD_CTRL_PID.KP_out + CCD_CTRL_PID.KI_out + CCD_CTRL_PID.KD_out ;


		CCD_CTRL_PID_H.KP_out = pid_setup.groups.ctrl3.kp * (CCD_CTRL_PID_H.error ) ;
		CCD_CTRL_PID_H.KD_out = pid_setup.groups.ctrl3.kd * CCD_CTRL_PID_H.devir ;

		CCD_CTRL_PID_H.PID_out = CCD_CTRL_PID_H.KP_out + CCD_CTRL_PID_H.KI_out + CCD_CTRL_PID_H.KD_out ;
		CCD_CTRL_PID.shell_Pout= 0;

	}
	else
	{
		if(mode == 1)
		{
			except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f );   //30 ;   //max 30度	
			except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( (-80) ,0,30 )/500.0f ); 

			CCD_CTRL_PID.KP_out =  pid_setup.groups.ctrl3.kp *  (CCD_CTRL_PID.error + CCD_CTRL_PID.shell_Pout) ;
			CCD_CTRL_PID.KD_out = pid_setup.groups.ctrl3.kd * CCD_CTRL_PID.devir ;

			CCD_CTRL_PID.PID_out = CCD_CTRL_PID.KP_out + CCD_CTRL_PID.KI_out + CCD_CTRL_PID.KD_out ;

			CCD_CTRL_PID_H.PID_out = 0 ;
		}
		else if(mode == 0)
		{
			except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f );   //30 ;   //max 30度	
			except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f ); 

			CCD_CTRL_PID.KP_out =  pid_setup.groups.ctrl3.kp *  (CCD_CTRL_PID.error ) ;
			CCD_CTRL_PID.KD_out = pid_setup.groups.ctrl3.kd * CCD_CTRL_PID.devir ;
			CCD_CTRL_PID.PID_out = CCD_CTRL_PID.KP_out + CCD_CTRL_PID.KI_out + CCD_CTRL_PID.KD_out ;


			CCD_CTRL_PID_H.KP_out = pid_setup.groups.ctrl3.kp * (CCD_CTRL_PID_H.error ) ;
			CCD_CTRL_PID_H.KD_out = pid_setup.groups.ctrl3.kd * CCD_CTRL_PID_H.devir ;

			CCD_CTRL_PID_H.PID_out = CCD_CTRL_PID_H.KP_out + CCD_CTRL_PID_H.KI_out + CCD_CTRL_PID_H.KD_out ;
		}
	}

	

}



 /*******************************************************
    mode_state 						飞行模式，姿态，定高
    change_CCD_data     			开始循迹标志位
    camera_Trance.take_off_flag     自动降落标志位	降落时不前进
    camera_Trance.trance_flag       飞行方向
	duty_mode :     				飞行模式   下位机结合按键模式		电赛要求的前进定点与直接定点
									0  ：定点 1：前进     2：跟踪  		
    Duty.fly_mode ：			    0 定点，1 前进
	Duty.time：						按时降落
    Duty.time_cycle ： 				运行时间
*******************************************************/

void direction_control(float T,u8 duty_mode)
{
	static u8 control_flag= 0;
	camera_shell_pid();
	if( mode_state ==2 )
	{

		if(change_CCD_data ==1)
		{
			Duty.time_cycle += T;
			
//			if(control_flag == 0)
//				camera_shell_pid();
//			 control_flag ++;
//			
//			if(control_flag == 2)
//				control_flag = 0;
			mode_2_duty(T,0);
//			 if(duty_mode == 0)
//			 {
//				mode_0_duty(T,0);            //定点
//			 }
//			 else if(duty_mode == 1)
//			 {
//				mode_1_duty(T,Duty.mode);    //跟踪模式   时间短
//			 }
//			 else if(duty_mode == 2 )
//			 {
//                mode_2_duty(T,Duty.mode);
//			 }
			 
			 if(camera_Trance.take_off_flag == 1 && duty_mode ==0)               //开始降落 停在旁边
			 {
				except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f );   //右停;   //max 30度	
				except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( (0) ,0,30 )/500.0f ); 
			 }   

			
		}
		else 
		{
			Duty.time_cycle = 0;
			Duty.time = 0;
			except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( ( CH_filter[ROL]) ,0,30 )/500.0f );   //30
			except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( (-CH_filter[PIT]) ,0,30 )/500.0f );  //30
			
			CCD_CTRL_PID.PID_out =0 ;
			CCD_CTRL_PID.shell_Pout = 0;
			CCD_CTRL_PID.merrer = 0;
			CCD_CTRL_PID_H.shell_Pout = 0;
			CCD_CTRL_PID_H.PID_out  = 0;
			CCD_CTRL_PID_H.merrer = 0;
			
			camera_Trance.merrer_H = 0;
			camera_Trance.merrer_V = 0;
			camera_Trance.PID_out_H = 0 ;
			camera_Trance.PID_out_V = 0 ;
  		}
	    
		
	}
	else 
	{
		Duty.time_cycle = 0;
			Duty.time = 0;
		except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( ( CH_filter[ROL]) ,0,30 )/500.0f );   //30
		except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( (-CH_filter[PIT]) ,0,30 )/500.0f );  //30


		CCD_CTRL_PID.PID_out =0 ;
		CCD_CTRL_PID.shell_Pout = 0;
	    CCD_CTRL_PID.merrer = 0;
		CCD_CTRL_PID_H.shell_Pout = 0;
		CCD_CTRL_PID_H.PID_out  = 0;
		CCD_CTRL_PID_H.merrer = 0;
		
		camera_Trance.merrer_H = 0;
		camera_Trance.merrer_V = 0;
		camera_Trance.PID_out_H = 0 ;
		camera_Trance.PID_out_V = 0 ;
	}
	
}

void feed_error(int Ori_H_error,int Ori_V_error, int *H_error,int *V_error,float Hight)
{
	*H_error = Ori_H_error  - Hight * tan(Roll/57.3f) ;
}












