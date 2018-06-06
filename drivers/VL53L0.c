#include "VL53L0.h"
#include "i2c_soft.h"
#include "include.h"
#include "mymath.h"
#include "fly_mode.h"
#include "imu.h"
#include <stdlib.h>
#include "mpu6050.h"
#include "height_ctrl.h"
#include "ultrasonic.h"
#include "ano_of.h"


GY530 GY_530 ;
extern float Roll,Pitch,Yaw;    				//��̬��

void GY_530_data(u8 *buff,float *data,u8 *GY_530_status)
{
	IIC_Write_1Byte(VL53L0X_Add,VL53L0X_REG_SYSRANGE_START,0x01);
	
	IIC_Read_nByte(VL53L0X_Add,0x14,12, buff);
	
	*data = makeuint16(buff[11], buff[10]);
	*GY_530_status= ((buff[0] & 0x78) >> 3);
}


void Hight_Duty(void)
{
	hight_PID_ctrl.shell.hight_pid_error = hight_PID_ctrl.shell.set_hight - GY_530.relative_height;
	hight_PID_ctrl.shell.hight_pid_merrer += hight_PID_ctrl.shell.hight_pid_error  ;
	hight_PID_ctrl.shell.hight_pid_deriv = (hight_PID_ctrl.shell.hight_pid_error - hight_PID_ctrl.shell.hight_pid_error_old) / 0.02f ;
	hight_PID_ctrl.shell.hight_pid_error_old =  hight_PID_ctrl.shell.hight_pid_error;
}
void GY_530_Duty(void)
{
	static float ALT_Hight[5];
	static int i,num;
	float sum=0;
	
	 
	GY_530_data(GY_530.data_buff,&GY_530.distance,&GY_530.status);
	
	if(GY_530.distance<2000&&GY_530.distance>20 && GY_530.status == 11 )
	{
//		GY_530.height = GY_530.distance*my_cos(Roll/57.3f)/my_cos(Pitch/57.3f)/10;  //cm��
			GY_530.height = GY_530.distance /10;
		GY_530.height = LPF_1st(GY_530.old_distance,GY_530.height,0.5);
//		
//		ALT_Hight[num] = GY_530.height ;
//		for(i=0;i<5;i++)
//		{
//			sum += ALT_Hight[i];
//		}
//		
//		
//		num = (num + 1) % 5;
//		
//		GY_530.height = sum / 5;
		if(num <=100)
			num ++;
		if(num > 100)
		{
			if(GY_530.height>GY_530.old_distance +10 ||GY_530.height< GY_530.old_distance -10)
			{
				 GY_530.height = GY_530.old_distance ;
			}
		}
		
		
		GY_530.old_distance= GY_530.height;
		
		
		GY_530.start_ok = 1;
	}
	else 
	{
		GY_530.height =  GY_530.old_distance;
	}

	GY_530.relative_height = GY_530.height ;
	GY_530.h_delta = GY_530.relative_height - GY_530.old_distance;


    if(GY_530.relative_height>50&&GY_530.relative_height<150)
	{
		GY_530.warning_flag = 1;
	}
    else 
		GY_530.warning_flag = 0;
	
	if(GY_530.warning_flag ==1)
	{
		GY_530.warning_count ++;
		if(GY_530.warning_count /10%2 ==1)
		{
			GPIO_SetBits(GPIOF,GPIO_Pin_1);
		}
		else
			GPIO_ResetBits(GPIOF,GPIO_Pin_1);
	}
	else
		GPIO_SetBits(GPIOF,GPIO_Pin_1);
	
//	USART_SendData(USART2, GY_530.warning_flag);         //�򴮿�1��������
//	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	Uart2_Send(&GY_530.warning_flag ,1);
}




uint16_t bswap(u8 b[])
{
	uint16_t val = ((b[0]<< 8) & b[1]);
	return val;
}

uint16_t VL53L0X_decode_vcsel_period(short vcsel_period_reg)
{
	uint16_t vcsel_period_pclks = (vcsel_period_reg + 1) <<1;
	return vcsel_period_pclks;
}

uint16_t makeuint16(int lsb, int msb)
{
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}


/**********************************/
extern float thr_value,thr_value_old;

void GY_530_hight_ctrl(float T)
{
	static float acc_temp;
	static float acc_speed;
	static float H_speed;
	static float F_speed;
		if(mode_state == 0 || mode_state == 1) //�ֶ�����
		{
			//��ջ���֮�����ֵ
			//�������ţ��߶�
			hight_PID_ctrl.shell.set_hight = ultra.relative_height;
			
				
				hight_PID_ctrl.shell.hight_pid_merrer = 0;
//				hight_PID_ctrl.shell.hight_pid_error_old = ultra.relative_height; 

		}
		else if(mode_state == 2 )
		{
			if((thr_value<thr_value_old-50))  //����Ҫ�½�  ����С�����߶� ���������
				hight_PID_ctrl.shell.set_hight += (thr_value-thr_value_old)/30000.0f;
			else if(thr_value> thr_value_old +50)  //����Ҫ���� �����������߶� ���������
				hight_PID_ctrl.shell.set_hight += (thr_value-thr_value_old)/30000.0f;
				
				
			acc_temp = sqrt(abs((int)mpu6050.Acc.x*(int)mpu6050.Acc.x) + abs(mpu6050.Acc.y*mpu6050.Acc.y) + abs(mpu6050.Acc.z*mpu6050.Acc.z)) 
							- GY_530.ACC_z_zero;
			acc_speed = (acc_temp / 4096) * 9800;//9800 *T;������+-4G��8G��65535/8g=8192 g�����ٶȣ�mms2����ÿƽ����				
			if(abs(acc_speed)<50)acc_speed=0;//������������
			
			acc_speed += acc_speed * T;
			if(ultra.start_ok == 1)
			{
				H_speed = ultra.h_delta / T ; 
				ultra.start_ok = 3;
			}
			F_speed = 0.9f	*acc_speed + 0.1f*H_speed;//��������ֱ�ٶȻ����˲�
			
			hight_PID_ctrl.shell.hight_pid_error = (hight_PID_ctrl.shell.set_hight -GY_530.relative_height) ;//��λ���׵�����

			hight_PID_ctrl.shell.hight_pid_KP_out= pid_setup.groups.ctrl3.kp* hight_PID_ctrl.shell.hight_pid_error;
			
			hight_PID_ctrl.shell.hight_pid_merrer +=  hight_PID_ctrl.shell.hight_pid_error ;
			hight_PID_ctrl.shell.hight_pid_merrer= LIMIT(hight_PID_ctrl.shell.hight_pid_merrer,-50,50);
			hight_PID_ctrl.shell.hight_pid_KI_out= pid_setup.groups.ctrl3.ki* hight_PID_ctrl.shell.hight_pid_merrer;
			
			hight_PID_ctrl.core.hight_pid_KD_out= pid_setup.groups.ctrl3.kd* F_speed;
			
			hight_PID_ctrl.shell.hight_pid_out = hight_PID_ctrl.shell.hight_pid_KP_out + hight_PID_ctrl.shell.hight_pid_KI_out
																	+ hight_PID_ctrl.shell.hight_pid_KD_out;
			
		}
}

