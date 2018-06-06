#include "warn.h"
#include "time.h"
#include "mpu6050.h"

void switch_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource2);//PE2 ���ӵ��ж���2
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource3);//PE3 ���ӵ��ж���3
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource4);//PE4 ���ӵ��ж���4
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource5);//PA0 ���ӵ��ж���0
	
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ�� 
	



	/* ����EXTI_Line2,3,4 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line2 | EXTI_Line3 | EXTI_Line4|EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½��ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//�ж���ʹ��
	EXTI_Init(&EXTI_InitStructure);//����

	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//�ⲿ�ж�2
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);//����


	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//�ⲿ�ж�3
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);//����


	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;//�ⲿ�ж�4
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);//����
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;//�ⲿ�ж�4
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);//����
}

void light_sound_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//ʹ��GPIOAʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ�� 
	
	GPIO_SetBits(GPIOF,GPIO_Pin_1);
	
	
	
}


void set_light_sound(u8 mode)
{
	static int T_count;
	if(mode == 1)
	{
		T_count ++;
		if(T_count /10 %2 ==0)
		{
			GPIO_SetBits(GPIOF,GPIO_Pin_1);
		}
		else 
			GPIO_ResetBits(GPIOF,GPIO_Pin_1);
	}
	else 
	{
		GPIO_SetBits(GPIOF,GPIO_Pin_1);
		T_count = 0;
	}
}




struct _Switch Switch;
struct _Switch Switch2;
struct _Switch Switch3;
//�ⲿ�ж�2�������
void EXTI2_IRQHandler(void)
{
	Delay_ms(5);
	if(KEY0==0)	  
	{				 
       Switch.count_mode ++;
		if(Switch.count_mode >=4)
			Switch.count_mode =0;
		if(Switch.count_mode == 2)
		{
			Duty.mode = 1;
		}
		else
			Duty.mode = Switch.count_mode ;
		Uart2_Send(&Switch.count_mode,1);
	}		 
	 EXTI_ClearITPendingBit(EXTI_Line2);//���LINE2�ϵ��жϱ�־λ 
}
//�ⲿ�ж�3�������
void EXTI3_IRQHandler(void)
{
	Delay_ms(10);
	if(KEY1==0)	 
	{
       
       Switch2.count_unlock ++;
		if(Switch2.count_unlock == 1)
		{
			Switch2.count_mode = 1;
			Switch2.flag = 1 ;
		}
		else if(Switch2.count_unlock == 2)
		{
			 Switch2.count_mode =0;
			Switch2.count_unlock = 0;
		}
	}		 
	 EXTI_ClearITPendingBit(EXTI_Line3);  //���LINE3�ϵ��жϱ�־λ  
}
//�ⲿ�ж�4�������

void EXTI4_IRQHandler(void)
{
	static u8 flag;
	Delay_ms(10);
	if(KEY2==0)	 
	{	
		
		Switch3.flag ++;
		
	     GPIO_ResetBits(GPIOF,GPIO_Pin_1);
	}		 
	 EXTI_ClearITPendingBit(EXTI_Line4);//���LINE4�ϵ��жϱ�־λ  
}




extern u8 change_CCD_data,mode_state,change_Hight_data,fly_ready;
extern float CH_filter[CH_NUM];

/*******************************
 һ������
�ı��־λ ����ң��������ͨ�� ��һ��ʧ�ر���ͨ��


************************/
void Auto_switch_2(float T)
{
	static float T_count= 0;
	static u8 falg =0 ;
	if( Switch2.count_mode == 1)
	{
		falg = 1 ;
		T_count += T;
		if(T_count < 8)
		{
			camera_Trance.sound = 1;
		}	
		else  if(T_count >= 8.0f&& T_count < 8.5f)
		{
//			CH_filter[ROL]= 0;
//			CH_filter[PIT]= 0;
//			CH_filter[THR]= 380;
//			CH_filter[YAW]= 0;
			
			camera_Trance.sound = 0;
			change_Hight_data = 1 ;
			mode_state = 2 ;
			change_CCD_data = 1;
			fly_ready = 1 ;
			
			
		}
		else 
		{
			CH_filter[ROL]= 0;
			CH_filter[PIT]= 0;
			CH_filter[THR]= -350;
			CH_filter[YAW]= 0;
//			CH_filter[AUX2] = 300 ;
		}
		
	}
	else 
	{
		T_count = 0;
		if(falg ==  1)
		{
			falg = 0 ;
			T_count = 0;
			camera_Trance.sound = 0;
			change_Hight_data = 0 ;
			mode_state = 2 ;
			change_CCD_data = 0;
		}
	}		
	
	
}



//У׼
void CALIBRATE_duty(float T)
{
	  
	 if(Switch3.flag >=1)
	 {
		Switch3.time += T; 
		 if(Switch3.time >=2.0f)
		 {
			Switch3.flag =0 ; 
			mpu6050.Acc_CALIBRATE = 1;		
			mpu6050.Gyro_CALIBRATE = 1; 
		 }
		 
	 }	

}

