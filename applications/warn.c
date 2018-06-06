#include "warn.h"
#include "time.h"
#include "mpu6050.h"

void switch_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource2);//PE2 连接到中断线2
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource3);//PE3 连接到中断线3
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource4);//PE4 连接到中断线4
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource5);//PA0 连接到中断线0
	
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化 
	



	/* 配置EXTI_Line2,3,4 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line2 | EXTI_Line3 | EXTI_Line4|EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//中断线使能
	EXTI_Init(&EXTI_InitStructure);//配置

	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//外部中断2
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);//配置


	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//外部中断3
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);//配置


	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;//外部中断4
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;//外部中断4
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);//配置
}

void light_sound_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//使能GPIOA时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化 
	
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
//外部中断2服务程序
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
	 EXTI_ClearITPendingBit(EXTI_Line2);//清除LINE2上的中断标志位 
}
//外部中断3服务程序
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
	 EXTI_ClearITPendingBit(EXTI_Line3);  //清除LINE3上的中断标志位  
}
//外部中断4服务程序

void EXTI4_IRQHandler(void)
{
	static u8 flag;
	Delay_ms(10);
	if(KEY2==0)	 
	{	
		
		Switch3.flag ++;
		
	     GPIO_ResetBits(GPIOF,GPIO_Pin_1);
	}		 
	 EXTI_ClearITPendingBit(EXTI_Line4);//清除LINE4上的中断标志位  
}




extern u8 change_CCD_data,mode_state,change_Hight_data,fly_ready;
extern float CH_filter[CH_NUM];

/*******************************
 一键启动
改变标志位 放弃遥控器方向通道 留一个失控保护通道


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



//校准
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

