/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：init.c
 * 描述    ：飞控初始化
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "include.h"
#include "pwm_out.h"
#include "mpu6050.h"
#include "i2c_soft.h"
#include "led.h"
#include "ctrl.h"
#include "ms5611.h"
#include "ak8975.h"
#include "ultrasonic.h"
#include "filter.h"
#include "PID.h"

u8 All_Init()
{
	NVIC_PriorityGroupConfig(NVIC_GROUP);		//中断优先级组别设置
	
	SysTick_Configuration(); 			//滴答时钟
	
	I2c_Soft_Init();					//初始化模拟I2C
	
	PWM_IN_Init();						//初始化接收机采集功能
	
	PWM_Out_Init(400);				//初始化电调输出功能	
	
//	Usb_Hid_Init();						//飞控usb接口的hid初始化
	
	MS5611_Init();						//气压计初始化
	
	Delay_ms(400);						//延时
	
	MPU6050_Init(20);   			//加速度计、陀螺仪初始化，配置20hz低通
	
	LED_Init();								//LED功能初始化
	
	Usart1_Init(500000);			//串口初始化，函数参数为波特率
	Uart2_Init(115200);
	Uart3_Init(500000);
	
	//TIM_INIT();
	
	Para_Init();							//参数初始化
    AT24CXX_Init();                         //24c02初始化
	light_sound_init();                     //声光报警
	switch_init();                          //按键
	
	Delay_ms(100);						//延时

	ak8975_ok = !(ANO_AK8975_Run());

	
	if( !mpu6050_ok )
	{
		LED_MPU_Err();
	}
	else if( !ak8975_ok )
	{
		LED_Mag_Err();
	}
	else if( !ms5611_ok )
	{
		LED_MS5611_Err();
	}

	
	Cycle_Time_Init();
	lpf2pInit(&LPF2filter , 500,4.0f);
	
	camera_Trance.count = 0 ;
	camera_Trance.take_off_flag = 0;
	camera_Trance.trance_flag = 1;
	camera_Trance.sound = 1;
	hight_PID_ctrl.core.set_hight = pid_setup.groups.hc_sp.kp * 100;
	GY_530.ACC_z_zero = 4096;
	
 	return (1);
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
