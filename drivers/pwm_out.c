/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：pwm_out.c
 * 描述    ：PWM输出
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "pwm_out.h"
#include "include.h"
#include "mymath.h"

//21分频到 84000000/21 = 4M   0.25us

#define INIT_DUTY 4000 //u16(1000/0.25)
#define ACCURACY 10000 //u16(2500/0.25) //accuracy
#define PWM_RADIO 4//(8000 - 4000)/1000.0

u8 PWM_Out_Init(uint16_t hz)//400hz
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	uint16_t PrescalerValue = 0;
	u32 hz_set = ACCURACY*hz;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);
	
	hz_set = LIMIT (hz_set,1,84000000);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
///////////////////////////////////////////////////////////////////////////////
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM2);
	
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIO_Pin_0 | GPIO_Pin_1 | 
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
//  GPIO_Init(GPIOA, &GPIO_InitStructure); 

////  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
////  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5); 
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);
//	
	/* Compute the prescaler value */
  PrescalerValue = (uint16_t) ( ( SystemCoreClock /2 ) / hz_set ) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);


  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM2, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
///////////////////////////////////////////////////////////////////////////////
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
//  GPIO_Init(GPIOE, &GPIO_InitStructure); 

//  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
//  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1); 
//  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
//  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
//	
//	/* Compute the prescaler value */
//  PrescalerValue = (uint16_t) ( ( SystemCoreClock ) / hz_set ) - 1;
//  /* Time base configuration */
//  TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
//  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);


//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

//  /* PWM1 Mode configuration: Channel1 */
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
//  //TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

//  /* PWM1 Mode configuration: Channel2 */
//  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
//  //TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

//  /* PWM1 Mode configuration: Channel3 */
//  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
//  //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

//  /* PWM1 Mode configuration: Channel4 */
//  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC4Init(TIM1, &TIM_OCInitStructure);
//  //TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
//	
//	TIM_CtrlPWMOutputs(TIM1, ENABLE);
//  TIM_ARRPreloadConfig(TIM1, ENABLE);
//  TIM_Cmd(TIM1, ENABLE);	
//	////////////////////////////////////////////////////////////////////////////////////

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; 
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
//  GPIO_Init(GPIOC, &GPIO_InitStructure); 

//  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
//  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);
//	
//	/* Compute the prescaler value */
//  PrescalerValue = (uint16_t) ( ( SystemCoreClock ) / hz_set ) - 1;
//  /* Time base configuration */
//  TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
//  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);


//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

//  /* PWM1 Mode configuration: Channel3 */
//  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC3Init(TIM8, &TIM_OCInitStructure);
//  //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

//  /* PWM1 Mode configuration: Channel4 */
//  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC4Init(TIM8, &TIM_OCInitStructure);
//  //TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
//	
//	TIM_CtrlPWMOutputs(TIM8, ENABLE);
//  TIM_ARRPreloadConfig(TIM8, ENABLE);
//  TIM_Cmd(TIM8, ENABLE);


	if( hz_set > 84000000 )
	{
		return 0;
	}
	else
	{
		return 1;
	}

////	GPIO_InitTypeDef GPIO_InitStructure;
//////	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
//////	TIM_OCInitTypeDef  				TIM_OCInitStructure;
////	
////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
////	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE );
////	
////	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
////	TIM_OCInitTypeDef  				TIM_OCInitStructure;

////	
////	
////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
////	
////	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
////  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
////  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
////  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
////	GPIO_Init(GPIOA, &GPIO_InitStructure);
////	
////	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);
////	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
////	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2);
////	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM2);

////	/**********************************************************
////	72 000 000/72=1M
////	1000 000/2500=400Hz
////	ËùÒÔ²úÉúµÄPWMÎª400Hz
////	ÖÜÆÚÎª2.5ms£¬¶ÔÓ¦2500µÄ¼ÆËãÖµ£¬1ms~2ms¶ÔÓ¦µÄ¼ÆËãÖµÎª1000~2000£»
////	**********************************************************/
////	TIM_TimeBaseStructure.TIM_Period = 2500;		//¼ÆÊýÉÏÏß	
////	TIM_TimeBaseStructure.TIM_Prescaler = 83;	//pwmÊ±ÖÓ·ÖÆµ
////	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
////	TIM_TimeBaseStructure.TIM_CounterMode =TIM_CounterMode_Up;		//ÏòÉÏ¼ÆÊý
//////	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
////	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
////	
////	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
////	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
////	TIM_OCInitStructure.TIM_Pulse = 1000;       //³õÊ¼Õ¼¿Õ±ÈÎª0
////	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
////	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
////	
////	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
////	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
////	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
////	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
////	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
////	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
////	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
////	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
////	
////	TIM_ARRPreloadConfig(TIM2, ENABLE);
////	TIM_Cmd(TIM2, ENABLE);
////	
////	return 1;
	
}

u8 CH_out_Mapping[MAXMOTORS] = {0,1,2,3};

void CH_out_Mapping_Fun(u16 *out,u16 *mapped )
{
	u8 i;
	for( i = 0 ; i < MAXMOTORS ; i++ )
	{
		*( mapped + i ) = *( out + CH_out_Mapping[i] );
	}
}

void SetPwm(int16_t pwm[MAXMOTORS],s16 min,s16 max)
{
	u8 i;
	s16 pwm_tem[MAXMOTORS];

	for(i=0;i<MAXMOTORS;i++)
	{
			pwm_tem[i] = pwm[i] ;
			pwm_tem[i] = LIMIT(pwm_tem[i],min,max);
	}
	
	
	TIM2->CCR1 = PWM_RADIO *( pwm_tem[CH_out_Mapping[0]] ) + INIT_DUTY;				//1	
	TIM2->CCR2 = PWM_RADIO *( pwm_tem[CH_out_Mapping[1]] ) + INIT_DUTY;				//2
	TIM2->CCR3 = PWM_RADIO *( pwm_tem[CH_out_Mapping[2]] ) + INIT_DUTY;				//3	
	TIM2->CCR4 = PWM_RADIO *( pwm_tem[CH_out_Mapping[3]] ) + INIT_DUTY;				//4
	
// 	TIM5->CCR4 = PWM_RADIO *( pwm_tem[CH_out_Mapping[4]] ) + INIT_DUTY;				//5	
// 	TIM5->CCR3 = PWM_RADIO *( pwm_tem[CH_out_Mapping[5]] ) + INIT_DUTY;				//6	
// 	TIM8->CCR4 = PWM_RADIO *( pwm_tem[CH_out_Mapping[6]] ) + INIT_DUTY;				//7	
// 	TIM8->CCR3 = PWM_RADIO *( pwm_tem[CH_out_Mapping[7]] ) + INIT_DUTY;				//8	
	
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
