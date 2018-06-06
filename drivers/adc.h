#ifndef __ADC_H
#define __ADC_H	
#include "stm32f4xx.h"

 							   
void Adc_Init(void); 				//ADC通道初始化
u16 Get_Adc(u8 ch)  ;
#endif 

