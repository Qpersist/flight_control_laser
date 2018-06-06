#ifndef _USART_H
#define _USART_H

#include "stm32f4xx.h"

extern u8 Rx_Buf[];
extern u8 mid_line;
void Usart1_Init(u32 br_num);
void Usart1_IRQ(void);
void Usart1_Send(unsigned char *DataToSend ,u8 data_num);


void Uart2_Init(u32 br_num);
void USART2_IRQHandler(void);
void Uart2_Send(unsigned char *DataToSend ,u8 data_num);

void Uart3_Init(u32 br_num);
void Uart3_Send(unsigned char *DataToSend ,u8 data_num);

#endif
