#ifndef __24CXX_H
#define __24CXX_H
#include "include.h"   
	
/************************************************/
#define AT24C01		127
#define AT24C02		255
#define AT24C04		511
#define AT24C08		1023
#define AT24C16		2047
#define AT24C32		4095
#define AT24C64	    8191
#define AT24C128	16383
#define AT24C256	32767  

#define EE_TYPE AT24C02
/***************************************************/
#define save_gyro_x           0x30
#define save_gyro_y           0x34
#define save_gyro_z           0x38
 
#define save_acc_x            0x3c
#define save_acc_y            0x40
#define save_acc_z            0x44

#define save_5883_x           0x48
#define save_5883_y           0x4c
#define save_5883_z           0x50

#define Shell_Pitch_P         0x60
#define Shell_Pitch_I         0x62
#define Shell_Pitch_D         0x64

#define Shell_Roll_P          0x68
#define Shell_Roll_I          0x6a
#define Shell_Roll_D          0x6c

#define Shell_Yaw_P           0x6e
#define Shell_Yaw_I           0x70
#define Shell_Yaw_D           0x72

#define Core_Pitch_P          0x74
#define Core_Pitch_I          0x76
#define Core_Pitch_D          0x78

#define Core_Roll_P           0x7a
#define Core_Roll_I           0x7c
#define Core_Roll_D           0x7e

#define Core_Yaw_P            0x80
#define Core_Yaw_I            0x82
#define Core_Yaw_D            0x84

#define save_5883_max_x           0x8A
#define save_5883_max_y           0x8B
#define save_5883_max_z           0x8C
#define save_5883_min_x           0x8D
#define save_5883_min_y           0x8E
#define save_5883_min_z           0x8F

#define Hight_P         0x90
#define Hight_I         0x92
#define Hight_D         0x94


u8 AT24CXX_ReadOneByte(u16 ReadAddr);							//指定地址读取一个字节
void AT24CXX_WriteOneByte(u16 WriteAddr,u32 DataToWrite);		//指定地址写入一个字节
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len);//指定地址开始写入指定长度的数据
int16_t AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len);					//指定地址开始读取指定长度数据
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite);	//从指定地址开始写入指定长度的数据
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead);   	//从指定地址开始读出指定长度的数据

u8 AT24CXX_Check(void);  //检查器件
void AT24CXX_Init(void); //初始化IIC

void AT24CXX_Write_save_Byte(u8 addr,int16_t data,u8 num);
u32 AT24CXX_Read_save_Byte(u8 addr);
void AT24cxx_save_Acc_Gyro_offest(void);
void AT24cxx_save_5883_offest(void);
void AT24cxx_save_PID_shell(void);
void AT24cxx_save_PID_core(void);
void AT24cxx_save_PID_hight(void);


void AT24cxx_read_Acc_Gyro_offest(void);
void AT24cxx_read_5883_offest(void);
void AT24cxx_read_PID_shell(void);
void AT24cxx_read_PID_core(void);
void AT24cxx_read_PID_hight(void);
#endif
















