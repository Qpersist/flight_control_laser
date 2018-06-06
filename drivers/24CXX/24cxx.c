#include "include.h" 
#include "i2c_soft.h"	
#include "mpu6050.h"
#include "ctrl.h"


//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//24CXX 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
u8 error_check;
//初始化IIC接口
void AT24CXX_Init(void)
{
	I2c_Soft_Init();//IIC初始化
	if((AT24CXX_Check())==0)
		error_check=1;
}
//在AT24CXX指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
u8  AT24CXX_ReadOneByte(u16 ReadAddr)
{				  
	u8  temp=0;		  	    																 
    I2c_Soft_Start();  
	I2c_Soft_SendByte(0XA0+((ReadAddr/256)<<1));   //发送器件地址0XA0,写数据 	   
	I2c_Soft_WaitAsk(); 
    I2c_Soft_SendByte(ReadAddr%256);   //发送低地址
	I2c_Soft_WaitAsk();	    
	I2c_Soft_Start();  	 	   
	I2c_Soft_SendByte(0XA1);           //进入接收模式			   
	I2c_Soft_WaitAsk();	 
    temp=I2c_Soft_ReadByte(0);	
//		IIC_NAck();
    I2c_Soft_Stop();//产生一个停止条件	    
	return temp;
}
//在AT24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
void AT24CXX_WriteOneByte(u16 WriteAddr,u32 DataToWrite)
{				   	  	    																 
    I2c_Soft_Start();  
	I2c_Soft_SendByte(0XA0+((WriteAddr/256)<<1));   //发送器件地址0XA0,写数据 	 
	I2c_Soft_WaitAsk();	   
    I2c_Soft_SendByte(WriteAddr%256);   //发送低地址
	I2c_Soft_WaitAsk(); 	 										  		   
	I2c_Soft_SendByte(DataToWrite);     //发送字节							   
	I2c_Soft_WaitAsk();  		    	   
    I2c_Soft_Stop();//产生一个停止条件 
	Delay_ms(10);	 
}
//在AT24CXX里面的指定地址开始写入长度为Len的数据
//该函数用于写入16bit或者32bit的数据.
//WriteAddr  :开始写入的地址  
//DataToWrite:数据数组首地址
//Len        :要写入数据的长度2,4
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//在AT24CXX里面的指定地址开始读出长度为Len的数据
//该函数用于读出16bit或者32bit的数据.
//ReadAddr   :开始读出的地址 
//返回值     :数据
//Len        :要读出数据的长度2,4
int16_t AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len)
{  	
	u8 t;
	u32 temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=AT24CXX_ReadOneByte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}
//检查AT24CXX是否正常
//这里用了24XX的最后一个地址(255)来存储标志字.
//如果用其他24C系列,这个地址要修改
//返回1:检测失败
//返回0:检测成功
u8 AT24CXX_Check(void)
{
	u8 temp;
	temp=AT24CXX_ReadOneByte(255);//避免每次开机都写AT24CXX			   
	if(temp==0X55)return 0;		   
	else//排除第一次初始化的情况
	{
		AT24CXX_WriteOneByte(32767,0X55);
	    temp=AT24CXX_ReadOneByte(32767);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//在AT24CXX里面的指定地址开始读出指定个数的数据
//ReadAddr :开始读出的地址 对24c02为0~255
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  
//在AT24CXX里面的指定地址开始写入指定个数的数据
//WriteAddr :开始写入的地址 对24c02为0~255
//pBuffer   :数据数组首地址
//NumToWrite:要写入数据的个数
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}


/******************************数据储存*************************************************/

/************************************************************   
* 函数名:IIC_ADD_write   
* 描述 : 向特定设备id的特定地址，写入字节 
* 输入  :设备id，内部地址，数据    
* 输出  :无    
*/
void IIC_ADD_write(u8 DeviceAddr,u8 address,u8 Bytes)
{
	I2c_Soft_Start();
	I2c_Soft_SendByte(DeviceAddr);
	I2c_Soft_WaitAsk();
	I2c_Soft_SendByte(address);   //发送低地址
	I2c_Soft_WaitAsk(); 	 										  		   
	I2c_Soft_SendByte(Bytes);     //发送字节							   
	I2c_Soft_WaitAsk();  		    	   
    I2c_Soft_Stop();//产生一个停止条件 
	Delay_ms(10);	
}

void IIC_ADD_writelen_BYTE(u16 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		IIC_ADD_write(0xA0,WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}


/************************************************************   
* 函数名:I2C_ReadByte   
* 描述 : 从特定设备id的特定地址读取内容
* 输入  :设备id，内部地址   
* 输出  :读取的内容   
*/
u8 IIC_ADD_read(u8 DeviceAddr,u8 address)
{
	  unsigned char temp;
   I2c_Soft_Start();
   I2c_Soft_SendByte(DeviceAddr);
   I2c_Soft_WaitAsk();
 
    I2c_Soft_SendByte(address);   //发送低地址
	I2c_Soft_WaitAsk();	    
	I2c_Soft_Start();  	 	   
	I2c_Soft_SendByte(DeviceAddr+1);           //进入接收模式			   
	I2c_Soft_WaitAsk();	 
    temp=I2c_Soft_ReadByte(0);		   
    I2c_Soft_Stop();//产生一个停止条件	    
	return temp;
}

/************************************************************   
* 函数名:IIC_Read_MultiBytes   
* 描述 : 从特定设备id的特定地址读取多个字节
* 输入  :设备id，内部地址，需要读的字节个数（0不读取）
* 输出  :读取的内容   
*************************************************************/
int16_t IIC_Read_MultiBytes(u8 DeviceAddr,u8 address,u8 Len)
{
	u8 t;
	u32 temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=IIC_ADD_read(DeviceAddr,address+Len-t-1); 	 				   
	}
	return temp;
}


void AT24cxx_save_Acc_Gyro_offest(void)
{
	
	
	
	IIC_ADD_writelen_BYTE(save_gyro_x,(int)(mpu6050.Gyro_Offset.x) *10,2);
	IIC_ADD_writelen_BYTE(save_gyro_y,(int)(mpu6050.Gyro_Offset.y *10),2);
	IIC_ADD_writelen_BYTE(save_gyro_z,(int)(mpu6050.Gyro_Offset.z) *10,2);
	
	IIC_ADD_writelen_BYTE(save_acc_x,(int)(mpu6050.Acc_Offset.x) *10,2);
	IIC_ADD_writelen_BYTE(save_acc_y,(int)(mpu6050.Acc_Offset.y) *10,2);
	IIC_ADD_writelen_BYTE(save_acc_z,(int)(mpu6050.Acc_Offset.z) ,2);
	
}

void AT24cxx_save_5883_offest(void)
{
	IIC_ADD_writelen_BYTE(save_5883_x,ak8975.Mag_Offset.x,2);
	IIC_ADD_writelen_BYTE(save_5883_y,ak8975.Mag_Offset.y,2);
	IIC_ADD_writelen_BYTE(save_5883_z,ak8975.Mag_Offset.z,2);
	

	
}




void AT24cxx_save_PID_shell(void)
{
	u16 temp_P,temp_I,temp_D;
	
	temp_P= ctrl_1.PID[PIDPITCH].kp *1000;
	temp_I= ctrl_1.PID[PIDPITCH].ki *1000;
	temp_D= ctrl_1.PID[PIDPITCH].kd *1000;
	
	IIC_ADD_writelen_BYTE(Shell_Pitch_P,temp_P,2);
	IIC_ADD_writelen_BYTE(Shell_Pitch_I,temp_I,2);
	IIC_ADD_writelen_BYTE(Shell_Pitch_D,temp_D,2);

	
	temp_P= ctrl_1.PID[PIDROLL].kp *1000;
	temp_I= ctrl_1.PID[PIDROLL].ki *1000;
	temp_D= ctrl_1.PID[PIDROLL].kd *1000;
	
	IIC_ADD_writelen_BYTE(Shell_Roll_P,temp_P,2);
	IIC_ADD_writelen_BYTE(Shell_Roll_I,temp_I,2);
	IIC_ADD_writelen_BYTE(Shell_Roll_D,temp_D,2);
	
	
	temp_P= ctrl_1.PID[PIDYAW].kp *1000;
	temp_I= ctrl_1.PID[PIDYAW].ki *1000;
	temp_D= ctrl_1.PID[PIDYAW].kd *1000;
	
	IIC_ADD_writelen_BYTE(Shell_Yaw_P,temp_P,2);
	IIC_ADD_writelen_BYTE(Shell_Yaw_I,temp_I,2);
	IIC_ADD_writelen_BYTE(Shell_Yaw_D,temp_D,2);
	
	
}



void AT24cxx_save_PID_core(void)
{
	u16 temp_P,temp_I,temp_D;
	
	temp_P= ctrl_2.PID[PIDPITCH].kp *1000;
	temp_I= ctrl_2.PID[PIDPITCH].ki *1000;
	temp_D= ctrl_2.PID[PIDPITCH].kd *1000;
	
	IIC_ADD_writelen_BYTE(Core_Pitch_P,temp_P,2);
	IIC_ADD_writelen_BYTE(Core_Pitch_I,temp_I,2);
	IIC_ADD_writelen_BYTE(Core_Pitch_D,temp_D,2);

	
	temp_P= ctrl_2.PID[PIDROLL].kp *1000;
	temp_I= ctrl_2.PID[PIDROLL].ki *1000;
	temp_D= ctrl_2.PID[PIDROLL].kd *1000;
	
	IIC_ADD_writelen_BYTE(Core_Roll_P,temp_P,2);
	IIC_ADD_writelen_BYTE(Core_Roll_I,temp_I,2);
	IIC_ADD_writelen_BYTE(Core_Roll_D,temp_D,2);
	
	
	temp_P= ctrl_2.PID[PIDYAW].kp *1000;
	temp_I= ctrl_2.PID[PIDYAW].ki *1000;
	temp_D= ctrl_2.PID[PIDYAW].kd *1000;
	
	IIC_ADD_writelen_BYTE(Core_Yaw_P,temp_P,2);
	IIC_ADD_writelen_BYTE(Core_Yaw_I,temp_I,2);
	IIC_ADD_writelen_BYTE(Core_Yaw_D,temp_D,2);

	
}


void AT24cxx_save_PID_hight(void)
{
	u16 temp_P,temp_I,temp_D;
	
	temp_P= pid_setup.groups.hc_height.kp *1000;
	temp_I= pid_setup.groups.hc_height.ki *1000;
	temp_D= pid_setup.groups.hc_height.kd *1000;
	
	IIC_ADD_writelen_BYTE(Hight_P,temp_P,2);
	IIC_ADD_writelen_BYTE(Hight_I,temp_I,2);
	IIC_ADD_writelen_BYTE(Hight_D,temp_D,2);
	
	
}


//*************************************************************************//

u32 AT24CXX_Read_save_Byte(u8 addr)
{
	u32 data;
	u32 data_high,data_low;
	data_high=AT24CXX_ReadOneByte(addr);
	data_low=AT24CXX_ReadOneByte(addr+1);
	data=(data_high<<8)|data_low;
	return data;
}
void AT24cxx_read_Acc_Gyro_offest(void)
{
	


	mpu6050.Gyro_Offset.x=IIC_Read_MultiBytes(0xA0,save_gyro_x,2) /10.0f;
	mpu6050.Gyro_Offset.y=IIC_Read_MultiBytes(0xA0,save_gyro_y,2) /10.0f;
	mpu6050.Gyro_Offset.z=IIC_Read_MultiBytes(0xA0,save_gyro_z,2) /10.0f;
	
	
	mpu6050.Acc_Offset.x=IIC_Read_MultiBytes(0xA0,save_acc_x,2) /10.0f;
	mpu6050.Acc_Offset.y=IIC_Read_MultiBytes(0xA0,save_acc_y,2) /10.0f;
	mpu6050.Acc_Offset.z=IIC_Read_MultiBytes(0xA0,save_acc_z,2);

}

void AT24cxx_read_5883_offest(void)
{
	ak8975.Mag_Offset.x=IIC_Read_MultiBytes(0xA0,save_5883_x,2);
	ak8975.Mag_Offset.y=IIC_Read_MultiBytes(0xA0,save_5883_y,2);
	ak8975.Mag_Offset.z=IIC_Read_MultiBytes(0xA0,save_5883_z,2);


}


void AT24cxx_read_PID_shell(void)
{
		u16 temp_P,temp_I,temp_D;
	
	temp_P= IIC_Read_MultiBytes(0xA0,Shell_Pitch_P,2);
	temp_I= IIC_Read_MultiBytes(0xA0,Shell_Pitch_I,2);
	temp_D= IIC_Read_MultiBytes(0xA0,Shell_Pitch_D,2);
	
	ctrl_1.PID[PIDPITCH].kp= (float)temp_P/1000.0f;
	ctrl_1.PID[PIDPITCH].ki= (float)temp_I/1000.0f;
	ctrl_1.PID[PIDPITCH].kd= (float)temp_D/1000.0f;
	
	
	temp_P= IIC_Read_MultiBytes(0xA0,Shell_Roll_P,2);
	temp_I= IIC_Read_MultiBytes(0xA0,Shell_Roll_I,2);
	temp_D= IIC_Read_MultiBytes(0xA0,Shell_Roll_D,2);
	
	ctrl_1.PID[PIDROLL].kp= (float)temp_P/1000.0f;
	ctrl_1.PID[PIDROLL].ki= (float)temp_I/1000.0f;
	ctrl_1.PID[PIDROLL].kd= (float)temp_D/1000.0f;
	
	
	temp_P= IIC_Read_MultiBytes(0xA0,Shell_Yaw_P,2);
	temp_I= IIC_Read_MultiBytes(0xA0,Shell_Yaw_I,2);
	temp_D= IIC_Read_MultiBytes(0xA0,Shell_Yaw_D,2);
	
	ctrl_1.PID[PIDYAW].kp= (float)temp_P/1000.0f;
	ctrl_1.PID[PIDYAW].ki= (float)temp_I/1000.0f;
	ctrl_1.PID[PIDYAW].kd= (float)temp_D/1000.0f;
	
}




void AT24cxx_read_PID_core(void)
{
		u16 temp_P,temp_I,temp_D;
	
	temp_P= IIC_Read_MultiBytes(0xA0,Core_Pitch_P,2);
	temp_I= IIC_Read_MultiBytes(0xA0,Core_Pitch_I,2);
	temp_D= IIC_Read_MultiBytes(0xA0,Core_Pitch_D,2);
	
	ctrl_2.PID[PIDPITCH].kp= (float)temp_P/1000.0f;
	ctrl_2.PID[PIDPITCH].ki= (float)temp_I/1000.0f;
	ctrl_2.PID[PIDPITCH].kd= (float)temp_D/1000.0f;
	
	
	temp_P= IIC_Read_MultiBytes(0xA0,Core_Roll_P,2);
	temp_I= IIC_Read_MultiBytes(0xA0,Core_Roll_I,2);
	temp_D= IIC_Read_MultiBytes(0xA0,Core_Roll_D,2);
	
	ctrl_2.PID[PIDROLL].kp= (float)temp_P/1000.0f;
	ctrl_2.PID[PIDROLL].ki= (float)temp_I/1000.0f;
	ctrl_2.PID[PIDROLL].kd= (float)temp_D/1000.0f;
	
	
	temp_P= IIC_Read_MultiBytes(0xA0,Core_Yaw_P,2);
	temp_I= IIC_Read_MultiBytes(0xA0,Core_Yaw_I,2);
	temp_D= IIC_Read_MultiBytes(0xA0,Core_Yaw_D,2);
	
	ctrl_2.PID[PIDYAW].kp= (float)temp_P/1000.0f;
	ctrl_2.PID[PIDYAW].ki= (float)temp_I/1000.0f;
	ctrl_2.PID[PIDYAW].kd= (float)temp_D/1000.0f;
	
}

void AT24cxx_read_PID_hight(void)
{
		u16 temp_P,temp_I,temp_D;
	
	temp_P= IIC_Read_MultiBytes(0xA0,Hight_P,2);
	temp_I= IIC_Read_MultiBytes(0xA0,Hight_I,2);
	temp_D= IIC_Read_MultiBytes(0xA0,Hight_D,2);
	
	pid_setup.groups.hc_height.kp= (float)temp_P/1000.0f;
	pid_setup.groups.hc_height.ki= (float)temp_I/1000.0f;
	pid_setup.groups.hc_height.kd= (float)temp_D/1000.0f;
	

	
}



//****************************************************************************************//

