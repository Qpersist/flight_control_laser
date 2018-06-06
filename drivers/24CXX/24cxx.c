#include "include.h" 
#include "i2c_soft.h"	
#include "mpu6050.h"
#include "ctrl.h"


//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//24CXX ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
u8 error_check;
//��ʼ��IIC�ӿ�
void AT24CXX_Init(void)
{
	I2c_Soft_Init();//IIC��ʼ��
	if((AT24CXX_Check())==0)
		error_check=1;
}
//��AT24CXXָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
u8  AT24CXX_ReadOneByte(u16 ReadAddr)
{				  
	u8  temp=0;		  	    																 
    I2c_Soft_Start();  
	I2c_Soft_SendByte(0XA0+((ReadAddr/256)<<1));   //����������ַ0XA0,д���� 	   
	I2c_Soft_WaitAsk(); 
    I2c_Soft_SendByte(ReadAddr%256);   //���͵͵�ַ
	I2c_Soft_WaitAsk();	    
	I2c_Soft_Start();  	 	   
	I2c_Soft_SendByte(0XA1);           //�������ģʽ			   
	I2c_Soft_WaitAsk();	 
    temp=I2c_Soft_ReadByte(0);	
//		IIC_NAck();
    I2c_Soft_Stop();//����һ��ֹͣ����	    
	return temp;
}
//��AT24CXXָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
void AT24CXX_WriteOneByte(u16 WriteAddr,u32 DataToWrite)
{				   	  	    																 
    I2c_Soft_Start();  
	I2c_Soft_SendByte(0XA0+((WriteAddr/256)<<1));   //����������ַ0XA0,д���� 	 
	I2c_Soft_WaitAsk();	   
    I2c_Soft_SendByte(WriteAddr%256);   //���͵͵�ַ
	I2c_Soft_WaitAsk(); 	 										  		   
	I2c_Soft_SendByte(DataToWrite);     //�����ֽ�							   
	I2c_Soft_WaitAsk();  		    	   
    I2c_Soft_Stop();//����һ��ֹͣ���� 
	Delay_ms(10);	 
}
//��AT24CXX�����ָ����ַ��ʼд�볤��ΪLen������
//�ú�������д��16bit����32bit������.
//WriteAddr  :��ʼд��ĵ�ַ  
//DataToWrite:���������׵�ַ
//Len        :Ҫд�����ݵĳ���2,4
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//��AT24CXX�����ָ����ַ��ʼ��������ΪLen������
//�ú������ڶ���16bit����32bit������.
//ReadAddr   :��ʼ�����ĵ�ַ 
//����ֵ     :����
//Len        :Ҫ�������ݵĳ���2,4
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
//���AT24CXX�Ƿ�����
//��������24XX�����һ����ַ(255)���洢��־��.
//���������24Cϵ��,�����ַҪ�޸�
//����1:���ʧ��
//����0:���ɹ�
u8 AT24CXX_Check(void)
{
	u8 temp;
	temp=AT24CXX_ReadOneByte(255);//����ÿ�ο�����дAT24CXX			   
	if(temp==0X55)return 0;		   
	else//�ų���һ�γ�ʼ�������
	{
		AT24CXX_WriteOneByte(32767,0X55);
	    temp=AT24CXX_ReadOneByte(32767);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//��AT24CXX�����ָ����ַ��ʼ����ָ������������
//ReadAddr :��ʼ�����ĵ�ַ ��24c02Ϊ0~255
//pBuffer  :���������׵�ַ
//NumToRead:Ҫ�������ݵĸ���
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  
//��AT24CXX�����ָ����ַ��ʼд��ָ������������
//WriteAddr :��ʼд��ĵ�ַ ��24c02Ϊ0~255
//pBuffer   :���������׵�ַ
//NumToWrite:Ҫд�����ݵĸ���
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}


/******************************���ݴ���*************************************************/

/************************************************************   
* ������:IIC_ADD_write   
* ���� : ���ض��豸id���ض���ַ��д���ֽ� 
* ����  :�豸id���ڲ���ַ������    
* ���  :��    
*/
void IIC_ADD_write(u8 DeviceAddr,u8 address,u8 Bytes)
{
	I2c_Soft_Start();
	I2c_Soft_SendByte(DeviceAddr);
	I2c_Soft_WaitAsk();
	I2c_Soft_SendByte(address);   //���͵͵�ַ
	I2c_Soft_WaitAsk(); 	 										  		   
	I2c_Soft_SendByte(Bytes);     //�����ֽ�							   
	I2c_Soft_WaitAsk();  		    	   
    I2c_Soft_Stop();//����һ��ֹͣ���� 
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
* ������:I2C_ReadByte   
* ���� : ���ض��豸id���ض���ַ��ȡ����
* ����  :�豸id���ڲ���ַ   
* ���  :��ȡ������   
*/
u8 IIC_ADD_read(u8 DeviceAddr,u8 address)
{
	  unsigned char temp;
   I2c_Soft_Start();
   I2c_Soft_SendByte(DeviceAddr);
   I2c_Soft_WaitAsk();
 
    I2c_Soft_SendByte(address);   //���͵͵�ַ
	I2c_Soft_WaitAsk();	    
	I2c_Soft_Start();  	 	   
	I2c_Soft_SendByte(DeviceAddr+1);           //�������ģʽ			   
	I2c_Soft_WaitAsk();	 
    temp=I2c_Soft_ReadByte(0);		   
    I2c_Soft_Stop();//����һ��ֹͣ����	    
	return temp;
}

/************************************************************   
* ������:IIC_Read_MultiBytes   
* ���� : ���ض��豸id���ض���ַ��ȡ����ֽ�
* ����  :�豸id���ڲ���ַ����Ҫ�����ֽڸ�����0����ȡ��
* ���  :��ȡ������   
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

