
#include "HCM5883.h"
#include "parameter.h"
#include "mymath.h"
#include "include.h"
#include "i2c_soft.h"
#include "ak8975.h"

u8 Mag_CALIBRATED;
#define  HMC5883_Buf_Size 10
// 	xyz_f_t Mag_Offset = { -1 , -1 , -1 };
// 	xyz_f_t Mag_Gain   = { 1 , 0.8538 , 0.9389 };

HMC5883_t HMC5883 = { {0,0,0},{-1,-1,-1},{1,0.8538,0.9389},{0,0,0} };
int16_t  HMC5883_FIFO[3][11]; //?????
int8_t  HMC5883_Buf_index = 0;
int16_t  HMC5883_maxx=0,HMC5883_maxy=0,HMC5883_maxz=0,
		 HMC5883_minx=-0,HMC5883_miny=-0,HMC5883_minz=-0;
unsigned char HMC5883_calib=0; //???????
int16_t HMC5883_Offset_X = 0,
				HMC5883_Offset_Y = 0,
				HMC5883_Offset_Z = 0;

float  HMC5883_Scale_X = 1.0f,
			 HMC5883_Scale_Y = 1.0f,
			 HMC5883_Scale_Z = 1.0f;
//bool ANO_HMC5883_Run(void)
//{
//	return IIC_Write_1Byte(HMC5883_ADDRESS,HMC5883_CNTL,0x01);	
//}

xyz_f_t XYZ_STRUCT_COPY(float x,float y, float z)
{
	xyz_f_t m ;
	m.x = x;
	m.y = y;
	m.z = z;
	return m;
}
u8 HMC5883_ok;
//void ANO_HMC5883_Read_Mag_Data(void)
//{
//	int16_t mag_temp[3];
//	u8 HMC5883_buffer[6]; //??????
//	
//	I2C_FastMode = 0;
//	
//	IIC_Read_1Byte(HMC5883_ADDRESS,HMC5883_HXL,&HMC5883_buffer[0]); 
//	IIC_Read_1Byte(HMC5883_ADDRESS,HMC5883_HXH,&HMC5883_buffer[1]);
//	mag_temp[1] = ((((int16_t)HMC5883_buffer[1]) << 8) | HMC5883_buffer[0]) ;  //???X?

//	IIC_Read_1Byte(HMC5883_ADDRESS,HMC5883_HYL,&HMC5883_buffer[2]);
//	IIC_Read_1Byte(HMC5883_ADDRESS,HMC5883_HYH,&HMC5883_buffer[3]);
//	mag_temp[0] = ((((int16_t)HMC5883_buffer[3]) << 8) | HMC5883_buffer[2]) ;  //???Y?

//	IIC_Read_1Byte(HMC5883_ADDRESS,HMC5883_HZL,&HMC5883_buffer[4]);
//	IIC_Read_1Byte(HMC5883_ADDRESS,HMC5883_HZH,&HMC5883_buffer[5]);
//	mag_temp[2] = -((((int16_t)HMC5883_buffer[5]) << 8) | HMC5883_buffer[4]) ;  //???Z?	
//	
//	HMC5883.Mag_Adc.x = mag_temp[0];
//	HMC5883.Mag_Adc.y = mag_temp[1];
//	HMC5883.Mag_Adc.z = mag_temp[2];
//	
//	HMC5883.Mag_Val.x = (HMC5883.Mag_Adc.x - HMC5883.Mag_Offset.x) ;
//	HMC5883.Mag_Val.y = (HMC5883.Mag_Adc.y - HMC5883.Mag_Offset.y) ;
//	HMC5883.Mag_Val.z = (HMC5883.Mag_Adc.z - HMC5883.Mag_Offset.z) ;
//	//???????	
//	ANO_HMC5883_CalOffset_Mag();
//	
//	//HMC5883????
//	ANO_HMC5883_Run();
//}
void HMC58X3_init(u8 setmode) {

	u8 tempA ,tempB ,tempC;
	
//	HCM_IIC_Write_1Byte(AK8975_ADDRESS,0x00,0xf0);	
//	HCM_IIC_Write_1Byte(AK8975_ADDRESS,0x01,0x20);	
	 HCM_IIC_Write_1Byte(AK8975_ADDRESS,0x02,0x01);

	
//	HCM_IIC_Read_1Byte(AK8975_ADDRESS,0x0A,&tempA);
//	HCM_IIC_Read_1Byte(AK8975_ADDRESS,0x0B,&tempB);
//	HCM_IIC_Read_1Byte(AK8975_ADDRESS,0x0C,&tempC);
//	return tempA + tempB + tempC;

}
void HMC58X3_setMode(unsigned char mode) {
  if (mode > 2) {
    return;
  }
  HMC58X3_writeReg(HMC58X3_R_MODE, mode);
 Delay_us(100);
}
void HMC58X3_writeReg(unsigned char reg, unsigned char val) {
   IIC_Read_1Byte(HMC58X3_ADDR,reg,&val);
}
void HMC58X3_setDOR(unsigned char DOR) {
  if (DOR>6) return;
  HMC58X3_writeReg(HMC58X3_R_CONFA,DOR<<2);
}
void HMC58X3_FIFO_init(void)
{
//  int16_t temp[3];
  unsigned char i;
  for(i=0;i<50;i++){
  HMC58X3_getRaw();
  Delay_us(200);  //ÑÓÊ±ÔÙ¶ÁÈ¡Êý¾Ý
  //LED_Change(); //LEDÉÁË¸
  }
}
void HMC5883L_SetUp(void)
{ 
//u8 BUF1[7]={0};	
//	u8 ID_A,ID_B,ID_C;
//	IIC_Read_1Byte(HMC58X3_ADDR,HMC5883L_ID_A,&ID_A);
//	IIC_Read_1Byte(HMC58X3_ADDR,HMC5883L_ID_B,&ID_A);
//	IIC_Read_1Byte(HMC58X3_ADDR,HMC5883L_ID_C,&ID_C);
//	if(ID_A=='H'&&ID_B=='4'&&ID_C=='3')
//	{
//		printf("\r??????\r\n\r");
//	}else
//	{
//		printf("\r??????!\r\n\r");
//	}
//	IIC_Write_1Byte(0xD0,0x37,0x42);//turn on Bypass Mode 
//	IIC_Write_1Byte(0xD0,0x6A,0x40);//close Master Mode	
//  HMC58X3_init(0); // Don't set mode yet, we'll do that later on.
//  HMC58X3_setMode(2);
//  HMC58X3_setDOR(6);  //75hz ¸
//  HMC58X3_FIFO_init();


	IIC_Write_1Byte(HMC58X3_ADDR,HMC58X3_R_CONFA,0x70);   //???75hz
//	hmcIIC_Write_1Byte(HMC58X3_ADDR,HMC58X3_R_CONFB,0xA0);   //????(-4.7Ga,+4.7Ga)
	IIC_Write_1Byte(HMC58X3_ADDR,HMC58X3_R_MODE,0x00);   //??????
	
	Delay_ms(5);
//	IIC_Read_nByte(0x3D,HMC5883L_Output_X_MSB,6,BUF1);
//	hmcIIC_Read_1Byte(0x3c,HMC5883L_Output_X_MSB,&BUF1[1]);//OUT_X_H
//	hmcIIC_Read_1Byte(0x3c,HMC5883L_Output_X_LSB,&BUF1[2]);//OUT_X_L

//	hmcIIC_Read_1Byte(0x3c,HMC5883L_Output_Y_MSB,&BUF1[3]);//OUT_Y_L_A
//	hmcIIC_Read_1Byte(0x3c,HMC5883L_Output_Y_LSB,&BUF1[4]);//OUT_Y_H_A

//  hmcIIC_Read_1Byte(0x3c,HMC5883L_Output_Z_MSB,&BUF1[5]);//OUT_Z_L_A
//	hmcIIC_Read_1Byte(0x3c,HMC5883L_Output_Z_LSB,&BUF1[6]);//OUT_Z_H_A
//	
//	HMC5883.Mag_Val.x=(float)((BUF1[1] << 8) | BUF1[2]); //Combine MSB and LSB of X Data output register
//	HMC5883.Mag_Val.y=(float)((BUF1[3] << 8) | BUF1[4]); //Combine MSB and LSB of Z Data output register
//  HMC5883.Mag_Val.z=(float)((BUF1[5] << 8) | BUF1[6]); //Combine MSB and LSB of Z Data output register
	
	
//  AT45DB_Read_config(); //¶ÁÈ¡ÅäÖÃ  {µ÷ÓÃAT45DB.cµÄ×Ó³ÌÐò} 
//  HMC5883_Offset_X = AT24CXX_ReadLenByte(AT_HMC5883_Offset_X,2);;  //È¡´ÅÁ¦¼ÆµÄ±ê¶¨ÖµÀ´ÓÃ
//  HMC5883_Offset_Y = AT24CXX_ReadLenByte(AT_HMC5883_Offset_Y,2); ; 
//  HMC5883_Offset_Z = AT24CXX_ReadLenByte(AT_HMC5883_Offset_Z,2); ; 

//  HMC5883_Scale_X = Config.Magnetic_Scale_X ; 
//  HMC5883_Scale_Y = Config.Magnetic_Scale_Y ;  
//  HMC5883_Scale_Z = Config.Magnetic_Scale_Z ; 
}

void HMC58X3_getRaw() {
	u8 BUF1[7]={0};
	int16_t mag_temp[3];
//	IIC_Write_1Byte(0x3c,HMC58X3_R_CONFA,0x14);   //30HZ,????
//	I2C_WriteByte(HMC5883L_Addr,HMC5883L_ConfigurationRegisterB,0x20);   //ÅäÖÃ¼Ä´æÆ÷B£ºÔöÒæ¿ØÖÆ
//	IIC_Write_1Byte(0x3c,HMC58X3_R_MODE,0x01);   //????
	
	Delay_ms(5);
//	IIC_Read_nByte(0x3d,HMC5883L_Output_X_MSB,6,BUF1);
//	IIC_Read_1Byte(0x3c,HMC5883L_Output_X_MSB,&BUF1[1]);//OUT_X_H
//	IIC_Read_1Byte(0x3c,HMC5883L_Output_X_LSB,&BUF1[2]);//OUT_X_L

//	IIC_Read_1Byte(0x3c,HMC5883L_Output_Y_MSB,&BUF1[3]);//OUT_Y_L_A
//	IIC_Read_1Byte(0x3c,HMC5883L_Output_Y_LSB,&BUF1[4]);//OUT_Y_H_A
//	
//  IIC_Read_1Byte(0x3c,HMC5883L_Output_Z_MSB,&BUF1[5]);//OUT_Z_L_A
//	IIC_Read_1Byte(0x3c,HMC5883L_Output_Z_LSB,&BUF1[6]);//OUT_Z_H_A
	
	IIC_Read_1Byte(0x3c,HMC5883L_Output_X_MSB,&BUF1[1]);//OUT_X_H
	IIC_Read_1Byte(0x3c,HMC5883L_Output_X_LSB,&BUF1[2]);//OUT_X_L
	mag_temp[0] = ((((int16_t)BUF1[1]) << 8) | BUF1[2]) ;  //???X?
	
	IIC_Read_1Byte(0x3c,HMC5883L_Output_Y_MSB,&BUF1[3]);//OUT_Y_L_A
	IIC_Read_1Byte(0x3c,HMC5883L_Output_Y_LSB,&BUF1[4]);//OUT_Y_H_A
	mag_temp[1] = ((((int16_t)BUF1[3]) << 8) | BUF1[4]) ;  //???Y?
	
  IIC_Read_1Byte(0x3c,HMC5883L_Output_Z_MSB,&BUF1[5]);//OUT_Z_L_A
	IIC_Read_1Byte(0x3c,HMC5883L_Output_Z_LSB,&BUF1[6]);//OUT_Z_H_A
	mag_temp[2] = ((((int16_t)BUF1[5]) << 8) | BUF1[6]) ;  //???Z?	
	 
	HMC58X3_newValues(mag_temp[0],mag_temp[1],mag_temp[2]);
	HMC5883.Mag_Adc.x = HMC5883_FIFO[0][10];
	HMC5883.Mag_Adc.y = HMC5883_FIFO[1][10];
	HMC5883.Mag_Adc.z = HMC5883_FIFO[2][10];
//	if(HMC5883.Mag_Val.x>0x7fff) HMC5883.Mag_Val.x-=0xffff;	  
//  if(HMC5883.Mag_Val.y>0x7fff) HMC5883.Mag_Val.y-=0xffff;
//  if(HMC5883.Mag_Val.z>0x7fff) HMC5883.Mag_Val.z-=0xffff;	
//	HMC5883.Mag_Val.x=((float)((BUF1[1] << 8) | BUF1[2])); //Combine MSB and LSB of X Data output register
//	HMC5883.Mag_Val.y=((float)((BUF1[3] << 8) | BUF1[4])); //Combine MSB and LSB of Z Data output register
//  HMC5883.Mag_Val.z=((float)((BUF1[5] << 8) | BUF1[6])); //Combine MSB and LSB of Z Data output register
	HMC5883.Mag_Val.x = (HMC5883.Mag_Adc.x - HMC5883.Mag_Offset.x) ;
	HMC5883.Mag_Val.y = (HMC5883.Mag_Adc.y - HMC5883.Mag_Offset.y) ;
	HMC5883.Mag_Val.z = (HMC5883.Mag_Adc.z - HMC5883.Mag_Offset.z) ;
	ANO_HMC5883_CalOffset_Mag();
}
void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
{
	unsigned char i ;
	int32_t sum=0;

	HMC5883_FIFO[0][HMC5883_Buf_index] = x;
	HMC5883_FIFO[1][HMC5883_Buf_index] = y;
	HMC5883_FIFO[2][HMC5883_Buf_index] = z;
	HMC5883_Buf_index = (HMC5883_Buf_index + 1) % HMC5883_Buf_Size;

	sum=0;
	for(i=0;i<10;i++){	//È¡Êý×éÄÚµÄÖµ½øÐÐÇóºÍÔÙÈ¡Æ½¾ù
   		sum+=HMC5883_FIFO[0][i];
	}
	HMC5883_FIFO[0][10]=sum/10;	//½«Æ½¾ùÖµ¸üÐÂ

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5883_FIFO[1][i];
	}
	HMC5883_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5883_FIFO[2][i];
	}
	HMC5883_FIFO[2][10]=sum/10;

	if(HMC5883_calib){//Ð£ÕýÓÐÐ§µÄ»° ²É¼¯±ê¶¨Öµ
		if(HMC5883_minx>HMC5883_FIFO[0][10])HMC5883_minx=(int16_t)HMC5883_FIFO[0][10];
		if(HMC5883_miny>HMC5883_FIFO[1][10])HMC5883_miny=(int16_t)HMC5883_FIFO[1][10];
		if(HMC5883_minz>HMC5883_FIFO[2][10])HMC5883_minz=(int16_t)HMC5883_FIFO[2][10];

		if(HMC5883_maxx<HMC5883_FIFO[0][10])HMC5883_maxx=(int16_t)HMC5883_FIFO[0][10];
		if(HMC5883_maxy<HMC5883_FIFO[1][10])HMC5883_maxy=(int16_t)HMC5883_FIFO[1][10];
		if(HMC5883_maxz<HMC5883_FIFO[2][10])HMC5883_maxz=(int16_t)HMC5883_FIFO[2][10];
//		LED_Set_Blink(Blue,30,50,1);  //Ö¸Ê¾ ÕýÔÚ±ê¶¨
	}

} //HMC58X3_newValues
//xyz_f_t ANO_HMC5883_Get_Mag(void)
//{
//	return HMC5883.Mag_Val;
//}

u8 Mag_CALIBRATED = 0;
//???????

void ANO_HMC5883_CalOffset_Mag(void)
{
	static xyz_f_t	MagMAX = { -100 , -100 , -100 }, MagMIN = { 100 , 100 , 100 }, MagSum;
	static uint16_t cnt_m=0;
	
	if(Mag_CALIBRATED)
	{	
		
//		if(ABS(HMC5883.Mag_Adc.x)<400&&ABS(HMC5883.Mag_Adc.y)<400&&ABS(HMC5883.Mag_Adc.z)<400)
//		{
			MagMAX.x = MAX(HMC5883.Mag_Adc.x, MagMAX.x);
			MagMAX.y = MAX(HMC5883.Mag_Adc.y, MagMAX.y);
			MagMAX.z = MAX(HMC5883.Mag_Adc.z, MagMAX.z);
			
			MagMIN.x = MIN(HMC5883.Mag_Adc.x, MagMIN.x);
			MagMIN.y = MIN(HMC5883.Mag_Adc.y, MagMIN.y);
			MagMIN.z = MIN(HMC5883.Mag_Adc.z, MagMIN.z);		
			
			if(cnt_m == CALIBRATING_MAG_CYCLES)
			{
				HMC5883.Mag_Offset.x = (int16_t)((MagMAX.x + MagMIN.x) * 0.5f);
				HMC5883.Mag_Offset.y = (int16_t)((MagMAX.y + MagMIN.y) * 0.5f);
				HMC5883.Mag_Offset.z = (int16_t)((MagMAX.z + MagMIN.z) * 0.5f);
	        
				MagSum.x = MagMAX.x - MagMIN.x;
				MagSum.y = MagMAX.y - MagMIN.y;
				MagSum.z = MagMAX.z - MagMIN.z;
				
				HMC5883.Mag_Gain.y = MagSum.x / MagSum.y;
				HMC5883.Mag_Gain.z = MagSum.x / MagSum.z;
				
				Param_SaveMagOffset(&HMC5883.Mag_Offset);//param_Save();//????
				cnt_m = 0;
				Mag_CALIBRATED = 0;
			}
//		}
		cnt_m++;
		
	}
	else
	{

	}
}

//void ANO_HMC5883_Read(void)
//{
//		//?????
//		ANO_HMC5883_Read_Mag_Data();
//}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

