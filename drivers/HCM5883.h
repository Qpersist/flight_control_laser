#ifndef _HCC5883_H_
#define	_HCC5883_H_

#include "stm32f4xx.h"
#include "stdbool.h"
#include "include.h"

#define CALIBRATING_MAG_CYCLES              2000  //??????20s

#define HMC58X3_ADDR 0x3C // 7 bit address of the HMC58X3 used with the Wire library
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2
// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_R_XM 3
#define HMC58X3_R_XL 4

#define HMC5883L_Output_X_MSB            0x03
#define HMC5883L_Output_X_LSB 					 0x04
#define HMC5883L_Output_Z_MSB            0x05
#define HMC5883L_Output_Z_LSB 					 0x06
#define HMC5883L_Output_Y_MSB            0x07
#define HMC5883L_Output_Y_LSB 					 0x08
#define HMC5883L_ID_A										 0x0A
#define HMC5883L_ID_B 									 0x0B
#define HMC5883L_ID_C 									 0x0C
typedef struct 
{
	xyz_s16_t Mag_Adc;			//???
	xyz_f_t Mag_Offset;		//???
	xyz_f_t 	Mag_Gain;			//????	
  xyz_f_t 	Mag_Val;			//?????
}HMC5883_t;

extern HMC5883_t HMC5883;

//bool ANO_HMC5883_Run(void);
//void ANO_HMC5883_CalOffset_Mag(void);
//void ANO_HMC5883_Read(void);

void HMC58X3_writeReg(unsigned char reg, unsigned char val);
void HMC5883L_SetUp(void);	//≥ı ºªØ
void HMC58X3_setMode(unsigned char mode);
void HMC58X3_getRaw(void);
void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z);
void ANO_HMC5883_CalOffset_Mag(void);
extern u8 Mag_CALIBRATED;
extern u8 HMC5883_ok;

#endif

