#ifndef _VL53L0_H
#define _VL53L0_H

#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14
#define VL53L0X_Add 0x29

#include "stm32f4xx.h"


typedef struct
{
	u8 data_buff[12];
	float distance;
	u8 status;
	
	float height;//cm
	
	float  relative_height;
	float old_distance;
	float h_delta;
	float h_dt;
	
	u8 measure_ok;
	u8 measure_ot_cnt;
	
	float ACC_z_zero;
	u8 start_ok;

	u8 warning_flag;
	int warning_count;
//	float h_delta;
}GY530;

extern GY530 GY_530 ;

void GY_530_data(u8 *buff,float *GY_530_data,u8 *GY_530_status);
void GY_530_Duty(void);
uint16_t bswap(u8 b[]); 				 
uint16_t VL53L0X_decode_vcsel_period(short vcsel_period_reg);	
uint16_t makeuint16(int lsb, int msb);		 
void GY_530_hight_ctrl(float T);
void Hight_Duty(void);




#endif

