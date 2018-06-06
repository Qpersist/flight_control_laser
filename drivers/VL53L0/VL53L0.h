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

#include "sys.h"

u8 VL53L0X_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);//连续写
u8 VL53L0X_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);//连续读
u8 VL53L0X_Write_Byte(u8 reg,u8 data);//写一个字节
u8 VL53L0X_Read_Byte(u8 reg);//读一个字节
uint16_t bswap(u8 b[]); 				 
uint16_t VL53L0X_decode_vcsel_period(short vcsel_period_reg);	
uint16_t makeuint16(int lsb, int msb);		 


#endif

