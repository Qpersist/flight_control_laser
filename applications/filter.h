#ifndef __FILTER_H
#define __FILTER_H

#include "parameter.h"

#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))

#define LPF_1(A,B,C,D) LPF_1_((A),(B),(C),*(D));
#define IMU_UPDATE_FREQ   500 
#define IIR_SHIFT         8

/**
 * Set ACC_WANTED_LPF1_CUTOFF_HZ to the wanted cut-off freq in Hz.
 * The highest cut-off freq that will have any affect is fs /(2*pi).
 * E.g. fs = 350 Hz -> highest cut-off = 350/(2*pi) = 55.7 Hz -> 55 Hz
 */
#define IMU_ACC_WANTED_LPF_CUTOFF_HZ 4

/**
 * Attenuation should be between 1 to 256.
 *
 * f0 = fs / 2*pi*attenuation ->
 * attenuation = fs / 2*pi*f0
 */
#define IMU_ACC_IIR_LPF_ATTENUATION (IMU_UPDATE_FREQ / (2 * 3.1415 * IMU_ACC_WANTED_LPF_CUTOFF_HZ))
#define IMU_ACC_IIR_LPF_ATT_FACTOR  (int)(((1<<IIR_SHIFT) / IMU_ACC_IIR_LPF_ATTENUATION) + 0.5)

#define M_PI    3.1415926
#define M_PI_F ((float) M_PI)
	
typedef struct 
{
	s16 x;
	s16 y;
	s16 z;
} LPF;

typedef  struct
{
	s32 x;
	s32 y;
	s32 z;
} StoredFilterValues;


typedef struct
{
	float a;
	float b;
	float e_nr;
	float out;
} _filter_1_st;

typedef struct {
  float a1;
  float a2;
  float b0;
  float b1;
  float b2;
  float delay_element_1;
  float delay_element_2;
} lpf2pData;

void anotc_filter_1(float base_hz,float gain_hz,float dT,float in,_filter_1_st *f1);

void Moving_Average(float moavarray[],u16 len ,u16 *fil_cnt,float in,float *out);
s32 Moving_Median(s32 moavarray[],u16 len ,u16 *fil_p,s32 in);
#define _xyz_f_t xyz_f_t
void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out);


extern LPF accelLPF,gyroLPF;
extern StoredFilterValues accelStoredFilterValues;
extern u8 imuAccLpfAttFactor;
 void imuAccIIRLPFilter(xyz_s16_t* in, LPF* out, StoredFilterValues* storedValues, s32 attenuation);

extern  lpf2pData LPF2filter;
void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
float lpf2pApply(lpf2pData* lpfData, float sample);
float lpf2pReset(lpf2pData* lpfData, float sample);
	
float HT_Moving_Median(u8 item,u8 width_num,float in);


#endif
