#include "include.h"
#include "filter.h"
#include "mymath.h"

#include <math.h>
#include <stdio.h>


// #define WIDTH_NUM 101
// #define FIL_ITEM  10
LPF accelLPF,gyroLPF;
StoredFilterValues accelStoredFilterValues;
u8 imuAccLpfAttFactor = IMU_ACC_IIR_LPF_ATT_FACTOR;

//低通滤波
void anotc_filter_1(float base_hz,float gain_hz,float dT,float in,_filter_1_st *f1)
{
	LPF_1_(gain_hz,dT,(in - f1->out),f1->a); //低通后的变化量

	f1->b = my_pow(in - f1->out);

	f1->e_nr = LIMIT(safe_div(my_pow(f1->a),((f1->b) + my_pow(f1->a)),0),0,1); //变化量的有效率
	
	LPF_1_(base_hz *f1->e_nr,dT,in,f1->out); //低通跟踪
}


//中位值滤波
 void Moving_Average(float moavarray[],u16 len ,u16 *fil_cnt,float in,float *out)
{
	u16 width_num;
	float last;

	width_num = len ;
	
	if( ++*fil_cnt >= width_num )	
	{
		*fil_cnt = 0; //now
	}
	
	last = moavarray[ *fil_cnt ];
	
	moavarray[ *fil_cnt ] = in;
	
	*out += ( in - ( last  ) )/(float)( width_num ) ;
	*out += 0.00001f *(in - *out);  //次要修正
	
}



#define HT_MED_WIDTH_NUM 11
#define HT_MED_FIL_ITEM  4

float HT_med_filter_tmp[HT_MED_FIL_ITEM][HT_MED_WIDTH_NUM ];
float HT_med_filter_out[HT_MED_FIL_ITEM];

u8 HT_med_fil_cnt[HT_MED_FIL_ITEM];

float HT_Moving_Median(u8 item,u8 width_num,float in)
{
	u8 i,j;
	float t;
	float tmp[HT_MED_WIDTH_NUM];
	
	if(item >= HT_MED_FIL_ITEM || width_num >= HT_MED_WIDTH_NUM )
	{
		return 0;
	}
	else
	{
		if( ++HT_med_fil_cnt[item] >= width_num )	
		{
			HT_med_fil_cnt[item] = 0;
		}
		
		HT_med_filter_tmp[item][ HT_med_fil_cnt[item] ] = in;
		
		for(i=0;i<width_num;i++)
		{
			tmp[i] = HT_med_filter_tmp[item][i];
		}
		
		for(i=0;i<width_num-1;i++)
		{
			for(j=0;j<(width_num-1-i);j++)
			{
				if(tmp[j] > tmp[j+1])
				{
					t = tmp[j];
					tmp[j] = tmp[j+1];
					tmp[j+1] = t;
				}
			}
		}		
		return ( tmp[(u16)width_num/2] );
	}
}

s32 Moving_Median(s32 moavarray[],u16 len ,u16 *fil_p,s32 in)
{
	u16 width_num;
	u16 now_p;
	float t;
	s8 pn=0;
	u16 start_p,i;
	s32 sum = 0;

	width_num = len ;
	
	if( ++*fil_p >= width_num )	
	{
		*fil_p = 0; //now
	}
	
	now_p = *fil_p ;	
	
	moavarray[ *fil_p ] = in;
	
	if(now_p<width_num-1) //保证比较不越界
	{
		while(moavarray[now_p] > moavarray[now_p + 1])
		{
			t = moavarray[now_p];
			moavarray[now_p] = moavarray[now_p + 1];
			moavarray[now_p + 1] = t;
			pn = 1;
			now_p ++;
			if(now_p == (width_num-1))
			{
				break;
			}
		}
	}
	
	if(now_p>0)  //保证比较不越界
	{
		while(moavarray[now_p] < moavarray[now_p - 1])
		{
			t = moavarray[now_p];
			moavarray[now_p] = moavarray[now_p - 1];
			moavarray[now_p - 1] = t;
			pn = -1;
			now_p--;
			if(now_p == 0)
			{
				break;
			}
		}
	
	}
	
	if(*fil_p == 0 && pn == 1)
	{
		*fil_p = width_num - 1;
	}
	else if(*fil_p == width_num - 1 && pn == -1)
	{
		*fil_p = 0;
	}
	else
	{
		*fil_p -= pn;
	}
	
	start_p = (u16)(0.25f * width_num );
	for(i = 0; i < width_num/2;i++)
	{
		sum += moavarray[start_p + i];
	}
	return (sum/(width_num/2));
}



void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out) //小范围内正确。
{
	static s8 pn;
	static float h_tmp_x,h_tmp_y;
	
	h_tmp_x = my_sqrt(my_pow(ref->z) + my_pow(ref->y));
	h_tmp_y = my_sqrt(my_pow(ref->z) + my_pow(ref->x));
	
	pn = ref->z < 0? -1 : 1;
	
	  out->x = ( h_tmp_x *in->x - pn *ref->x *in->z ) ;
		out->y = ( pn *h_tmp_y *in->y - ref->y *in->z ) ;
	
// 	 out->x = h_tmp_x *in->x - ref->x *in->z;
// 	 out->y = ref->z *in->y - ref->y *in->z;
	
	out->z = ref->x *in->x + ref->y *in->y + ref->z *in->z ;

}



 /**
 * IIR filter the samples.
IIR滤波
 */
int16_t iirLPFilterSingle(int32_t in, int32_t attenuation,  int32_t* filt)
{
  int32_t inScaled;
  int32_t filttmp = *filt;
  int16_t out;

  if (attenuation > (1<<IIR_SHIFT))
  {
    attenuation = (1<<IIR_SHIFT);
  }
  else if (attenuation < 1)
  {
    attenuation = 1;
  }

  // Shift to keep accuracy
  inScaled = in << IIR_SHIFT;
  // Calculate IIR filter
  filttmp = filttmp + (((inScaled-filttmp) >> IIR_SHIFT) * attenuation);
  // Scale and round
  out = (filttmp >> 8) + ((filttmp & (1 << (IIR_SHIFT - 1))) >> (IIR_SHIFT - 1));
  *filt = filttmp;

  return out;
}
                    
 void imuAccIIRLPFilter(xyz_s16_t* in, LPF* out, StoredFilterValues* storedValues, s32 attenuation)
{
  out->x = iirLPFilterSingle(in->x, attenuation, &storedValues->x);
  out->y = iirLPFilterSingle(in->y, attenuation, &storedValues->y);
  out->z = iirLPFilterSingle(in->z, attenuation, &storedValues->z);
}




/*****************
二阶低通滤波
截止频率请看.h文件
*****************/
/*
#define RATE_500_HZ 500
#define ATTITUDE_RATE RATE_500_HZ
#define ATTITUDE_RATE_LPF_CUTOFF_FREQ         30.0f
#define ATTITUDE_LPF_CUTOFF_FREQ              15.0f
*/

 lpf2pData LPF2filter;
/**
 * 2-Pole low pass filter
 */
void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq)
{
  if (lpfData == (lpf2pData*)NULL || cutoff_freq <= 0.0f) {
    return;
  }

  lpf2pSetCutoffFreq(lpfData, sample_freq, cutoff_freq);
}

void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq)
{
  float fr = sample_freq/cutoff_freq;
  float ohm = tanf(M_PI_F/fr);
  float c = 1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm;
  lpfData->b0 = ohm*ohm/c;
  lpfData->b1 = 2.0f*lpfData->b0;
  lpfData->b2 = lpfData->b0;
  lpfData->a1 = 2.0f*(ohm*ohm-1.0f)/c;
  lpfData->a2 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
  lpfData->delay_element_1 = 0.0f;
  lpfData->delay_element_2 = 0.0f;
}

float lpf2pApply(lpf2pData* lpfData, float sample)//sample is acc
{
  float delay_element_0 = sample - lpfData->delay_element_1 * lpfData->a1 - lpfData->delay_element_2 * lpfData->a2;
  if (!isfinite(delay_element_0)) {
    // don't allow bad values to propigate via the filter
    delay_element_0 = sample;
  }

  float output = delay_element_0 * lpfData->b0 + lpfData->delay_element_1 * lpfData->b1 + lpfData->delay_element_2 * lpfData->b2;

  lpfData->delay_element_2 = lpfData->delay_element_1;
  lpfData->delay_element_1 = delay_element_0;
  return output;
}

float lpf2pReset(lpf2pData* lpfData, float sample)
{
  float dval = sample / (lpfData->b0 + lpfData->b1 + lpfData->b2);
  lpfData->delay_element_1 = dval;
  lpfData->delay_element_2 = dval;
  return lpf2pApply(lpfData, sample);
}



