#ifndef __PD_DataProcess_H__
#define __PD_DataProcess_H__

#include "includes.h"

typedef struct _pd_maxmin	{//PD���ֵ ��Сֵ ��ֵ
	u8 idx;
	u16 min[2];
	u16 max[2];
	u16 aver;
}_pd_maxmin_t;

typedef struct _pd_data	{//PD���ݽṹ��
	u8 coll_enable;//�Ƿ�ɼ���־
	u8 ch;
	u16 HoleThreshold;//��Ч��λ�õ�PD��ֵ
	_pd_maxmin_t templatehole;
}_pd_data_t;

extern _pd_data_t gPD_Data;

void PD_DataInit(void);
void StartCollTemplateHolePD(void);
void StopCollTemplateHolePD(void);
void CalcTemplateHolePDAver(void);
void PD_DataCollect(u16 ad_vol, u8 pd_ch);

#endif

