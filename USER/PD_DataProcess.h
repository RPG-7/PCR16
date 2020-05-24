#ifndef __PD_DataProcess_H__
#define __PD_DataProcess_H__

#include "includes.h"


//typedef struct _pd_maxmin	{//PD���ֵ ��Сֵ ��ֵ
//	u8 idx;
//	u16 min[2];
//	u16 max[2];
//	u16 aver;
//}_pd_maxmin_t;

typedef struct _pd_data	{//PD���ݽṹ��
	volatile u8 coll_enable;//�Ƿ�ɼ���־
//	volatile u8 DataValid;
	volatile u8 ch;
//	u16 HoleThreshold;//��Ч��λ�õ�PD��ֵ
	u16 PDVol[HOLE_NUM];//16�׵�PD��Ч�ź� mv
	u16 PDBaseBlue[HOLE_NUM];//16�ױ����ź�mv
	u16 PDBaseGreen[HOLE_NUM];
}_pd_data_t;

extern _pd_data_t gPD_Data;
//extern _pd_maxmin_t templatehole;

void PD_DataInit(void);
void StartCaliHolePDBase(void);
void StopCaliHolePDBase(void);
void CalcHolePDBase(u8 flag);
void PD_DataCollect(u16 ad_vol, u8 pd_ch);
void StartCaliHolePosition(void);
void StopCaliHolePosition(void);
void CalcHolePositon(void);
void ExchangePDBuf(void);
void StartCollPDData(void);
void StopCollPDData(void);
void CalcPDData(u8 hole_idx);
#endif

