#ifndef __SYS_DATA_H
#define __SYS_DATA_H

#include "sys_bits.h"
#include "sys_types.h"
#include "sys_defs.h"
#include "tlsf.h"
#include "timestamp.h"

#define	FILE_NAME_LEN		50
#define	LOG_FILE_NAME		"Pcr16Log.txt"//"PerfusionLog.txt"

#define SysState_None							0
#define SysState_Standby					DEF_BIT00_MASK
#define SysState_DeleteLabTB			DEF_BIT01_MASK
#define SysState_RunningTB				DEF_BIT04_MASK

//�豸����ģʽ
enum	{
	DevState_IDLE,//����ģʽ
	DevState_Running,
	DevState_Error,//�豸�й���
	DevState_Test,
};

typedef struct _syserror	{
	BIT8 Y1;
	BIT8 Y2;
	//BIT8 Y3;
}_syserror_t;

typedef struct _sys	{
	volatile u32 state;
	volatile u8 devstate;//�豸����״̬
	volatile u8 devsubstate;//�豸������״̬
	volatile u8 AlarmType;//��������
	u8 StandbyFlag;
}_sys_t;

//typedef struct _sys_data	{
//	s8 HeatCoverTemp;//�ȸ��¶�
//	s8 Temp1;//
//	s8 Temp2;//
//	s16 PD_1;//PD������1
//	s16 PD_2;//PD������2
//}_sys_data_t;
#define	STAGE_MAX		10
#define	STEP_MAX		3
typedef struct _lab_step	{
	s16 temp;
	u16 tim;
}_lab_step_t;

typedef struct _lab_stage	{
	u8 id;
	u8 repeat;
	_lab_step_t step[STEP_MAX];
}_lab_stage_t;

typedef struct _lab_data	{
	u8 StageNum;
	u8 HeatCoverEnable;
	s16 HeatCoverTemp;
	_lab_stage_t stage[STAGE_MAX];
}_lab_data_t;

extern _sys_t Sys;
//extern _sys_data_t SysData;
extern RTC_TIME_ST SysTime;
extern tlsf_t UserMem;
extern _syserror_t SysError;;

void SysDataInit(void);

#endif
