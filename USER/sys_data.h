#ifndef __SYS_DATA_H
#define __SYS_DATA_H

#include "sys_bits.h"
#include "sys_types.h"
#include "sys_defs.h"
#include "tlsf.h"
#include "timestamp.h"

#define	FILE_NAME_LEN		50
#define	LOG_FILE_NAME		"Pcr16Log.txt"//"PerfusionLog.txt"

#define	HEATCOVER_TEMPMAX		110
#define	HEATCOVER_TEMPMIN		30

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
#define	STAGE_MAX		9
#define	STEP_MAX		9
#define	LAB_ID_LEN		22
#define	LAB_NAME_LEN		15
#define	HOLE_NUM		16
typedef struct _step	{
	s16 temp;
	u16 tim;
}_step_t;

typedef struct _stage	{
	u8 CollEnable;//�ɼ�ʹ��
	u8 Type;//0-repeatģʽ;1-continue ģʽ;2-step ģʽ
	u16 T_Rate;//��������
	u16 T_Inter;//�¶ȼ��
	u16 T_Tim;//����ʱ�� s
	u8 Repeat;//
	u8 StepNum;
	_step_t step[STEP_MAX];
}_stage_t;

typedef struct _temp_data	{//�¶���Ϣ
	u8 StageNum;
	u8 HeatCoverEnable;
	s16 HeatCoverTemp;
	_stage_t stage[STAGE_MAX];
}_temp_data_t;

typedef struct _lab_data	{//ʵ������
	char id[LAB_ID_LEN];
	char name[LAB_NAME_LEN];
	char type;//ʵ�����͡�0-����/���ԣ�1-������2-�������
	char method;//ʵ�鷽����0-��׼���ߣ�1-�Ƚ�CT
}_lab_data_t;

typedef struct _hole_data	{//����Ϣ
	char name[LAB_NAME_LEN];
	char prj[LAB_NAME_LEN];
	char sample_t;//�������ͣ�S-��׼Ʒ;U-����;N-���Զ���;P-���Զ���;0x20-��
	char channel;//ͨ��	
}_hole_data_t;

typedef struct _sample_data	{//��������
//	u8 id;
	_hole_data_t hole[HOLE_NUM];
}_sample_data_t;

extern _sys_t Sys;
extern RTC_TIME_ST SysTime;
extern tlsf_t UserMem;
extern _syserror_t SysError;
extern _sample_data_t sample_data;
extern _lab_data_t	lab_data;
extern _temp_data_t temp_data;

void SysDataInit(void);

#endif
