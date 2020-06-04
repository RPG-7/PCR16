#ifndef __SYS_DATA_H
#define __SYS_DATA_H

#include <string.h>
#include "sys_bits.h"
#include "sys_types.h"
#include "sys_defs.h"
#include "tlsf.h"
#include "timestamp.h"

#define	FILE_NAME_LEN		50
#define	LOG_FILE_NAME		"Pcr16Log.txt"//"PerfusionLog.txt"
#define	TEMPJSON_FILE_NAME		"Temp.json"
#define	LabJSON_FILE_NAME		"Lab.json"
#define	CALI_FILE_NAME		"calibrate"

#define	LabFolderName		"Lab"
#define	DataFolderName		"Data"
#define	TmpFolderName		"tmp"

#define	HEATCOVER_TEMPMAX		110
#define	HEATCOVER_TEMPMIN		30

#define SysState_None							0
#define SysState_Standby					DEF_BIT00_MASK
#define SysState_DeleteLabTB			DEF_BIT01_MASK
#define SysState_ReadTXT				DEF_BIT02_MASK
#define SysState_ReadTXTOK					DEF_BIT03_MASK
#define SysState_RunningTB				DEF_BIT04_MASK
#define SysState_StopTB					DEF_BIT05_MASK
#define SysState_StageTB					DEF_BIT06_MASK
#define SysState_StepTB					DEF_BIT07_MASK
#define SysState_AddStep					DEF_BIT08_MASK
#define SysState_AddStage				DEF_BIT09_MASK
#define SysState_UpdataFWTB				DEF_BIT10_MASK
#define SysState_CaliHolePostion				DEF_BIT11_MASK
#define SysState_CollHolePD			DEF_BIT12_MASK
//�豸����ģʽ
enum devstate	{
	DevState_IDLE=0,//����ģʽ
	DevState_Running,//������
	DevState_Pause,//��ͣ��
	DevState_Standby,//����
	DevState_Error,//�豸�й���
	DevState_Debug,
};
//�豸������ģʽ
enum devsubstate	{	
	DevSubState_Fluo=3,//ӫ��ɼ���
	DevSubState_DebugTemp=4,
	DevSubState_Unkown=0xff,//δ֪
};

enum lab_type	{
	LabTypeNone=0,//��
	LabTypeNegativeOrPositive=0x01,//����/����
	LabTypeQuantify=0x02,//����
	LabTypeGeneticTyping=0x03,//�������
};

enum lab_method	{
	LabMethodNone=0,//��
	LabMethodStandard=0x01,//��׼����
	LabMethodCompare=0x02,//�Ƚ�CT
};

enum stage_type	{//0-repeatģʽ;1-continue ģʽ;2-step ģʽ
	StageByRepeat=0,//
	StageByContinue=0x01,//
	StageByStep=0x02,
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
#define	LAB_NAME_LEN		16
#define	HOLE_NUM		16
#define	STAGE_REPEAT_MAX		100
#define	HOLE_TEMP_MAX		9500//95�� 0.01
#define	HOLE_TEMP_MIN		100//1�� 
typedef struct _step	{
	u8 CollEnable;//�ɼ�ʹ��
	s16 temp;
	u16 tim;
}_step_t;

typedef struct _stage	{	
	u16 T_Rate;//��������
	u16 T_Inter;//�¶ȼ��
	u16 Const_Tim;//����ʱ�� s
	u8 Type;//0-repeatģʽ;�ܽ����ߣ�1-continue ģʽ;2-step ģʽ
	u8 RepeatNum;//
	u8 StepNum;//�ܲ� 
	u8 CurStep;//��ǰ�� ��д��json�ļ�
	u8 CurRepeat;//��ǰѭ�� ��д��json�ļ�
	_step_t step[STEP_MAX];
}_stage_t;

typedef struct _temp_data	{//�¶���Ϣ
	u8 StageNum;//�ܽ׶� 
	u8 CurStage;//��ǰ�׶� ��д��json�ļ�
//	u8 HeatCoverEnable;
	s16 HeatCoverTemp;
	_stage_t stage[STAGE_MAX];
}_temp_data_t;

typedef struct _lab_data	{//ʵ������
	char id[LAB_ID_LEN];
	char name[LAB_NAME_LEN];
	char type;//ʵ�����͡�0-�ޣ�0x01-����/���ԣ�0x02-������0x04-�������
	char method;//ʵ�鷽����0-�ޣ�0x01-��׼���ߣ�0x02-�Ƚ�CT
}_lab_data_t;

typedef struct _hole_data	{//����Ϣ
//	u8 enable;
	char name[LAB_NAME_LEN];
	char prj[LAB_NAME_LEN];
	char sample_t[2];//�������ͣ�S-��׼Ʒ;U-����;N-���Զ���;P-���Զ���;0-��
	char channel[4];//ͨ��	
}_hole_data_t;

typedef struct _sample_data	{//��������
//	u32 enable;
	_hole_data_t hole[HOLE_NUM];
}_sample_data_t;



typedef struct _position	{//
	s16 x1;
	s16 x2;
}_position_t;

typedef struct _hole_pos	{//
	u8 idx;
	_position_t pos[HOLE_NUM];
}_hole_pos_t;

extern _sys_t Sys;
extern RTC_TIME_ST SysTime;
extern tlsf_t UserMem;
extern _syserror_t SysError;
extern _sample_data_t sample_data;
extern _lab_data_t	lab_data;
extern _temp_data_t temp_data;
extern _hole_pos_t HolePos;

void *user_malloc(size_t size);
void user_free(void* ptr);	
void SysDataInit(void);
void HeatCoverOnOff(u8 flag);
void CollDataOnOff_InStep(u8 flag);
void DisableSampleData(u32 enable);
void SetSampleChannel(u32 enable, char typeidx);
void SetSampleType(u32 enable, char typeidx);
void ResetSampleDataDefault(void);
void ResetLabDataDefault(void);
void ResetTempDataDefault(void);
void ResetStage(u8 id);
void DelStage(u8 del_id);
void ResetStep(u8 stageid, u8 stepid);
void ClearAllSysStateTB(void);
#endif
