#ifndef __SYS_DATA_H
#define __SYS_DATA_H

#include <string.h>
#include "sys_bits.h"
#include "sys_types.h"
#include "sys_defs.h"
#include "tlsf.h"
#include "timestamp.h"

#define	FILE_NAME_LEN		50
#define	LOG_FILE_NAME		"PCRLog.txt"//"PerfusionLog.txt"
//#define	TEMPJSON_FILE_NAME		"Temp.json"
#define	LabJSON_FILE_NAME		"Lab.json"
#define	CALI_FILE_NAME		"calibrate"

#define	LabFolderName		"Lab"//实验文件夹 保存最新的5次实验配置信息
#define	DataFolderName		"Data"//数据文件夹 保存最新的50条实验数据 包括配置信息和荧光数据
#define	TmpFolderName		"Tmp"//临时文件夹 其它临时文件
#define	ServerLabFolderName		"Tmp/Lab"//保存上位机设置的实验配置信息+荧光数据(只保存一个) 

#define	HOLE_TEMPMAX		10000//模块孔温度控制范围 10-100
#define	HOLE_TEMPMIN		100
#define	HOLE_TEMPDEFAULT		9500
#define	HEATCOVER_TEMPMAX		11000//热盖温度控制范围 0-110
#define	HEATCOVER_TEMPMIN		-4000
#define	HEATCOVER_TEMPDEFAULT		10500

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
#define SysState_CaliHolePostion		DEF_BIT11_MASK
#define SysState_CollHolePD				DEF_BIT12_MASK
#define SysState_LabFromServer			DEF_BIT13_MASK//来自上位机的实验
#define SysState_CoverTempOk			DEF_BIT14_MASK//热盖温度已经稳定
//设备运行模式
enum devstate	{
	DevState_IDLE=0,//待机模式
	DevState_Running,//运行中
	DevState_Pause,//暂停中
	DevState_Standby,//休眠
	DevState_Error,//设备有故障
	DevState_Debug,
};
//设备运行子模式
enum devsubstate	{
	DevSubState_TempUp=0,//温度上升中
	DevSubState_TempDn=1,//温度下降中
	DevSubState_TempKeep=2,//温度保持中
	DevSubState_CollectFluo=3,//荧光采集中
	DevSubState_DebugTemp = 4,
    DevSubState_Unkown=0xFF,//未知
};

enum lab_type	{
	LabTypeNone=0,//无
	LabTypeNegativeOrPositive=0x01,//阴性/阳性
	LabTypeQuantify=0x02,//定量
	LabTypeGeneticTyping=0x03,//基因分型
};

enum lab_method	{
	LabMethodNone=0,//无
	LabMethodStandard=0x01,//标准曲线
	LabMethodCompare=0x02,//比较CT
};

enum stage_type	{//0-repeat模式;1-continue 模式;2-step 模式
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
	volatile u8 devstate;//设备运行状态
	volatile u8 devsubstate;//设备运行子状态
	volatile u8 AlarmType;//报警类型
	u8 StandbyFlag;
}_sys_t;

//typedef struct _sys_data	{
//	s8 HeatCoverTemp;//热盖温度
//	s8 Temp1;//
//	s8 Temp2;//
//	s16 PD_1;//PD传感器1
//	s16 PD_2;//PD传感器2
//}_sys_data_t;
#define	STAGE_MAX		9
#define	STEP_MAX		9
#define	LAB_ID_LEN		22
#define	LAB_NAME_LEN		16
#define	HOLE_NUM		16
#define	STAGE_REPEAT_MAX		100
//#define	HOLE_TEMP_MAX		9500//95度 0.01
//#define	HOLE_TEMP_MIN		100//1度 
typedef struct _step	{
	u8 CollEnable;//采集使能
	s16 temp;
	u16 tim;
}_step_t;

typedef struct _stage	{	
	u16 T_Rate;//升温速率
	u16 T_Inter;//温度间隔
	u16 Const_Tim;//恒温时间 s
	u8 Type;//0-repeat模式;溶解曲线：1-continue 模式;2-step 模式
	u8 RepeatNum;//
	u8 StepNum;//总步 
	u8 CurStep;//当前步 不写入json文件
	u8 CurRepeat;//当前循环 不写入json文件
	_step_t step[STEP_MAX];
}_stage_t;

typedef struct _temp_data	{//温度信息
	u8 StageNum;//总阶段 
	u8 CurStage;//当前阶段 不写入json文件
//	u8 HeatCoverEnable;
	s16 HeatCoverTemp;
	_stage_t stage[STAGE_MAX];
}_temp_data_t;

typedef struct _lab_data	{//实验数据
	char id[LAB_ID_LEN];
	char name[LAB_NAME_LEN];
	char type;//实验类型。0-无；0x01-阴性/阳性；0x02-定量；0x04-基因分型
	char method;//实验方法。0-无；0x01-标准曲线；0x02-比较CT
}_lab_data_t;

typedef struct _hole_data	{//孔信息
//	u8 enable;
	char name[LAB_NAME_LEN];
	char prj[LAB_NAME_LEN];
	char sample_t[2];//样本类型：S-标准品;U-待测;N-阴性对照;P-阳性对照;0-空
	char channel[4];//通道	
}_hole_data_t;

typedef struct _sample_data	{//样本数据
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
void *json_malloc(size_t size);
void json_free(void* ptr);	
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
