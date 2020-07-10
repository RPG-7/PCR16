#include "sys_data.h"
#include "json.h"

_sys_t Sys;
_syserror_t SysError;
RTC_TIME_ST SysTime;
tlsf_t UserMem;

_sample_data_t sample_data;
_lab_data_t	lab_data;
_temp_data_t temp_data;
_hole_pos_t HolePos;

const char SampleType[][2] = {
	{0},{"S"},{"U"},{"N"},{"P"},
};//样本类型：S-标准品;U-待测;N-阴性对照;P-阳性对照;0-空

const char SampleChannel[][4] = {
	{0},{"FAM"},{"HEX"}
};//样本类型：S-标准品;U-待测;N-阴性对照;P-阳性对照;0-空

void SysDataInit(void)
{
	UserMem = tlsf_create_with_pool((void *)0x20012000, 0x6000);//内存0x12000 - 0x18000 区域24KB内存使用tlsf管理
	jansson_init();
	SysError.Y1.ubyte = 0x0;//主板故障
	SysError.Y2.ubyte = 0x0;//
	
	Sys.state = SysState_None;
	Sys.devstate = DevState_IDLE;
	
	SysTime.tm_year = 2019;
	SysTime.tm_mon = 7;
	SysTime.tm_mday = 17;
	SysTime.tm_hour = 17;
	SysTime.tm_min = 44;
	SysTime.tm_sec = 20;
	SysTime.tm_wday = 3;
	
	memset(&HolePos,0,sizeof(HolePos));
	ResetLabDataDefault();
	ResetSampleDataDefault();
	ResetTempDataDefault();
//	CreateTemp_Jsonfile();
//	CreateLab_Jsonfile();
//	jansson_pack_test();
}
//设置样本类型
void SetSampleType(u32 enable, char typeidx)
{
	u8 i;
	u32 tmp;
	
	for(i=0;i<HOLE_NUM;i++)	{
		tmp = enable&(DEF_BIT00_MASK<<i);
		if(tmp)	{
			strcpy(sample_data.hole[i].sample_t, &SampleType[typeidx][0]);
//			sample_data.enable |= tmp;
		}
	}
}
//设置样本通道
void SetSampleChannel(u32 enable, char typeidx)
{
	u8 i;
	u32 tmp;
	
	for(i=0;i<HOLE_NUM;i++)	{
		tmp = enable&(DEF_BIT00_MASK<<i);
		if(tmp)	{
			strcpy(sample_data.hole[i].channel, &SampleChannel[typeidx][0]);
//			sample_data.enable |= tmp;
		}
	}
}
//关闭样本空
void DisableSampleData(u32 enable)
{
	u8 i;
	u32 tmp;
	
	for(i=0;i<HOLE_NUM;i++)	{
		tmp = enable&(DEF_BIT00_MASK<<i);
		if(tmp)	{
			strcpy(sample_data.hole[i].sample_t, &SampleType[0][0]);
			strcpy(sample_data.hole[i].channel, &SampleChannel[0][0]);
//			sample_data.enable &= ~tmp;
		}
	}
}
//设置样本信息默认值
void ResetSampleDataDefault(void)
{
	u8 i;
	
//	sample_data.enable = 0;
	for(i=0;i<HOLE_NUM;i++)	{
		memset((void *)sample_data.hole[i].name, 0, LAB_NAME_LEN);
		memset(sample_data.hole[i].prj, 0, LAB_NAME_LEN);
		strcpy(sample_data.hole[i].channel, &SampleChannel[0][0]);
		strcpy(sample_data.hole[i].sample_t, &SampleType[0][0]);
	}
}
//设置实验数据默认值
void ResetLabDataDefault(void)
{
	memset(lab_data.name,0,LAB_NAME_LEN);
	lab_data.type = LabTypeNone;
	lab_data.method = LabMethodNone;
}

void ResetStep(u8 stageid, u8 stepid)
{
	temp_data.stage[stageid].step[stepid].CollEnable = DEF_False;
	temp_data.stage[stageid].step[stepid].temp = HOLE_TEMPDEFAULT;
	temp_data.stage[stageid].step[stepid].tim = 60;
}

void ResetStage(u8 id)
{
	u8 j;
	
	temp_data.stage[id].RepeatNum = 1;
	temp_data.stage[id].StepNum = 0;
	temp_data.stage[id].CurStep = 0;
	temp_data.stage[id].CurRepeat = 0;
	temp_data.stage[id].Type = 0;//0-repeat模式;溶解曲线：1-continue 模式;2-step 模式
	temp_data.stage[id].T_Rate = 0;//升温速率
	temp_data.stage[id].T_Inter = 0;//温度间隔
	temp_data.stage[id].Const_Tim = 0;//恒温时间 s
	for(j=0;j<STEP_MAX;j++)	{
		ResetStep(id,j);
	}
}
//删除阶段
void DelStage(u8 del_id)
{
	s8 i,j;
	
	j = temp_data.StageNum-1;
	if(j>=0)	{
		for(i=del_id;i<j;i++)	{
			memcpy(&temp_data.stage[i], &temp_data.stage[i+1], sizeof(_stage_t));
		}
		ResetStage(i);
		temp_data.StageNum = j;
	}
}
//设置温度程序默认值
void ResetTempDataDefault(void)
{
	u8 i;
//	temp_data.HeatCoverEnable = DEF_True;
	temp_data.HeatCoverTemp = HEATCOVER_TEMPDEFAULT;
	temp_data.StageNum = 0;
	temp_data.CurStage = 0;
	for(i=0;i<STAGE_MAX;i++)	{
		ResetStage(i);
	}
}

//设置温度程序DNA值
void ResetTempDataDNA(void)
{
	temp_data.HeatCoverTemp = HEATCOVER_TEMPDEFAULT;
	temp_data.StageNum = 2;
	temp_data.CurStage = 0;
	temp_data.stage[0].RepeatNum = 10;
	temp_data.stage[0].StepNum = 2;
	temp_data.stage[0].CurStep = 0;
	temp_data.stage[0].CurRepeat = 0;
	temp_data.stage[0].Type = 0;//0-repeat模式;溶解曲线：1-continue 模式;2-step 模式
	temp_data.stage[0].T_Rate = 0;//升温速率
	temp_data.stage[0].T_Inter = 0;//温度间隔
	temp_data.stage[0].Const_Tim = 0;//恒温时间 s
	temp_data.stage[0].step[0].CollEnable = DEF_False;
	temp_data.stage[0].step[0].temp = HOLE_TEMPDEFAULT;
	temp_data.stage[0].step[0].tim = 30;
	temp_data.stage[0].step[1].CollEnable = DEF_True;
	temp_data.stage[0].step[1].temp = 4500;
	temp_data.stage[0].step[1].tim = 45;
	
	temp_data.stage[1].RepeatNum = 10;
	temp_data.stage[1].StepNum = 3;
	temp_data.stage[1].CurStep = 0;
	temp_data.stage[1].CurRepeat = 0;
	temp_data.stage[1].Type = 0;//0-repeat模式;溶解曲线：1-continue 模式;2-step 模式
	temp_data.stage[1].T_Rate = 0;//升温速率
	temp_data.stage[1].T_Inter = 0;//温度间隔
	temp_data.stage[1].Const_Tim = 0;//恒温时间 s
	temp_data.stage[1].step[0].CollEnable = DEF_False;
	temp_data.stage[1].step[0].temp = 7500;
	temp_data.stage[1].step[0].tim = 30;
	temp_data.stage[1].step[1].CollEnable = DEF_True;
	temp_data.stage[1].step[1].temp = 5500;
	temp_data.stage[1].step[1].tim = 45;
	temp_data.stage[1].step[2].CollEnable = DEF_True;
	temp_data.stage[1].step[2].temp = 9500;
	temp_data.stage[1].step[2].tim = 45;
}
//设置温度程序RNA值
void ResetTempDataRNA(void)
{
	temp_data.HeatCoverTemp = HEATCOVER_TEMPDEFAULT;
	temp_data.StageNum = 2;
	temp_data.CurStage = 0;
	temp_data.stage[0].RepeatNum = 10;
	temp_data.stage[0].StepNum = 2;
	temp_data.stage[0].CurStep = 0;
	temp_data.stage[0].CurRepeat = 0;
	temp_data.stage[0].Type = 0;//0-repeat模式;溶解曲线：1-continue 模式;2-step 模式
	temp_data.stage[0].T_Rate = 0;//升温速率
	temp_data.stage[0].T_Inter = 0;//温度间隔
	temp_data.stage[0].Const_Tim = 0;//恒温时间 s
	temp_data.stage[0].step[0].CollEnable = DEF_False;
	temp_data.stage[0].step[0].temp = HOLE_TEMPDEFAULT;
	temp_data.stage[0].step[0].tim = 30;
	temp_data.stage[0].step[1].CollEnable = DEF_True;
	temp_data.stage[0].step[1].temp = 4500;
	temp_data.stage[0].step[1].tim = 45;
	
	temp_data.stage[1].RepeatNum = 10;
	temp_data.stage[1].StepNum = 3;
	temp_data.stage[1].CurStep = 0;
	temp_data.stage[1].CurRepeat = 0;
	temp_data.stage[1].Type = 0;//0-repeat模式;溶解曲线：1-continue 模式;2-step 模式
	temp_data.stage[1].T_Rate = 0;//升温速率
	temp_data.stage[1].T_Inter = 0;//温度间隔
	temp_data.stage[1].Const_Tim = 0;//恒温时间 s
	temp_data.stage[1].step[0].CollEnable = DEF_False;
	temp_data.stage[1].step[0].temp = 7500;
	temp_data.stage[1].step[0].tim = 30;
	temp_data.stage[1].step[1].CollEnable = DEF_True;
	temp_data.stage[1].step[1].temp = 5500;
	temp_data.stage[1].step[1].tim = 45;
	temp_data.stage[1].step[2].CollEnable = DEF_True;
	temp_data.stage[1].step[2].temp = 9500;
	temp_data.stage[1].step[2].tim = 45;
}

void ClearAllSysStateTB(void)
{
	Sys.state &= ~SysState_RunningTB;
	Sys.state &= ~SysState_StopTB;
	Sys.state &= ~SysState_DeleteLabTB;
	Sys.state &= ~SysState_StageTB;
	Sys.state &= ~SysState_StepTB;
}
//重定向tlsf_malloc
void *user_malloc(size_t size)
{
	void *ret;
	ret = tlsf_malloc(UserMem, size);
	return ret;
}
//重定向user_free
void user_free(void* ptr)
{
	if(ptr == NULL)
		return;
	tlsf_free(UserMem, ptr);
	ptr = NULL;
}
