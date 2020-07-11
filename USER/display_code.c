#include "display_code.h"
#include "rw_spiflash.h"

typedef struct 	{
	_MultiDat data[16];
}_MultiTXT_;
_MultiTXT_ *pMultiTXT_t;

const char Code_Warning[][12] = {	
	{"是否删除"},
	{"是否启动"},
	{"是否停止"},
};

const char Code_Message[][12] = {	
	{"复制完成!"},
	{"U盘未连接!"},
	{"复制中..."},
	{"无效操作"},
	{"设置成功"},
	{"运行中..."},
	{"删除中...:"},
};

const char Code_Choose[][5] = {	
	{"取消"},
	{"确认"},
	{"删除"},
	{"编辑"},
};

void SaveUIEditInfor(void)
{
	appdis.pUI->editinfo.screen_id = appdis.pUI->screen_id;
	appdis.pUI->editinfo.ctrl_id = appdis.pUI->ctrl_id;
}

void UUIDBackup(void)
{
	appdis.pUI->screen_idbk = appdis.pUI->screen_id;
	appdis.pUI->ctrl_idbk = appdis.pUI->ctrl_id;
}

void DisplayUIIDAndBackup(u8 id)
{
	UUIDBackup();
	appdis.pUI->screen_id = id;
	DaCai_SwitchUI(appdis.pUI);
}

void DisplayBackupUIID(void)
{
	appdis.pUI->screen_id = appdis.pUI->screen_idbk;
	appdis.pUI->ctrl_id = appdis.pUI->ctrl_idbk;
	DaCai_SwitchUI(appdis.pUI);
	if(appdis.pUI->screen_id==Lab_UIID)	{
		DisplayLabUI();
	}
	else if(appdis.pUI->screen_id==Menu_UIID)	{
		appdis.pUI->button_id = 19;
		if(Sys.devstate == DevState_Running)	{		
			DaCai_ButtonCtrl(appdis.pUI, DEF_Press);
		}
		else	{
			DaCai_ButtonCtrl(appdis.pUI, DEF_Release);
		}
	}
	else if(appdis.pUI->screen_id==Temp_UIID)	{
		DisplayTempProgramUI(0,0);
	}
}

void DisplayEditUI(void)
{
	appdis.pUI->screen_id = appdis.pUI->editinfo.screen_id;
	appdis.pUI->ctrl_id = appdis.pUI->editinfo.ctrl_id;
	DaCai_SwitchUI(appdis.pUI);
}

void DisplayKeyboardUI(void)
{
//	s8 id;
	DisplayUIIDAndBackup(Keyboard_UIID);
//	id = appdis.pUI->ctrl_id;
	appdis.pUI->ctrl_id = 43;
	DaCai_ClearTXT(appdis.pUI);
}

void DisplayMessageUI(char *pbuf, u8 backup)
{	
	if(backup)	
		UUIDBackup();
	appdis.pUI->screen_id = Message_UIID;
	DaCai_SwitchUI(appdis.pUI);
	appdis.pUI->ctrl_id  = 4;	
	appdis.pUI->datlen = sprintf((char *)appdis.pUI->pdata,"%s", pbuf);
	DaCai_UpdateTXT(appdis.pUI);
}

void DisplayWarningUI(char *pbuf, char *buf1, char *buf2)
{	
	DisplayUIIDAndBackup(Confirm_UIID);
	appdis.pUI->ctrl_id  = 4;	
	appdis.pUI->datlen = sprintf((char *)appdis.pUI->pdata,"%s", pbuf);
	DaCai_UpdateTXT(appdis.pUI);
//	if(buf1!=NULL)	{	
		appdis.pUI->ctrl_id  = 1;	
		appdis.pUI->datlen = sprintf((char *)appdis.pUI->pdata,"%s", buf1);
		DaCai_UpdateTXT(appdis.pUI);
//	}
//	if(buf2!=NULL)	{
		appdis.pUI->ctrl_id  = 5;	
		appdis.pUI->datlen = sprintf((char *)appdis.pUI->pdata,"%s", buf2);
		DaCai_UpdateTXT(appdis.pUI);
//	}
}

void DisplayLogUI(void)
{
	u32 ret;

	appdis.pUI->screen_id = Log_UIID;//LOG界面					
	DaCai_SwitchUI(appdis.pUI);
	ret = read_log(appdis.pUI->pdata);
	if(ret)	{//读取log长度不为0
		appdis.pUI->ctrl_id = 3;
		appdis.pUI->datlen = ret;
		DaCai_UpdateTXT(appdis.pUI);
	}
}

//显示实验界面
void DisplayLabUI(void)
{
	u8 i;
	u8 button_id[6],value[6];

	appdis.pUI->screen_id = Lab_UIID;
	DaCai_SwitchUI(appdis.pUI);

	appdis.pUI->button_id = 1;
	for(i=0;i<5;i++)	{
		button_id[i] = appdis.pUI->button_id ++;
		value[i] = BUTTON_RELEASE;
	}
	button_id[i] = 19;
	if(Sys.devstate == DevState_Running)	{		
		value[i] = BUTTON_PRESS;
	}
	else	{
		value[i] = BUTTON_RELEASE;
	}
	DaCai_UpdateMultiButton(appdis.pUI, button_id, value, i+1);
	pMultiTXT_t = (_MultiTXT_ *)user_malloc(sizeof(_MultiTXT_));
	ReadLabTemplateList();//读取实验列表
	appdis.pUI->ctrl_id = 6;
	for(i=0;i<gLabTemplatelist.num;i++)	{			
		pMultiTXT_t->data[i].id = appdis.pUI->ctrl_id++;
		pMultiTXT_t->data[i].len = sprintf(pMultiTXT_t->data[i].buf, "%-25.20s%s",gLabTemplatelist.list[i].time, gLabTemplatelist.list[i].name);				
	}
	for(;i<5;i++)	{
		pMultiTXT_t->data[i].id = appdis.pUI->ctrl_id++;
		pMultiTXT_t->data[i].buf[0] = 0;
		pMultiTXT_t->data[i].len = 1;
	}
	DaCai_UpdateMultiTXT(appdis.pUI, pMultiTXT_t->data, i);
	user_free(pMultiTXT_t);
	appdis.pUI->index = 0x0f;
}
//显示实验属性界面
void DisplayLabAttrUI(void)
{
	u8 button_id[3],value[3];
	
	appdis.pUI->screen_id = LabAttr_UIID;
	DaCai_SwitchUI(appdis.pUI);
	appdis.pUI->ctrl_id  = 5;	
	appdis.pUI->datlen = sprintf((char *)appdis.pUI->pdata,"%s", lab_data.name);
	DaCai_UpdateTXT(appdis.pUI);
	button_id[0] = 6;
	if(lab_data.type==LabTypeNegativeOrPositive) 
		value[0] = BUTTON_PRESS;
	else
		value[0] = BUTTON_RELEASE;
	button_id[1] = 7;
	if(lab_data.type==LabTypeQuantify) 
		value[1] = BUTTON_PRESS;
	else
		value[1] = BUTTON_RELEASE;
	button_id[2] = 8;
	if(lab_data.type==LabTypeGeneticTyping) 
		value[2] = BUTTON_PRESS;
	else
		value[2] = BUTTON_RELEASE;
	DaCai_UpdateMultiButton(appdis.pUI, button_id, value, 3);//更新多个控件
	button_id[0] = 9;
	if(lab_data.method==LabMethodStandard) 
		value[0] = BUTTON_PRESS;
	else
		value[0] = BUTTON_RELEASE;
	button_id[1] = 10;
	if(lab_data.method==LabMethodCompare) 
		value[1] = BUTTON_PRESS;
	else
		value[1] = BUTTON_RELEASE;
	DaCai_UpdateMultiButton(appdis.pUI, button_id, value, 2);//更新多个控件
}
//清除样本信息界面的按钮选中状态
void ClearButtonInSampleInfor(void)
{
	u8 i;
	u8 button_id[HOLE_NUM],value[HOLE_NUM];
	
	for(i=0;i<HOLE_NUM;i++)	{//Release按钮状态
		value[i] = BUTTON_RELEASE;
		button_id[i] = i+1;
	}
	DaCai_UpdateMultiButton(appdis.pUI, button_id, value, i);
	appdis.pUI->button_id = 0;
}
//更新样本类型数据显示
void UpdateSampleInfor(void)
{
	u8 i,j,ctrl_id;
	
	pMultiTXT_t = (_MultiTXT_ *)user_malloc(sizeof(_MultiTXT_));
	ctrl_id = 46;
	j = 0;
	for(i=0;i<HOLE_NUM;i++)	{			
		pMultiTXT_t->data[j].id = ctrl_id++;
		pMultiTXT_t->data[j].len = sprintf(pMultiTXT_t->data[j].buf, "%s\r\n\r\n%s", sample_data.hole[i].sample_t,sample_data.hole[i].channel);
		j++;
	}
	DaCai_UpdateMultiTXT(appdis.pUI, pMultiTXT_t->data, j);
	user_free(pMultiTXT_t);
}
//显示样本信息界面
void DisplaySampleInforUI(void)
{
	appdis.pUI->screen_id = SampleInfor_UIID;
	DaCai_SwitchUI(appdis.pUI);
//	appdis.pUI->button_id = sample_data.enable;
	ClearButtonInSampleInfor();
	UpdateSampleInfor();
}
//更新样本信息 列表界面
void UpdateSampleInforList(u8 index)
{
	u8 i,j,ctrl_id;
	u8 button_id[5],value[5];
	u8 hole_idx;
	
	ctrl_id = 26;
	pMultiTXT_t = (_MultiTXT_ *)user_malloc(sizeof(_MultiTXT_));
	for(i=0;i<5;i++)	{
		hole_idx = index + i;
		if(hole_idx<HOLE_NUM)
		{
			if(sample_data.hole[hole_idx].sample_t[0] != 0)
				value[i] = DEF_Press;
			else
				value[i] = DEF_Release;
			button_id[i] = 1+i;
			j=0;
			pMultiTXT_t->data[j].id = ctrl_id++;
			pMultiTXT_t->data[j].len = sprintf(pMultiTXT_t->data[j].buf, "A%d", hole_idx+1);
			pMultiTXT_t->data[++j].id = ctrl_id++;
			pMultiTXT_t->data[j].len = sprintf(pMultiTXT_t->data[j].buf, "%s", sample_data.hole[hole_idx].name);
			pMultiTXT_t->data[++j].id = ctrl_id++;
			pMultiTXT_t->data[j].len = sprintf(pMultiTXT_t->data[j].buf, "%s", sample_data.hole[hole_idx].prj);
			pMultiTXT_t->data[++j].id = ctrl_id++;
			pMultiTXT_t->data[j].len = sprintf(pMultiTXT_t->data[j].buf, "%s", sample_data.hole[hole_idx].sample_t);
			pMultiTXT_t->data[++j].id = ctrl_id++;
			pMultiTXT_t->data[j].len = sprintf(pMultiTXT_t->data[j].buf, "None");//实验结果
		}
		else	{
			value[i] = DEF_Release;
			button_id[i] = 1+i;
			j=0;
			pMultiTXT_t->data[j].id = ctrl_id++;
			pMultiTXT_t->data[j].len = sprintf(pMultiTXT_t->data[j].buf, " ");
			pMultiTXT_t->data[++j].id = ctrl_id++;
			pMultiTXT_t->data[j].len = sprintf(pMultiTXT_t->data[j].buf, " ");
			pMultiTXT_t->data[++j].id = ctrl_id++;
			pMultiTXT_t->data[j].len = sprintf(pMultiTXT_t->data[j].buf, " ");
			pMultiTXT_t->data[++j].id = ctrl_id++;
			pMultiTXT_t->data[j].len = sprintf(pMultiTXT_t->data[j].buf, " ");
			pMultiTXT_t->data[++j].id = ctrl_id++;
			pMultiTXT_t->data[j].len = sprintf(pMultiTXT_t->data[j].buf, " ");//实验结果
		}
		DaCai_UpdateMultiTXT(appdis.pUI, pMultiTXT_t->data, j+1);
	}
	user_free(pMultiTXT_t);
	DaCai_UpdateMultiButton(appdis.pUI, button_id, value, 5);
}
//显示样本信息 列表界面	
void DisplaySampleInforUIByList(void)
{
	appdis.pUI->screen_id = SampleList_UIID;
	DaCai_SwitchUI(appdis.pUI);
	appdis.pUI->index=0;
	UpdateSampleInforList(appdis.pUI->index);	
}

void DisplayMenuUI(void)
{
	appdis.pUI->screen_id = Menu_UIID;
	DaCai_SwitchUI(appdis.pUI);
	appdis.pUI->button_id = 19;
	if(Sys.devstate == DevState_Running)	{		
		DaCai_ButtonCtrl(appdis.pUI, DEF_Press);
	}
	else	{
		DaCai_ButtonCtrl(appdis.pUI, DEF_Release);
	}
}

//void DisplayHeatCoverIcon(void)
//{
//	appdis.pUI->ctrl_id = 8;
//	DaCai_IconCtrl(appdis.pUI,temp_data.HeatCoverEnable);
//}
//根据当前实验状态，提示停止实验还是启动实验
void DisplayQiTingLab(void)
{	
	if(Sys.devstate == DevState_Running)	{//停止实验
			sprintf((char *)appdis.pUI->pdata,"%s实验 %s ?", &Code_Warning[2][0],lab_data.name);//显示 是否停止
			DisplayWarningUI((char *)appdis.pUI->pdata, (char *)&Code_Choose[0][0], (char *)&Code_Choose[1][0]);
//			appdis.pUI->datlen = sprintf((char *)appdis.pUI->pdata,"%s", &Code_Warning[2][0]);//显示 是否停止
			Sys.state |= SysState_StopTB;
	}
	else if(Sys.devstate == DevState_IDLE)		{	//启动实验
		if(lab_data.name[0] == 0)	{//启动前 检查
			sprintf((char *)appdis.pUI->pdata,"%s","无效实验名称");
			DisplayMessageUI((char *)appdis.pUI->pdata,1);
		}
		else	if(temp_data.StageNum==0)	{
			sprintf((char *)appdis.pUI->pdata,"%s","无效温度程序");
			DisplayMessageUI((char *)appdis.pUI->pdata,1);
		}
		else	{		
			sprintf((char *)appdis.pUI->pdata,"%s实验 %s ?", &Code_Warning[1][0], lab_data.name);//显示 是否启动 
			DisplayWarningUI((char *)appdis.pUI->pdata, (char *)&Code_Choose[0][0], (char *)&Code_Choose[1][0]);		
			Sys.state |= SysState_RunningTB;
		}
	}
}

void DisplayStepUI(s8 stageid, s8 stepid)
{
	s8 m,n,i;

	pMultiTXT_t = (_MultiTXT_ *)user_malloc(sizeof(_MultiTXT_));
	appdis.pUI->screen_id = Step_UIID;//step界面					
	DaCai_SwitchUI(appdis.pUI);

	m = stageid;
	n = stepid;
	i=0;
	pMultiTXT_t->data[++i].id = 8;
	pMultiTXT_t->data[i].len = sprintf(pMultiTXT_t->data[i].buf, "%d.%d", temp_data.stage[m].step[n].temp/100,(temp_data.stage[m].step[n].temp%100)/10);
	pMultiTXT_t->data[++i].id = 9;
	pMultiTXT_t->data[i].len = sprintf(pMultiTXT_t->data[i].buf, "%d", temp_data.stage[m].step[n].tim/60);
	pMultiTXT_t->data[++i].id = 10;
	pMultiTXT_t->data[i].len = sprintf(pMultiTXT_t->data[i].buf, "%d", temp_data.stage[m].step[n].tim%60);
	DaCai_UpdateMultiTXT(appdis.pUI, pMultiTXT_t->data, i+1);

	appdis.pUI->button_id = 1;
	DaCai_ButtonCtrl(appdis.pUI, temp_data.stage[m].step[n].CollEnable);
	user_free(pMultiTXT_t);
}

void DisplayStageUI(s8 stageid)
{
	u8 i,m;
	
	appdis.pUI->screen_id = Stage_UIID;//stage界面					
	DaCai_SwitchUI(appdis.pUI);
	m = stageid;
	i=0;
	pMultiTXT_t = (_MultiTXT_ *)user_malloc(sizeof(_MultiTXT_));
	pMultiTXT_t->data[++i].id = 8;
	pMultiTXT_t->data[i].len = sprintf(pMultiTXT_t->data[i].buf, "%d", temp_data.stage[m].StepNum);
	pMultiTXT_t->data[++i].id = 4;
	pMultiTXT_t->data[i].len = sprintf(pMultiTXT_t->data[i].buf, "%d", temp_data.stage[m].RepeatNum);
	DaCai_UpdateMultiTXT(appdis.pUI, pMultiTXT_t->data, i+1);
	user_free(pMultiTXT_t);
}
#define TEMP_RECTANGLE_X	17
#define TEMP_RECTANGLE_X_INTER	154
#define TEMP_RECTANGLE_Y	142
#define TEMP_RECTANGLE_W	150
#define TEMP_RECTANGLE_H	200
#define TEMP_UI_MAXSTEP		5
#include "DaCai_TouchEvent.h"

static void ClearTempProgramUI(u8 clear_flag)
{
	u8 j;
	
	if(clear_flag==0)
		return;
	for(j=0;j<9;j++)	{
		appdis.pUI->ctrl_id = 15+j;
		DaCai_ClearTXT(appdis.pUI);
		DaCai_SetTXTBackgroundNoColor(appdis.pUI);
		appdis.pUI->ctrl_id = 24+j;
		DaCai_ClearTXT(appdis.pUI);
		DaCai_SetTXTBackgroundNoColor(appdis.pUI);
	}
}
struct _dis_idx_t	{
	u8 StageIdx;
	u8 StepIdx;
 }CurIdx,NextIdx;
static u16 g_templast;
//请温度界面阶段和步标志
void ClearTempProgramIdx(void)
{
	CurIdx.StageIdx = 0;
	CurIdx.StepIdx = 0;
	NextIdx.StageIdx = 0;
	NextIdx.StepIdx = 0;
	g_templast = 30;
}

//按照按钮id 查找当前是那个stage step
u8 CheckIdFromButton(u8 button, u8 *stageid, u8 *stepid)
{
	s8 i,j;
	u8 flag;
	u8 Stageid,Stepid,stepcnt;
	
	Stageid = 0;
	Stepid = 0;
	flag = 0;
	stepcnt = 0;
	j = CurIdx.StepIdx;
	for(i=CurIdx.StageIdx;i<temp_data.StageNum;i++)	{//根据按钮id 找到要删除的stage id和step id
		for(;j<temp_data.stage[i].StepNum;j++)	{
			if(button==stepcnt)	{
				flag=1;
				Stageid = i;
				Stepid = j;
				break;
			}
			stepcnt ++;
			if(stepcnt>=TEMP_UI_MAXSTEP)	{
				return 0;
			}
		}
		if(flag)	{
			*stageid = Stageid;
			*stepid = Stepid;
			return 1;
		}
		j=0;
	}
	return 0;
}
//删除操作 type=0 删阶段，type=1 删除步
void DeleTempProgam(u8 button, u8 type)
{
	s8 i,j,k;
	u8 Stageid,Stepid;

	if(CheckIdFromButton(button, &Stageid, &Stepid)==1)
	{
		if(type==0)	{//删阶
			DelStage(Stageid);
		}
		else if(type==1)	{//删步
			i = Stageid;
			j = temp_data.stage[i].StepNum-1;
			if(j==0)	{//该阶段无步 直接删除该阶段
				DelStage(Stageid);
				temp_data.stage[i].StepNum = j;				
			}
			else if(j>0)	{
				for(k=Stepid;k<j;k++)	{
					memcpy(&temp_data.stage[i].step[k], &temp_data.stage[i].step[k+1], sizeof(_step_t));
				}
				temp_data.stage[i].StepNum = j;
				ResetStep(i,k);
			}
		}
	}
	ClearTempProgramIdx();		
	DisplayTempProgramUI(0,1);
}

//温度程序 界面刷新
void DisplayTempProgramUI(u8 page_flag, u8 clear_flag)
{
	u8 i,j,k,stepcnt;
	u8 stage_id,repeat_id;
	u16 rec_x,rec_y,height,rec_y_last;
	u16 xie_w,xie_h;
	u16 temp,templast;
	u8 flag;
	
	appdis.pUI->screen_id = Temp_UIID;
	DaCai_SwitchUI(appdis.pUI);
	
//	DisplayHeatCoverIcon();	

	ClearTempProgramUI(clear_flag);
	pMultiTXT_t = (_MultiTXT_ *)user_malloc(sizeof(_MultiTXT_));
	i=0;
	if(page_flag)	{//翻页处理
		if(NextIdx.StageIdx>=temp_data.StageNum)	
			ClearTempProgramIdx();
		else	{
			CurIdx.StageIdx = NextIdx.StageIdx;
			CurIdx.StepIdx = NextIdx.StepIdx;
		}
	}
	pMultiTXT_t->data[i].id = 6;
	pMultiTXT_t->data[i].len = sprintf(pMultiTXT_t->data[i].buf, "%d", temp_data.HeatCoverTemp/100);

	stage_id = 14;repeat_id = 23;
	j = CurIdx.StageIdx;
	flag = temp_data.stage[j].StepNum - CurIdx.StepIdx;
	for(;temp_data.StageNum>0;)	{//显示标题
		stage_id += flag;
		pMultiTXT_t->data[++i].id = stage_id;
		stage_id += flag;
		pMultiTXT_t->data[i].len = sprintf(pMultiTXT_t->data[i].buf, "Stage%d", j+1);
//		appdis.pUI->ctrl_id = pMultiTXT_t->data[i].id;
//		DaCai_SetTXTBackgroundColor(appdis.pUI, 0x0397);
		repeat_id += flag;
		pMultiTXT_t->data[++i].id = repeat_id;
		repeat_id += flag;
		pMultiTXT_t->data[i].len = sprintf(pMultiTXT_t->data[i].buf, "x%d", temp_data.stage[j].RepeatNum);
		appdis.pUI->ctrl_id = pMultiTXT_t->data[i].id;
		if(Sys.devstate != DevState_Running)	{
			DaCai_SetTXTBackgroundColor(appdis.pUI, 0xffff);
			DaCai_SetForeColor(appdis.pUI, 0x033F);
		}
		else	{
			DaCai_SetTXTBackgroundColor(appdis.pUI, 0x0397);
			DaCai_SetForeColor(appdis.pUI, 0xffff);
		}
		if(repeat_id>=32)
			break;		
		j++;
		if(j>=temp_data.StageNum)
			break;
		flag = temp_data.stage[j].StepNum;
	}
	DaCai_UpdateMultiTXT(appdis.pUI, pMultiTXT_t->data, i+1);		
	user_free(pMultiTXT_t);
	templast = g_templast;
	stepcnt = 0;
	k=CurIdx.StepIdx;
	for(j=CurIdx.StageIdx;j<temp_data.StageNum;j++)	{//刷新图形
		for(;k<temp_data.stage[j].StepNum;k++)	{
			temp = temp_data.stage[j].step[k].temp/100;	//温度精确到0.01		
			rec_x = TEMP_RECTANGLE_X + TEMP_RECTANGLE_X_INTER*stepcnt;
			height = temp*(TEMP_RECTANGLE_H/100);
			rec_y = TEMP_RECTANGLE_Y+TEMP_RECTANGLE_H-height;
			if(temp>templast)	{		
				DaCai_DisplayCutPic(rec_x, rec_y, 2, 0, 0, TEMP_RECTANGLE_W, height);
				xie_h = (temp - templast)*(TEMP_RECTANGLE_H/100);
				xie_w = xie_h/(TEMP_RECTANGLE_H/50);
				DaCai_DisplayCutPic(rec_x, rec_y, 1, 50-xie_w, 0, xie_w, xie_h);
			}
			else	if(temp<templast)	{
				DaCai_DisplayCutPic(rec_x, rec_y, 2, 0, 0, TEMP_RECTANGLE_W, height);
//				height = templast*(TEMP_RECTANGLE_H/100);
				rec_y_last = TEMP_RECTANGLE_Y+TEMP_RECTANGLE_H - templast*(TEMP_RECTANGLE_H/100);
				xie_h = (templast-temp)*(TEMP_RECTANGLE_H/100);
				xie_w = xie_h/(TEMP_RECTANGLE_H/50);
				DaCai_DisplayCutPic(rec_x, rec_y_last, 0, 0, 0, xie_w, xie_h);
			}
			else	{
				DaCai_DisplayCutPic(rec_x, rec_y, 2, 0, 0, TEMP_RECTANGLE_W, height);
			}
			DaCai_SetFontColor(65504);//设置文字颜色 黄色
			appdis.pUI->datlen = sprintf((char *)appdis.pUI->pdata,"%d.%d℃", temp, temp_data.stage[j].step[k].temp%10);
			DaCai_DisplayTXT(appdis.pUI, rec_x+60, rec_y-18, 6);
			appdis.pUI->datlen = sprintf((char *)appdis.pUI->pdata,"%d:%d", temp_data.stage[j].step[k].tim/60, temp_data.stage[j].step[k].tim%60);
			DaCai_DisplayTXT(appdis.pUI, rec_x+60, rec_y+5, 6);
			if(temp_data.stage[j].step[k].CollEnable)
				PaintTriangle(rec_x+130,rec_y,14,30);
			templast = temp;
			stepcnt	++;
			if(stepcnt>=TEMP_UI_MAXSTEP)	{//显示已满 
				g_templast = temp;				
				k++;
				if(k>=temp_data.stage[j].StepNum)
					NextIdx.StepIdx = 0;
				else
					NextIdx.StepIdx = k;
				NextIdx.StageIdx = ++j;
				return;
			}
		}
		k=0;
	}
}
//画三角形 (x,y)是顶点坐标 w,h是三角形宽和高
void PaintTriangle(u16 x, u16 y, u16 w, u16 h)
{
	_coordinate_t coo[4];
	u16 w2,h2;
	u8 i;
	
	w2 = w/2;
	h2 = h/2;
	for(i=0;i<2;i++)	{
		coo[0].x = x+i;
		coo[0].y = y+i;
		coo[1].x = x-w2+i;
		coo[1].y = y+h2+i;
		coo[2].x = x+w2+i;
		coo[2].y = y+h2+i;
		coo[3].x = x+i;
		coo[3].y = y+i;
		DaCai_PaintLine(coo, 4);
	}
}


