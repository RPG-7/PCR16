#include "app_temp.h"
#include "ad7124.h"
#include "timer.h"
#include "app_spiflash.h"
#include "app_motor.h"
//堆栈
__align(4) OS_STK  TASK_TEMP_STK[STK_SIZE_TEMP]; //任务堆栈声?

_app_temp_t app_temp;
temp_ctrl_t TempCtrl[TEMPCTRL_NUM];
#define	HOLE_TECPWM_PLUSE		400
#define	COVER_TECPWM_PLUSE		800
#define	HOLE_TECPWM_MAX		62//TEC pwm占空比最大值
#define	COVER_TECPWM_MAX		100//TEC pwm占空比最大值
#define	HOLECTRL_ACCURACY		10//孔温控精度±0.1
#define	COVERCTRL_ACCURACY		100//热盖温控精度±1
#define	TEMPCOLLECT_ACCURACY		5//温度采集精度 0.05
static  message_pkt_t    msg_pkt_temp;
static void AppTempTask (void *parg);

void AppTempInit (void)
{
	OSTaskCreate(AppTempTask,  (void * )0, (OS_STK *)&TASK_TEMP_STK[STK_SIZE_TEMP-1], TASK_PRIO_TEMP);
}
static void TempDatInit(void)
{
	TempCtrl[HOLE_ID].enable = DEF_False;
	TempCtrl[HOLE_ID].PIDid = PID_ID1;
	TempCtrl[HOLE_ID].pTECPWM = &htim8;
	TempCtrl[HOLE_ID].TimCH = TIM_CHANNEL_1;
	TempCtrl[HOLE_ID].TimPluse = HOLE_TECPWM_PLUSE;
	TempCtrl[HOLE_ID].DutyMax = HOLE_TECPWM_MAX;
	TempCtrl[HOLE_ID].TempMin = HOLE_TEMPMIN;//温度最小值
	TempCtrl[HOLE_ID].TempMax = HOLE_TEMPMAX;//温度最大值
	SetPIDOutputLimits(PID_ID1, -HOLE_TECPWM_MAX, HOLE_TECPWM_MAX);//设置PID控制输出上下限
	SetPIDVal(PID_ID1, P1_PARAM, I1_PARAM, D1_PARAM);//设置PID 参数
	
	TempCtrl[COVER_ID].enable = DEF_False;
	TempCtrl[COVER_ID].PIDid = PID_ID2;
	TempCtrl[COVER_ID].pTECPWM = &htim2;
	TempCtrl[COVER_ID].TimCH = TIM_CHANNEL_4;
	TempCtrl[COVER_ID].TimPluse = COVER_TECPWM_PLUSE;
	TempCtrl[COVER_ID].DutyMax = COVER_TECPWM_MAX;
	TempCtrl[COVER_ID].TempMin = HEATCOVER_TEMPMIN;//温度最小值
	TempCtrl[COVER_ID].TempMax = HEATCOVER_TEMPMAX;//温度最大值
	SetPIDOutputLimits(PID_ID2, 0, COVER_TECPWM_MAX);//设置PID控制输出上下限
	SetPIDVal(PID_ID2, P2_PARAM, I2_PARAM, D2_PARAM);//设置PID 参数
}
//实验开启温控
u8 StartAPPTempCtrl(void)
{
	msg_pkt_temp.Src = MSG_WriteLabTemplate;//保存实验模板, 路径 ./lab/Lab.json
	OSQPost(spiflash.MSG_Q, &msg_pkt_temp);	
	OSTimeDly(500);
//	if(temp_data.HeatCoverEnable)
//	SetTempCtrlTarget(COVER_ID, temp_data.HeatCoverTemp);//先开启热盖温控
	TempCtrl[HOLE_ID].enable = DEF_True;
	return 1;
}
//实验停止温控
void StopAPPTempCtrl(void)
{	
	StopTempCtrl(HOLE_ID);
	StopTempCtrl(COVER_ID);
	SoftTimerStop(&SoftTimer1, DEF_True);
	SoftTimerStop(&SoftTimer2, DEF_False);
}
//开启温控 设置温控目标
void SetTempCtrlTarget(u8 id, s16 temp)
{
	temp_ctrl_t *pTempCtrl = &TempCtrl[id];
	
	SetPIDTarget(pTempCtrl->PIDid, temp);
	pTempCtrl->enable = DEF_True;
}
//停止温控
void StopTempCtrl(u8 id)
{
	temp_ctrl_t *pTempCtrl = &TempCtrl[id];
	
	StopPWM(pTempCtrl->pTECPWM, pTempCtrl->TimCH);
	StopPIDControl(pTempCtrl->PIDid);
	pTempCtrl->enable = DEF_False;
}
//TEC pwm控制
static void StartTECPWM(temp_ctrl_t *pTempCtrl, u8 duty)
{
	u16 temp;
	
	temp = (pTempCtrl->TimPluse*duty)/100;
	if(duty>=100)	{
		temp += 1;
	}
	StartPWM(pTempCtrl->pTECPWM, pTempCtrl->TimCH, temp);
}
u16 setval;
//模块和孔温度调节 PID增量算法
static void TempControl(u8 id, u16 cur_t)
{
	s16 dat;
	float temp;
//	u16 setval;
	temp_ctrl_t *pTempCtrl = &TempCtrl[id];
	
	if(pTempCtrl->enable == DEF_False)//控温使能
		return;
	if(cur_t<pTempCtrl->TempMin || cur_t>pTempCtrl->TempMax)	{	//超过温度范围 半导体片停止工作
		StopTempCtrl(pTempCtrl->PIDid);		
		return;
	}
	temp = PIDControl(pTempCtrl->PIDid, cur_t);//PID 调节 增量算法
	dat = (s16)temp;
	if(id == HOLE_ID)	{//模块
		if(dat<0)	{//当前温度高于目标温度 将TEC切换到制冷模式 快速降温
			TEC_DIR_COLD();
		}
		else	{//当前温度低于目标温度 将TEC切换到制热模式 快速升温
			TEC_DIR_HOT();
		}
	}
	else	{//热盖
		if(dat<0)	{
			dat = 0;
		}
	}
	setval = abs(dat);
	if(setval > pTempCtrl->DutyMax)
		setval = pTempCtrl->DutyMax;
	StartTECPWM(pTempCtrl, setval);
//	SYS_PRINTF("D:%d,T:%d ",dat,cur_t);
}
//u8 hengwenflag;
//恒温时间达到 回调函数
static void ConstantTempArrivedCallback(void)
{
	u8 m;
	
	m = temp_data.CurStage;
	temp_data.stage[m].CurStep++;
	if(temp_data.stage[m].CurStep>=temp_data.stage[m].StepNum)	{//达到当前阶段的最后一步
		temp_data.stage[m].CurRepeat ++;
		if(temp_data.stage[m].CurRepeat>=temp_data.stage[m].RepeatNum)	{//达到当前阶段的最后一个循环 进入下阶段
			temp_data.CurStage++;
			if(temp_data.CurStage>=temp_data.StageNum)	{//达到最后一个阶段 停止控温
				Sys.devstate = DevState_IDLE;
			}else	{
				m = temp_data.CurStage;
				temp_data.stage[m].CurStep=0;
			}
		}else	{//未达到当前阶段的最后一个循环 继续该阶段			
			temp_data.stage[m].CurStep = 0;
		}
	}
	SoftTimerStop(&SoftTimer1, DEF_True);
}
//荧光采集 回调函数
static void PD_DataCollectCallback(void)
{
	msg_pkt_temp.Src = MSG_CollectHolePD_EVENT;//启动电机 开始采集孔PD值
	OSMboxPost(tMotor[MOTOR_ID1].Mbox, &msg_pkt_temp);
	SoftTimerStop(&SoftTimer2, DEF_False);
	Sys.devsubstate = DevSubState_CollectFluo;
}

//按照设置好的温度程序巡视 设置的温度曲线控温
static void TempProgramLookOver(s16 c_temp)
{
	u8 m,n;
	s16 target;
	static u8 ConstantTempCnt;
	
	if(Sys.devstate != DevState_Running)//只有实验情况下 才开启温度程序巡视
		return;
	if(TempCtrl[HOLE_ID].enable == DEF_False)//等热盖温度稳定后 再进行模块控温
		return;
	m = temp_data.CurStage;
	n = temp_data.stage[m].CurStep;
	target = temp_data.stage[m].step[n].temp;
	if(abs(c_temp-target)>HOLECTRL_ACCURACY)	{//温度差大于0.15度 当前处于升/降温阶段		
		ConstantTempCnt = 0;
		SetPIDTarget(PID_ID1, target);//设置目标温度 开启控温
//		hengwenflag = 0;
		Sys.devsubstate = DevSubState_TempUp;
	}
	else {//到达目标温度 当前处于恒温阶段
		ConstantTempCnt++;
		if(ConstantTempCnt>=5)	{//持续400ms 温度差小于0.2度 判断温度控制已稳定
			Sys.devsubstate = DevSubState_TempKeep;//恒温阶段
			ConstantTempCnt = 0;
			if(GetSoftTimerState(&SoftTimer1)==DEF_Stop)	{//100ms 为单位
				SoftTimerStart(&SoftTimer1, temp_data.stage[m].step[n].tim*10, DEF_True); //设置恒温时间定时
				SoftTimer1.pCallBack = &ConstantTempArrivedCallback;
				if(temp_data.stage[m].step[n].CollEnable)	{//是否启动PD数据采集
					SoftTimerStart(&SoftTimer2, (temp_data.stage[m].step[n].tim-8)*10, DEF_False);//提前8s启动PD数据采集
					SoftTimer2.pCallBack = &PD_DataCollectCallback;
				}
			}
//			hengwenflag = target;
		}
	}
}
//风扇控制：flag 开关，duty pwm占空比
static void StartCoolFan(u8 flag, u8 duty)
{
	u32 temp;
	
	if(flag==DEF_ON)	{
		temp = (80*duty)/100;
		StartPWM(&htim4, TIM_CHANNEL_3, temp);
	}
	else if(flag==DEF_OFF)
		StopPWM(&htim4, TIM_CHANNEL_3);
}

static void AppTempTask (void *parg)
{
	s32 cur_temp;
	
	PIDParamInit();
	TempDatInit();
	StopTempCtrl(HOLE_ID);
	StopTempCtrl(COVER_ID);
	OSTimeDly(1000);
	StartCoolFan(DEF_ON, 100-68);//打开制冷片风扇 默认50%占空比
	EquipFAN_OFF();//打开设备风扇

	for(;;)
    {
		if(Sys.devstate == DevState_Running||Sys.devsubstate == DevSubState_DebugTemp)	
		{
			if(CalcTemperature(GetADCVol(TEMP_ID1), &cur_temp)==0)	{//计算模块温度				
				app_temp.current_t[TEMP_ID1] = cur_temp;//0.01
				TempProgramLookOver(cur_temp);//温度程序巡视 
				TempControl(HOLE_ID, cur_temp);//孔温度调节
				SysError.Y1.bits.b3 = DEF_Active;
			}else	{//温度传感器脱落
				SysError.Y1.bits.b3 = DEF_Inactive;//温度传感器异常
			}
			if(CalcTemperature(GetADCVol(TEMP_ID2), (s32 *)&cur_temp)==0)	{
				app_temp.current_t[TEMP_ID2] = cur_temp;
				SysError.Y1.bits.b4 = DEF_Active;				
			}else	{
				SysError.Y1.bits.b4 = DEF_Inactive;
			}
			if(CalcTemperature(GetADCVol(TEMP_ID3), (s32 *)&cur_temp)==0)	{//计算热盖温度
				app_temp.current_t[TEMP_ID3] = cur_temp;
				if((Sys.devstate == DevState_Running)&&abs(cur_temp - temp_data.HeatCoverTemp)<COVERCTRL_ACCURACY)	{//实验时 热盖温度稳定在±1度内 开启模块控温
					TempCtrl[HOLE_ID].enable = DEF_True;
				}
				TempControl(COVER_ID, cur_temp);//热盖温度调节			
				SysError.Y1.bits.b5 = DEF_Active;
			}else	{
				SysError.Y1.bits.b5 = DEF_Inactive;
			}
//			if(CalcTemperature(GetADCVol(TEMP_ID4), (s32 *)&cur_temp)==0)	{//散热器 预留
//				app_temp.current_t[TEMP_ID4] = cur_temp;				
//			}else	{
//			
//			}
		}
		else
		{
			StopTempCtrl(HOLE_ID);
			StopTempCtrl(COVER_ID);
		}
		OSTimeDly(80);
	}
}
//获取热盖温度
s16 GetCoverTemperature(void)
{
	return app_temp.current_t[TEMP_ID3];
}
//获取孔模块1 2温度
s16 GetHoleTemperature(u8 hole)
{
	if(hole==1)
		return app_temp.current_t[TEMP_ID1];
	else
		return app_temp.current_t[TEMP_ID2];
}

