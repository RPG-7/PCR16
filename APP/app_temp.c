#include "app_temp.h"
#include "ad7124.h"
#include "timer.h"
#include "app_spiflash.h"
#include "app_motor.h"
//��ջ
__align(4) OS_STK  TASK_TEMP_STK[STK_SIZE_TEMP]; //�����ջ��?

_app_temp_t app_temp;
pid_ctrl_t TempPid[PIDCTRL_NUM];
#define	HOLE_TECPWM_PLUSE		400
#define	COVER_TECPWM_PLUSE		800
#define	HOLE_TECPWM_MAX		62//TEC pwmռ�ձ����ֵ
#define	COVER_TECPWM_MAX		100//TEC pwmռ�ձ����ֵ
static  message_pkt_t    msg_pkt_temp;
static void AppTempTask (void *parg);

void AppTempInit (void)
{
	OSTaskCreate(AppTempTask,  (void * )0, (OS_STK *)&TASK_TEMP_STK[STK_SIZE_TEMP-1], TASK_PRIO_TEMP);
}

static void TempDatInit(void)
{
	TempPid[HOLE_TEMP].PIDid = PID_ID1;
	TempPid[HOLE_TEMP].pTECPWM = &htim8;
	TempPid[HOLE_TEMP].TimCH = TIM_CHANNEL_1;
	TempPid[HOLE_TEMP].TimPluse = HOLE_TECPWM_PLUSE;
	TempPid[HOLE_TEMP].DutyMax = HOLE_TECPWM_MAX;
	TempPid[HOLE_TEMP].target_t = 3700;//0.01
	TempPid[HOLE_TEMP].PIDParam = 0.0;
	
	TempPid[COVER_TEMP].PIDid = PID_ID2;
	TempPid[COVER_TEMP].pTECPWM = &htim2;
	TempPid[COVER_TEMP].TimCH = TIM_CHANNEL_4;
	TempPid[COVER_TEMP].TimPluse = COVER_TECPWM_PLUSE;
	TempPid[COVER_TEMP].DutyMax = COVER_TECPWM_MAX;
	TempPid[COVER_TEMP].target_t = 0;	
	TempPid[COVER_TEMP].PIDParam = 0.0;
}
//TEC pwm����
void StartTECPWM(pid_ctrl_t *pTempPid, u8 duty)
{
	static u8 dutybk;
	u16 temp;
	
	if(dutybk==duty)
		return;
	temp = (pTempPid->TimPluse/100)*duty;
	if(duty==100)	{
		temp++;
	}
	StartPWM(pTempPid->pTECPWM, pTempPid->TimCH, temp);
	dutybk = duty;
}

//ֹͣ�¶ȿ���
static void StopTempCtrl(pid_ctrl_t *pTempPid)
{
	StopPWM(pTempPid->pTECPWM, pTempPid->TimCH);
	pTempPid->PIDParam = 0.0;
}
u16 setval;
//pid���ڰ뵼��Ƭ�¶� �������������� pwmռ�ձȲ��ܳ���50%
static void TempCtrl(pid_ctrl_t *pTempPid, u16 cur_t)
{
	s16 dat;
	float temp;
//	u16 setval;
	
	temp = pTempPid->PIDParam;
	temp += PID_control(pTempPid->PIDid, dat, cur_t);
	pTempPid->PIDParam = temp;
	dat = (s16)floatToInt(temp);
	if(dat<0)	{//��ǰ�¶ȸ���Ŀ���¶� ��TEC�л�������ģʽ ���ٽ���
		if(dat < -pTempPid->DutyMax)
			setval = pTempPid->DutyMax;
		else
			setval = -dat;
		TEC_DIR_COLD();
	}
	else {//��ǰ�¶ȵ���Ŀ���¶� ��TEC�л�������ģʽ ��������
		if(dat > pTempPid->DutyMax)
			setval = pTempPid->DutyMax;
		else
			setval = dat;
		TEC_DIR_HOT();
	}
	StartTECPWM(pTempPid, setval);
//	SYS_PRINTF("D:%d,T:%d ",dat,cur_t);
}
//
u8 StartAPPTempCtrl(void)
{
	msg_pkt_temp.Src = MSG_WriteLabTemplate;//����ʵ��ģ��, ·��./lab/Temp.json; ./lab/Lab.json
	OSQPost(spiflash.MSG_Q, &msg_pkt_temp);	
	OSTimeDly(500);
	ClearPIDDiff(TempPid[HOLE_TEMP].PIDid);	
	SetPIDVal(PID_ID1, 0.65, 0.00025, 5.8);
	return 1;
}

void StopAPPTempCtrl(void)
{	
	SoftTimerStop(&SoftTimer1, DEF_True);
	SoftTimerStop(&SoftTimer2, DEF_False);
}
u8 hengwenflag;
//����ʱ��ﵽ ���øú���
static void ConstantTempArrivedCallback(void)
{
	u8 m;
	
	m = temp_data.CurStage;
	temp_data.stage[m].CurStep++;
	if(temp_data.stage[m].CurStep>=temp_data.stage[m].StepNum)	{//�ﵽ��ǰ�׶ε����һ��
		temp_data.stage[m].CurRepeat ++;
		if(temp_data.stage[m].CurRepeat>=temp_data.stage[m].RepeatNum)	{//�ﵽ��ǰ�׶ε����һ��ѭ�� �����½׶�
			temp_data.CurStage++;
			if(temp_data.CurStage>=temp_data.StageNum)	{//�ﵽ���һ���׶� ֹͣ����
				Sys.devstate = DevState_IDLE;
			}else	{
				m = temp_data.CurStage;
				temp_data.stage[m].CurStep=0;
			}
		}else	{//δ�ﵽ��ǰ�׶ε����һ��ѭ�� �����ý׶�			
			temp_data.stage[m].CurStep = 0;
		}
	}
	SoftTimerStop(&SoftTimer1, DEF_True);
}

static void PD_DataCollectCallback(void)
{
	msg_pkt_temp.Src = MSG_CollectHolePD_EVENT;//������� ��ʼ�ɼ���PDֵ
	OSMboxPost(tMotor[MOTOR_ID1].Mbox, &msg_pkt_temp);
	SoftTimerStop(&SoftTimer2, DEF_False);
}

//�������úõ��¶ȳ���Ѳ�� ���õ��¶����߿���
void TempProgramLookOver(s16 c_temp)
{
	u8 m,n;
	s16 target;
	static u8 ConstantTempCnt;
	
	m = temp_data.CurStage;
	n = temp_data.stage[m].CurStep;
	target = temp_data.stage[m].step[n].temp;
	if(abs(c_temp-target)>100)	{//�¶Ȳ����1�� ��ǰ���������½׶�		
		ConstantTempCnt = 0;
		SetPIDTarget(PID_ID1, target);
		hengwenflag = 0;
	}
	else {//����Ŀ���¶� ��ǰ���ں��½׶�
		ConstantTempCnt++;
		if(ConstantTempCnt>=5)	{//����500ms �¶Ȳ�С��1�� �ж��¶ȿ������ȶ�
			ConstantTempCnt = 0;
			if(GetSoftTimerState(&SoftTimer1)==DEF_Stop)	{//100ms Ϊ��λ
				SoftTimerStart(&SoftTimer1, temp_data.stage[m].step[n].tim*10, DEF_True); //���ú���ʱ�䶨ʱ
				SoftTimer1.pCallBack = &ConstantTempArrivedCallback;
				SoftTimerStart(&SoftTimer2, (temp_data.stage[m].step[n].tim-8)*10, DEF_False);//����PD���ݲɼ�ʱ�䶨ʱ
				SoftTimer2.pCallBack = &PD_DataCollectCallback;
			}
			hengwenflag = target;
		}
	}
}

#define	TEMPCTRL_ACCURACY		10//�¿ؾ���0.1
#define	TEMPCOLLECT_ACCURACY		5//�¶Ȳɼ����� 0.05
static void AppTempTask (void *parg)
{
	s32 cur_temp;
	
	TempDatInit();
	PIDParamInit();
	StopTempCtrl(&TempPid[HOLE_TEMP]);
	OSTimeDly(1000);
//	COOLFAN_ON();
//	SetPIDVal(PID_ID1, 0.65, 0.00025, 5.8);
	SetPIDVal(PID_ID1, 0.0, 0.0, 0.0);
	
	for(;;)
    {
		if(Sys.devstate == DevState_Running||Sys.devsubstate == DevSubState_DebugTemp)	
		{
			if(CalcTemperature(GetADCVol(TEMP_ID1), &cur_temp)==0)	{//����ģ���¶�
				TempProgramLookOver(cur_temp);
				app_temp.current_t[TEMP_ID1] = cur_temp;//0.01
//				SetPIDTarget(PID_ID1, TempPid[HOLE_TEMP].target_t);//���ÿ���Ŀ��
				TempCtrl(&TempPid[HOLE_TEMP], cur_temp);//pid���� ����������
			}else	{//�¶ȴ���������
			
			}
			if(CalcTemperature(GetADCVol(TEMP_ID2), (s32 *)&cur_temp)==0)	{
				app_temp.current_t[TEMP_ID2] = cur_temp;				
			}else	{
			
			}
			if(CalcTemperature(GetADCVol(TEMP_ID3), (s32 *)&cur_temp)==0)	{
				app_temp.current_t[TEMP_ID3] = cur_temp;
//				SetPIDTarget(PID_ID2, TempPid[COVER_TEMP].target_t);//���ÿ���Ŀ��
//				TempCtrl(&TempPid[COVER_TEMP], cur_temp);//�ȸ�pid���� ����������
			}else	{
			
			}
//			if(CalcTemperature(GetADCVol(TEMP_ID4), (s32 *)&cur_temp)==0)	{//ɢ���� Ԥ��
//				app_temp.current_t[TEMP_ID4] = cur_temp;				
//			}else	{
//			
//			}
		}
		else	
		{
			ClearPIDDiff(TempPid[HOLE_TEMP].PIDid);
			StopTempCtrl(&TempPid[HOLE_TEMP]);
			ClearPIDDiff(TempPid[COVER_TEMP].PIDid);
			StopTempCtrl(&TempPid[COVER_TEMP]);
		}
		OSTimeDly(80);
	}
}

s16 GetCoverTemperature(void)
{
	return app_temp.current_t[TEMP_ID3];
}

s16 GetHoleTemperature(void)
{
	return app_temp.current_t[TEMP_ID1];
}

