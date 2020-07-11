#include "app_temp.h"
#include "ad7124.h"
#include "timer.h"
#include "app_spiflash.h"
#include "app_motor.h"
//��ջ
__align(4) OS_STK  TASK_TEMP_STK[STK_SIZE_TEMP]; //�����ջ��?

_app_temp_t app_temp;
temp_ctrl_t TempCtrl[TEMPCTRL_NUM];
#define	HOLE_TECPWM_PLUSE		400
#define	COVER_TECPWM_PLUSE		800
#define	HOLE_TECPWM_MAX		62//TEC pwmռ�ձ����ֵ
#define	COVER_TECPWM_MAX		100//TEC pwmռ�ձ����ֵ
#define	HOLECTRL_ACCURACY		10//���¿ؾ��ȡ�0.1
#define	COVERCTRL_ACCURACY		100//�ȸ��¿ؾ��ȡ�1
#define	TEMPCOLLECT_ACCURACY		5//�¶Ȳɼ����� 0.05
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
	TempCtrl[HOLE_ID].TempMin = HOLE_TEMPMIN;//�¶���Сֵ
	TempCtrl[HOLE_ID].TempMax = HOLE_TEMPMAX;//�¶����ֵ
	SetPIDOutputLimits(PID_ID1, -HOLE_TECPWM_MAX, HOLE_TECPWM_MAX);//����PID�������������
	SetPIDVal(PID_ID1, P1_PARAM, I1_PARAM, D1_PARAM);//����PID ����
	
	TempCtrl[COVER_ID].enable = DEF_False;
	TempCtrl[COVER_ID].PIDid = PID_ID2;
	TempCtrl[COVER_ID].pTECPWM = &htim2;
	TempCtrl[COVER_ID].TimCH = TIM_CHANNEL_4;
	TempCtrl[COVER_ID].TimPluse = COVER_TECPWM_PLUSE;
	TempCtrl[COVER_ID].DutyMax = COVER_TECPWM_MAX;
	TempCtrl[COVER_ID].TempMin = HEATCOVER_TEMPMIN;//�¶���Сֵ
	TempCtrl[COVER_ID].TempMax = HEATCOVER_TEMPMAX;//�¶����ֵ
	SetPIDOutputLimits(PID_ID2, 0, COVER_TECPWM_MAX);//����PID�������������
	SetPIDVal(PID_ID2, P2_PARAM, I2_PARAM, D2_PARAM);//����PID ����
}
//ʵ�鿪���¿�
u8 StartAPPTempCtrl(void)
{
	msg_pkt_temp.Src = MSG_WriteLabTemplate;//����ʵ��ģ��, ·�� ./lab/Lab.json
	OSQPost(spiflash.MSG_Q, &msg_pkt_temp);	
	OSTimeDly(500);
//	if(temp_data.HeatCoverEnable)
//	SetTempCtrlTarget(COVER_ID, temp_data.HeatCoverTemp);//�ȿ����ȸ��¿�
	TempCtrl[HOLE_ID].enable = DEF_True;
	return 1;
}
//ʵ��ֹͣ�¿�
void StopAPPTempCtrl(void)
{	
	StopTempCtrl(HOLE_ID);
	StopTempCtrl(COVER_ID);
	SoftTimerStop(&SoftTimer1, DEF_True);
	SoftTimerStop(&SoftTimer2, DEF_False);
}
//�����¿� �����¿�Ŀ��
void SetTempCtrlTarget(u8 id, s16 temp)
{
	temp_ctrl_t *pTempCtrl = &TempCtrl[id];
	
	SetPIDTarget(pTempCtrl->PIDid, temp);
	pTempCtrl->enable = DEF_True;
}
//ֹͣ�¿�
void StopTempCtrl(u8 id)
{
	temp_ctrl_t *pTempCtrl = &TempCtrl[id];
	
	StopPWM(pTempCtrl->pTECPWM, pTempCtrl->TimCH);
	StopPIDControl(pTempCtrl->PIDid);
	pTempCtrl->enable = DEF_False;
}
//TEC pwm����
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
//ģ��Ϳ��¶ȵ��� PID�����㷨
static void TempControl(u8 id, u16 cur_t)
{
	s16 dat;
	float temp;
//	u16 setval;
	temp_ctrl_t *pTempCtrl = &TempCtrl[id];
	
	if(pTempCtrl->enable == DEF_False)//����ʹ��
		return;
	if(cur_t<pTempCtrl->TempMin || cur_t>pTempCtrl->TempMax)	{	//�����¶ȷ�Χ �뵼��Ƭֹͣ����
		StopTempCtrl(pTempCtrl->PIDid);		
		return;
	}
	temp = PIDControl(pTempCtrl->PIDid, cur_t);//PID ���� �����㷨
	dat = (s16)temp;
	if(id == HOLE_ID)	{//ģ��
		if(dat<0)	{//��ǰ�¶ȸ���Ŀ���¶� ��TEC�л�������ģʽ ���ٽ���
			TEC_DIR_COLD();
		}
		else	{//��ǰ�¶ȵ���Ŀ���¶� ��TEC�л�������ģʽ ��������
			TEC_DIR_HOT();
		}
	}
	else	{//�ȸ�
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
//����ʱ��ﵽ �ص�����
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
//ӫ��ɼ� �ص�����
static void PD_DataCollectCallback(void)
{
	msg_pkt_temp.Src = MSG_CollectHolePD_EVENT;//������� ��ʼ�ɼ���PDֵ
	OSMboxPost(tMotor[MOTOR_ID1].Mbox, &msg_pkt_temp);
	SoftTimerStop(&SoftTimer2, DEF_False);
	Sys.devsubstate = DevSubState_CollectFluo;
}

//�������úõ��¶ȳ���Ѳ�� ���õ��¶����߿���
static void TempProgramLookOver(s16 c_temp)
{
	u8 m,n;
	s16 target;
	static u8 ConstantTempCnt;
	
	if(Sys.devstate != DevState_Running)//ֻ��ʵ������� �ſ����¶ȳ���Ѳ��
		return;
	if(TempCtrl[HOLE_ID].enable == DEF_False)//���ȸ��¶��ȶ��� �ٽ���ģ�����
		return;
	m = temp_data.CurStage;
	n = temp_data.stage[m].CurStep;
	target = temp_data.stage[m].step[n].temp;
	if(abs(c_temp-target)>HOLECTRL_ACCURACY)	{//�¶Ȳ����0.15�� ��ǰ������/���½׶�		
		ConstantTempCnt = 0;
		SetPIDTarget(PID_ID1, target);//����Ŀ���¶� ��������
//		hengwenflag = 0;
		Sys.devsubstate = DevSubState_TempUp;
	}
	else {//����Ŀ���¶� ��ǰ���ں��½׶�
		ConstantTempCnt++;
		if(ConstantTempCnt>=5)	{//����400ms �¶Ȳ�С��0.2�� �ж��¶ȿ������ȶ�
			Sys.devsubstate = DevSubState_TempKeep;//���½׶�
			ConstantTempCnt = 0;
			if(GetSoftTimerState(&SoftTimer1)==DEF_Stop)	{//100ms Ϊ��λ
				SoftTimerStart(&SoftTimer1, temp_data.stage[m].step[n].tim*10, DEF_True); //���ú���ʱ�䶨ʱ
				SoftTimer1.pCallBack = &ConstantTempArrivedCallback;
				if(temp_data.stage[m].step[n].CollEnable)	{//�Ƿ�����PD���ݲɼ�
					SoftTimerStart(&SoftTimer2, (temp_data.stage[m].step[n].tim-8)*10, DEF_False);//��ǰ8s����PD���ݲɼ�
					SoftTimer2.pCallBack = &PD_DataCollectCallback;
				}
			}
//			hengwenflag = target;
		}
	}
}
//���ȿ��ƣ�flag ���أ�duty pwmռ�ձ�
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
	StartCoolFan(DEF_ON, 100-68);//������Ƭ���� Ĭ��50%ռ�ձ�
	EquipFAN_OFF();//���豸����

	for(;;)
    {
		if(Sys.devstate == DevState_Running||Sys.devsubstate == DevSubState_DebugTemp)	
		{
			if(CalcTemperature(GetADCVol(TEMP_ID1), &cur_temp)==0)	{//����ģ���¶�				
				app_temp.current_t[TEMP_ID1] = cur_temp;//0.01
				TempProgramLookOver(cur_temp);//�¶ȳ���Ѳ�� 
				TempControl(HOLE_ID, cur_temp);//���¶ȵ���
				SysError.Y1.bits.b3 = DEF_Active;
			}else	{//�¶ȴ���������
				SysError.Y1.bits.b3 = DEF_Inactive;//�¶ȴ������쳣
			}
			if(CalcTemperature(GetADCVol(TEMP_ID2), (s32 *)&cur_temp)==0)	{
				app_temp.current_t[TEMP_ID2] = cur_temp;
				SysError.Y1.bits.b4 = DEF_Active;				
			}else	{
				SysError.Y1.bits.b4 = DEF_Inactive;
			}
			if(CalcTemperature(GetADCVol(TEMP_ID3), (s32 *)&cur_temp)==0)	{//�����ȸ��¶�
				app_temp.current_t[TEMP_ID3] = cur_temp;
				if((Sys.devstate == DevState_Running)&&abs(cur_temp - temp_data.HeatCoverTemp)<COVERCTRL_ACCURACY)	{//ʵ��ʱ �ȸ��¶��ȶ��ڡ�1���� ����ģ�����
					TempCtrl[HOLE_ID].enable = DEF_True;
				}
				TempControl(COVER_ID, cur_temp);//�ȸ��¶ȵ���			
				SysError.Y1.bits.b5 = DEF_Active;
			}else	{
				SysError.Y1.bits.b5 = DEF_Inactive;
			}
//			if(CalcTemperature(GetADCVol(TEMP_ID4), (s32 *)&cur_temp)==0)	{//ɢ���� Ԥ��
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
//��ȡ�ȸ��¶�
s16 GetCoverTemperature(void)
{
	return app_temp.current_t[TEMP_ID3];
}
//��ȡ��ģ��1 2�¶�
s16 GetHoleTemperature(u8 hole)
{
	if(hole==1)
		return app_temp.current_t[TEMP_ID1];
	else
		return app_temp.current_t[TEMP_ID2];
}

