#include "app_temp.h"
#include "ad7124.h"
#include "PID.h"

//��ջ
__align(4) OS_STK  TASK_TEMP_STK[STK_SIZE_TEMP]; //�����ջ��?

_app_temp_t app_temp;
pid_ctrl_t TempPid[PIDCTRL_NUM];

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
	TempPid[HOLE_TEMP].target_t = 3700;//0.01
	TempPid[HOLE_TEMP].PIDParam = 0.0;
	
	TempPid[COVER_TEMP].PIDid = PID_ID2;
	TempPid[COVER_TEMP].pTECPWM = &htim2;
	TempPid[COVER_TEMP].target_t = 0;	
	TempPid[COVER_TEMP].PIDParam = 0.0;
}

#define	TECPWM_CH		TIM_CHANNEL_1
#define	TECPWM_PLUSE		400
static void StopTECPWM(TIM_HandleTypeDef *pPWM)
{
	HAL_TIM_PWM_Stop(pPWM, TECPWM_CH);
}
//�޸�TEC pwmռ�ձ�
static void UpdateTECPWM(TIM_HandleTypeDef *pPWM, INT16U duty)
{
	u16 temp;
	
	temp = (TECPWM_PLUSE/100)*duty;
    __HAL_TIM_SET_AUTORELOAD(pPWM, TECPWM_PLUSE);
    __HAL_TIM_SET_COMPARE(pPWM, TECPWM_CH, temp);
}

static void StartTECPWM(TIM_HandleTypeDef *pPWM, u16 duty)
{
	static u16 dutybk;
	
	if(dutybk==duty)
		return;
	if(duty==100)	{
		duty++;
	}
	__HAL_TIM_CLEAR_FLAG(pPWM, TIM_FLAG_UPDATE);
	UpdateTECPWM(pPWM, duty);
	HAL_TIM_PWM_Start(pPWM, TECPWM_CH);	
	dutybk = duty;
}
//ֹͣ�¶ȿ���
static u32 StopTempCtrl(pid_ctrl_t *pTempPid)
{
	StopTECPWM(pTempPid->pTECPWM);
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
		if(dat<-TECPWMDUTY_MAX)
			setval = TECPWMDUTY_MAX;
		else
			setval = -dat;
		TEC_DIR_COLD();
	}
	else {//��ǰ�¶ȵ���Ŀ���¶� ��TEC�л�������ģʽ ��������
		if(dat>TECPWMDUTY_MAX)
			setval = TECPWMDUTY_MAX;
		else
			setval = dat;
		TEC_DIR_HOT();
	}
	StartTECPWM(pTempPid->pTECPWM, setval);
//	SYS_PRINTF("D:%d,T:%d ",dat,cur_t);
}
//
void StartTempCtrl(u16 target)
{
	TempPid[HOLE_TEMP].target_t = target;
	ClearPIDDiff(TempPid[HOLE_TEMP].PIDid);
	Sys.devstate = DevState_Running;
}

//float Kpbk;
#define	TEMPCTRL_MAX		1000//
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
//	SetPIDVal(PID_ID1, 1, 0, 0);
	for(;;)
    {
		if(Sys.devstate == DevState_Running)	
		{
			if(CalcTemperature(GetADCVol(TEMP_ID1), &cur_temp)==0)	{
				app_temp.current_t[TEMP_ID1] = cur_temp;//0.01
				{
					SetPIDTarget(TempPid[HOLE_TEMP].PIDid, TempPid[HOLE_TEMP].target_t);//���ÿ���Ŀ��
//					diff = abs(TempPid[HOLE_TEMP].target_t - cur_temp);//��ȡPID��ǰ���
//					if(diff > 200)	{//�����¶Ȳ� ����pid����
//						SetPIDVal(PID_ID1, 1, 0, 0);
//					}
//					else 
//						SetPIDVal(PID_ID1, 1, 0, 0);
//					if(diff > TEMPCOLLECT_ACCURACY)	
					{//����0.1�� 
						TempCtrl(&TempPid[HOLE_TEMP], cur_temp);//pid���� ����������
					}
//					else	{//С<=0.1�� ������ TECֹͣ����
//						ClearPIDDiff(TempPid[HOLE_TEMP].PIDid);
//						StopTempCtrl(&TempPid[HOLE_TEMP]);			
//					}
				}
			}else	{//�¶ȴ���������
			
			}
			if(CalcTemperature(GetADCVol(TEMP_ID2), (s32 *)&cur_temp)==0)	{
				app_temp.current_t[TEMP_ID2] = cur_temp;				
			}else	{
			
			}
//			if(CalcTemperature(GetADCVol(TEMP_ID3), (s32 *)&cur_temp)==0)	{
//				app_temp.current_t[TEMP_ID3] = cur_temp;
//			}else	{
//			
//			}
			if(CalcTemperature(GetADCVol(TEMP_ID4), (s32 *)&cur_temp)==0)	{
				app_temp.current_t[TEMP_ID4] = cur_temp;
				TempCtrl(&TempPid[COVER_TEMP], cur_temp);//�ȸ�pid���� ����������
			}else	{
			
			}
		}
		else	{
			ClearPIDDiff(TempPid[HOLE_TEMP].PIDid);
			StopTempCtrl(&TempPid[HOLE_TEMP]);
		}
		OSTimeDly(80);
	}
}
