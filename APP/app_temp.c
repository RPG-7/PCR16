#include "app_temp.h"
#include "ad7124.h"
#include "PID.h"

//��ջ
__align(4) OS_STK  TASK_TEMP_STK[STK_SIZE_TEMP]; //�����ջ��?

_app_temp_t app_temp;

static  message_pkt_t    msg_pkt_temp;
static void AppTempTask (void *parg);

void AppTempInit (void)
{
	OSTaskCreate(AppTempTask,  (void * )0, (OS_STK *)&TASK_TEMP_STK[STK_SIZE_TEMP-1], TASK_PRIO_TEMP);
}

static void TempDatInit(void)
{
//	app_temp.pTemp = &tempctrl;
	app_temp.target_t[HOLE_TEMP] = 3700;//0.01
	app_temp.target_t[COVER_TEMP] = 0;
	app_temp.PIDParam[HOLE_TEMP] = 0.0;
	app_temp.PIDParam[COVER_TEMP] = 0.0;
	app_temp.pTECPWM = &htim8;
}

#define	TECPWM_CH		TIM_CHANNEL_1
#define	TECPWM_PLUSE		400
static void StopTECPWM(void)
{
	HAL_TIM_PWM_Stop(app_temp.pTECPWM, TECPWM_CH);
}
//�޸�TEC pwmռ�ձ�
static void UpdateTECPWM(INT16U duty)
{
	u16 temp;
	
	temp = (TECPWM_PLUSE/100)*duty;
    __HAL_TIM_SET_AUTORELOAD(app_temp.pTECPWM, TECPWM_PLUSE);
    __HAL_TIM_SET_COMPARE(app_temp.pTECPWM, TECPWM_CH, temp);
}

static void StartTECPWM(u16 duty)
{
	static u16 dutybk;
	
	if(dutybk==duty)
		return;
	if(duty==100)	{
		duty++;
	}
	__HAL_TIM_CLEAR_FLAG(app_temp.pTECPWM, TIM_FLAG_UPDATE);
	UpdateTECPWM(duty);
	HAL_TIM_PWM_Start(app_temp.pTECPWM, TECPWM_CH);	
	dutybk = duty;
}
//ֹͣ�¶ȿ���
static u32 StopTempCtrl(void)
{
	StopTECPWM();
	return 0;
}
u16 setval;
//pid���ڰ뵼��Ƭ�¶� �������������� pwmռ�ձȲ��ܳ���50%
static float TempCtrl(u8 id, float pid_param, u16 cur_t)
{
	s16 dat;
	float temp;
//	u16 setval;
	
	temp = pid_param;
	temp += PID_control(id, dat, cur_t);
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
	StartTECPWM(setval);
//	SYS_PRINTF("D:%d,T:%d ",dat,cur_t);
	return temp;
}
float Kpbk;
#define	TEMPCTRL_ACCURACY		10//�¿ؾ���0.2
#define	TEMPCOLLECT_ACCURACY		5//�¶Ȳɼ����� 0.05
static void AppTempTask (void *parg)
{
	s32 cur_temp;
	u16 diff;
	
	TempDatInit();
	PIDParamInit();
	app_temp.PIDParam[HOLE_TEMP] = StopTempCtrl();
	COOLFAN_ON();
//	SetPIDVal(PID_ID1, 1, 0, 0);
	for(;;)
    {
		if(Sys.devstate == DevState_Running)	
		{
			if(CalcTemperature(GetADCVol(TEMP_ID1), &cur_temp)==0)	{
//				diff = abs(app_temp.current_t[TEMP_ID1] - cur_temp);
//				if(diff>TEMPCOLLECT_ACCURACY)	
				{
					app_temp.current_t[TEMP_ID1] = cur_temp;//0.01
					SetPIDTarget(PID_ID1, app_temp.target_t[HOLE_TEMP]);//���ÿ���Ŀ��
					diff = abs(app_temp.target_t[HOLE_TEMP] - cur_temp);//��ȡPID��ǰ���
//					if(diff > 200)	{//�����¶Ȳ� ����pid����
//						SetPIDVal(PID_ID1, 1, 0, 0);
//					}
//					else 
//						SetPIDVal(PID_ID1, 1, 0, 0);
					if(cur_temp > (app_temp.target_t[HOLE_TEMP]+1000))	{
						app_temp.PIDParam[HOLE_TEMP] = StopTempCtrl();
						return;
					}
					if(diff > TEMPCOLLECT_ACCURACY)	
					{//����0.1�� 
						if(PID[PID_ID1].Kp != Kpbk)		{
							Kpbk = PID[PID_ID1].Kp;
							SetPIDVal(PID_ID1, PID[PID_ID1].Kp,PID[PID_ID1].Ki,PID[PID_ID1].Kd);
						}
						app_temp.PIDParam[HOLE_TEMP] = TempCtrl(PID_ID1, app_temp.PIDParam[HOLE_TEMP], cur_temp);//pid���� ����������
					}
					else	{//С<=0.1�� ������ TECֹͣ����
//						StartTECPWM(1);
						ClearPIDDiff(PID_ID1);
						app_temp.PIDParam[HOLE_TEMP] = StopTempCtrl();				
					}
				}
			}else	{//�¶ȴ���������
			
			}
			if(CalcTemperature(GetADCVol(TEMP_ID2), (s32 *)&cur_temp)==0)	{
				app_temp.current_t[TEMP_ID2] = cur_temp;
			}else	{
			
			}
			if(CalcTemperature(GetADCVol(TEMP_ID3), (s32 *)&cur_temp)==0)	{
				app_temp.current_t[TEMP_ID3] = cur_temp;
			}else	{
			
			}
			if(CalcTemperature(GetADCVol(TEMP_ID4), (s32 *)&cur_temp)==0)	{
				app_temp.current_t[TEMP_ID4] = cur_temp;
			}else	{
			
			}
		}
		else	{
			ClearPIDDiff(PID_ID1);
			app_temp.PIDParam[HOLE_TEMP] = StopTempCtrl();
		}
		OSTimeDly(80);
	}
}
