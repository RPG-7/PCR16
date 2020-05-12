#include "PD_DataProcess.h"

_pd_data_t gPD_Data;

void PD_DataInit(void)
{
	gPD_Data.coll_enable = DEF_False;
	gPD_Data.ch = LED_NONE;
	memset(&gPD_Data.templatehole, 0, sizeof(gPD_Data.templatehole));
}

//�����տ�PDֵ�ɼ� ʹ������LED
void StartCollTemplateHolePD(void)
{	
	gPD_Data.ch = LED_BLUE;
	FluoLED_OnOff(LED_BLUE, DEF_ON);
	Sys.state |= SysState_CaliTemplateHoleFluo;//У׼�տ�PDֵ
	memset(&gPD_Data.templatehole, 0, sizeof(gPD_Data.templatehole));
	gPD_Data.templatehole.min[0] = 0xffff;
	gPD_Data.templatehole.min[1] = 0xffff;
	gPD_Data.coll_enable = DEF_False;
}
void StopCollTemplateHolePD(void)
{	
	Sys.state &= ~SysState_CaliTemplateHoleFluo;//У׼����
	gPD_Data.coll_enable = DEF_False;
	FluoLED_OnOff(LED_BLUE, DEF_OFF);	
}

//����տ�PD��ֵ
void CalcTemplateHolePDAver(void)
{
	u32 temp;
	
	temp = gPD_Data.templatehole.min[0] + gPD_Data.templatehole.min[1] + gPD_Data.templatehole.max[0] + gPD_Data.templatehole.max[1];
	gPD_Data.templatehole.aver = temp/4;//����տ�PD��ֵ
	temp = gPD_Data.templatehole.aver*15;//������Ч��λ��PD��ֵ
	gPD_Data.HoleThreshold = temp/10;
}

#include "motor.h"
u8 HolePositionCaliFlag;
//PD���ݲɼ�
void PD_DataCollect(u16 ad_vol, u8 pd_ch)
{
	u8 idx;
	if(gPD_Data.coll_enable==DEF_False)//�Ƿ�ɼ�����
		return;
	if(pd_ch != gPD_Data.ch)
		return;
	if(Sys.state & SysState_CaliTemplateHoleFluo)	{//����տ�PD���ֵ ��Сֵ		
		idx = gPD_Data.templatehole.idx;
		if(ad_vol>gPD_Data.templatehole.max[idx])	//���ֵ
			gPD_Data.templatehole.max[idx] = ad_vol;
		else if(ad_vol<gPD_Data.templatehole.min[idx])//��Сֵ
			gPD_Data.templatehole.min[idx] = ad_vol;
	}
	else if(Sys.state & SysState_CaliHolePostion)	{//��λ��У׼
		idx = HolePos.idx;
		if(ad_vol < gPD_Data.HoleThreshold)	{
			if(HolePositionCaliFlag == 0)	{
				HolePositionCaliFlag = 1;
			}
			else if(HolePositionCaliFlag == 2)	{
				HolePos.pos[idx].x2 = tMotor[MOTOR_ID1].CurSteps;
				HolePositionCaliFlag = 0;
				HolePos.idx ++;
			}
		}
		else if(HolePositionCaliFlag == 1)	{				
			HolePos.pos[idx].x1 = tMotor[MOTOR_ID1].CurSteps;
			HolePositionCaliFlag = 2;			
		}
	}
}

