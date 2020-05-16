#include "PD_DataProcess.h"

#define BUFSIZE         64

_pd_data_t gPD_Data;
_pd_maxmin_t templatehole;

typedef struct _pd_buf	{
	u8 idx;
	u16 buf[BUFSIZE];
}_pd_buf_t;

_pd_buf_t pdbuf[2];
_pd_buf_t *p_pdbuf,*p_pdbuf_bk;


void PD_DataInit(void)
{
	gPD_Data.coll_enable = DEF_False;
	gPD_Data.ch = LED_NONE;
	memset(&templatehole, 0, sizeof(templatehole));
	p_pdbuf = &pdbuf[0];
	p_pdbuf->idx = 0;
	memset(gPD_Data.PDVol, 0, HOLE_NUM);
}

//�����տ�PDֵ�ɼ� ʹ������LEDɨ��
void StartCollTemplateHolePD(void)
{	
	gPD_Data.ch = LED_BLUE;
	FluoLED_OnOff(LED_BLUE, DEF_ON);
	Sys.state |= SysState_CollTemplateHolePD;//У׼�տ�PDֵ
	memset(&templatehole, 0, sizeof(templatehole));
	templatehole.min[0] = 0xffff;
	templatehole.min[1] = 0xffff;
	gPD_Data.coll_enable = DEF_False;
}
void StopCollTemplateHolePD(void)
{	
	Sys.state &= ~SysState_CollTemplateHolePD;//У׼����
	gPD_Data.coll_enable = DEF_False;
	FluoLED_OnOff(LED_BLUE, DEF_OFF);	
}

//����տ�PD��ֵ
void CalcTemplateHolePDAver(void)
{
	u32 temp;
	
	temp = templatehole.min[0] + templatehole.min[1] + templatehole.max[0] + templatehole.max[1];
	templatehole.aver = temp/4;//����տ�PD��ֵ
	temp = templatehole.aver*15;//������Ч��λ��PD��ֵ
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
	if(Sys.state & SysState_CollHolePD)	{//�ɼ���PDֵ
		if(p_pdbuf->idx<BUFSIZE)	{
			p_pdbuf->buf[p_pdbuf->idx] = ad_vol;
			p_pdbuf->idx++;
		}
	}
	else if(Sys.state & SysState_CollTemplateHolePD)	{//�ɼ��տ�PD���ֵ ��Сֵ		
		idx = templatehole.idx;
		if(ad_vol>templatehole.max[idx])	//���ֵ
			templatehole.max[idx] = ad_vol;
		else if(ad_vol<templatehole.min[idx])//��Сֵ
			templatehole.min[idx] = ad_vol;
	}
	else if(Sys.state & SysState_CaliHolePostion)	{//��λ��У׼
		idx = HolePos.idx;
//		if(ad_vol < gPD_Data.HoleThreshold)	{
		if(ad_vol <= templatehole.aver)	{
			if(HolePositionCaliFlag == 0)	{
				HolePositionCaliFlag = 1;
			}
			else if(HolePositionCaliFlag == 2)	{
				HolePos.pos[idx].x2 = tMotor[MOTOR_ID1].CurSteps;
				HolePositionCaliFlag = 0;
				HolePos.idx ++;
			}
		}
		else if(ad_vol >= gPD_Data.HoleThreshold)	{
			if(HolePositionCaliFlag == 1)	{
				HolePos.pos[idx].x1 = tMotor[MOTOR_ID1].CurSteps;
				HolePositionCaliFlag = 2;	
			}				
		}
	}
}
//׼���ɼ�ʵ��PD���� ����˫����
void ReadyToCollPD_LabData(void)
{
	p_pdbuf = &pdbuf[0];
	p_pdbuf->idx = 0;
//	memset(gPD_Data.PDVol, 0, HOLE_NUM);
}
//���������ַ
void ExchangePDBuf(void)
{
	p_pdbuf_bk = p_pdbuf;
	if(p_pdbuf == &pdbuf[0])
		p_pdbuf = &pdbuf[1];
	else
		p_pdbuf = &pdbuf[0];
	p_pdbuf->idx = 0;
}
//���ݲɼ�����PDֵ �����PD��ֵ
#define PDMaxFont		5
#define PDMaxBack		5
void CalcPDData(u8 hole_idx)
{
	u8 i,max_idx,m,n;
	u16 max;
	u32 temp;
	
	max = p_pdbuf_bk->buf[0];
	for(i=0;i<p_pdbuf_bk->idx;i++)//���ҵ����ֵ �ٽ����ֵǰ5�� ��5��ȥƽ�� �õ�PD��ֵ
    {
        if(p_pdbuf_bk->buf[i] > max)
        {
            max = p_pdbuf_bk->buf[i];
            max_idx = i;
        }
	}	
	n = max_idx + PDMaxBack;
	m = max_idx - PDMaxFont;
	temp = 0;
	for(i=m;i<n;i++)	{
		temp += p_pdbuf_bk->buf[i];
	}
	temp /= (PDMaxFont + PDMaxBack);
	gPD_Data.PDVol[hole_idx] = (u16)temp;//��ǰ��PDֵ
}

