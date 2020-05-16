#include "PD_DataProcess.h"
#include "motor.h"

#define BUFSIZE         64
#define	HOLE_POSITION_MIN_OFFSET	(u32)(1*Motor_StepsPerum)	//
#define	HOLE_POSITION_MAX_OFFSET	(u32)(1*Motor_StepsPerum)	//
#define	HOLE_POSITION_DISTANCE	(u32)(4.5*Motor_StepsPerum)	//

_pd_data_t gPD_Data;
u16 PD_max;

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
	memset(gPD_Data.PDBase, 0, sizeof(gPD_Data.PDBase));
	p_pdbuf = &pdbuf[0];
	p_pdbuf->idx = 0;
	memset(gPD_Data.PDVol, 0, HOLE_NUM);
	PD_max = 0x00;
}

//������λ��У׼ ʹ������LEDɨ��
void StartCaliHolePosition(void)
{
	gPD_Data.ch = LED_BLUE;
	FluoLED_OnOff(LED_BLUE, DEF_ON);
	PD_max = 0;	
	Sys.state |= SysState_CaliHolePostion;//��λ��У׼
	gPD_Data.coll_enable = DEF_True;
}
void StopCaliHolePosition(void)
{	
	Sys.state &= ~SysState_CaliHolePostion;//У׼����
	gPD_Data.coll_enable = DEF_False;
	gPD_Data.ch = LED_NONE;
	FluoLED_OnOff(LED_BLUE, DEF_OFF);
}
//�����λ�� ��֪��һ����λ�� ������λ�ð��տ׼��4.5mm����
void CalcHolePositon(void)
{
	u8 i;
	
	HolePos.pos[0].x1 -= HOLE_POSITION_MIN_OFFSET;
	HolePos.pos[0].x2 =  HolePos.pos[0].x1 + HOLE_POSITION_MAX_OFFSET;
	for(i=1;i<HOLE_NUM;i++)	{
		HolePos.pos[i].x1 = HolePos.pos[i-1].x1 + HOLE_POSITION_DISTANCE;
		HolePos.pos[i].x2 =  HolePos.pos[i-1].x2 + HOLE_POSITION_DISTANCE;
	}
}

//�����տ�PD�����ź�У׼ ʹ������LEDɨ��
void StartCaliHolePDBase(void)
{	
	gPD_Data.ch = LED_BLUE;
	FluoLED_OnOff(LED_BLUE, DEF_ON);
	memset(gPD_Data.PDBase, 0, sizeof(gPD_Data.PDBase));
	StartCollPDData();//��PD���ݲɼ�׼��
}
void StopCaliHolePDBase(void)
{	
	StopCollPDData();
}
//����ױ����ź�
void CalcHolePDBase(void)
{
	memcpy(gPD_Data.PDBase, gPD_Data.PDVol, HOLE_NUM);
}

//PD���ݲɼ�
void PD_DataCollect(u16 ad_vol, u8 pd_ch)
{	
	if(gPD_Data.coll_enable==DEF_False)//�Ƿ�ɼ�����
		return;
	if(pd_ch != gPD_Data.ch)
		return;
	
	if(Sys.state & SysState_CollHolePD)	{//�ɼ���PD��Чֵ
		if(p_pdbuf->idx<BUFSIZE)	{
			p_pdbuf->buf[p_pdbuf->idx] = ad_vol;
			p_pdbuf->idx++;
		}
	}
	else if(Sys.state & SysState_CaliHolePostion)	{//��λ��У׼ �ҵ���һ����λ�� ������λ�ð��տ׼��4.5mm����
		if(ad_vol>PD_max)	{	//�ҵ���һ���׵����ֵ 
			PD_max = ad_vol;
			HolePos.pos[0].x1 = tMotor[MOTOR_ID1].CurSteps;
		}
	}
}
//׼���ɼ�ʵ��PD���� ����˫����
void StartCollPDData(void)
{
	p_pdbuf = &pdbuf[0];
	p_pdbuf->idx = 0;
	Sys.state |= SysState_CollHolePD;
	gPD_Data.coll_enable = DEF_False;
}
void StopCollPDData(void)
{
	Sys.state &= ~SysState_CollHolePD;//�ɼ�����
	gPD_Data.coll_enable = DEF_False;
	gPD_Data.ch = LED_NONE;
	FluoLED_OnOff(LED_BLUE|LED_GREEN, DEF_OFF);
	
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
	gPD_Data.PDVol[hole_idx] = (u16)temp;//��ǰ��PD��Чֵ
}

