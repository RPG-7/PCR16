#include "ad7124.h"
//#include "TempCtrl.h"

#define AD7124_DATA_READY()		HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11)
#define	AD7124_REF_VOLTAGE		(2500) //�ο���ѹ mV 3304
#define	DEFAULT_VDD						(float)(3.3)
_ad7124_t ad7124;

#define	AVER_MAX		3
#define	DISCARD_NUM		1
#define	CH_AVERNUMS			4//��Ҫ��ƽ����ͨ������
struct _AdcVolAver_t {
	u16 buf[AVER_MAX];
	u8 idx;
//	u16 aver;
}AdcVolAver[CH_AVERNUMS];

static float CalcADCVoltage(u32 adcode);
static void AD7124ChannelEnable(void);
static void CalcADCVolAverage(u8 ch, float vol);

u8 ad7124_id;
void AD7124Init(void)
{
//	u8 ad7124_id;

	ad7124.status = AD7124_IDLE;
	ad7124.busy = DEF_Idle;
	ad7124.channel = uCH_0;
	ad7124.channel_last = uCH_0;
	
	AdcVolAver[uCH_0].idx = 0;
	AdcVolAver[uCH_1].idx = 0;
	AdcVolAver[uCH_2].idx = 0;
	AdcVolAver[uCH_3].idx = 0;
	
	ad7124.pdev = bsp_ad7124_init(AD7124_ID1);
	ad7124_id = bsp_ad7124_id_get(ad7124.pdev);
	if(ad7124_id == AD7124_ID)	{
		SysError.Y2.bits.b2 = DEF_Active;
	}else	{
		SysError.Y2.bits.b2 = DEF_Inactive;//AD7124�쳣
	}
	bsp_ad7124_filterreg_set(ad7124.pdev);
	bsp_ad7124_control_set(ad7124.pdev);//��������ģʽ
	AD7124ChannelEnable();//ͨ������
}
//u32 ad7124reg_r;
static void AD7124ChannelEnable(void)
{
	ad7124_chcfg_t g_chcfg_tbl1=\
	{uCH_0, DEF_Enable, CONFIG_0,  UNIPOLAR, 0, AIN0, AIN_AVSS, 1, AD7124_GAIN_1_MUL};//channel_0 ���üĴ���0 ������ �ڲ�����64
	
	bsp_ad7124_cfg_set(ad7124.pdev, &g_chcfg_tbl1);//��ʼ�����üĴ���0 ʹ���ڲ��ο���ѹ �ڲ�����64 ����ѹ���ɼ�
//	ad7124reg_r = bsp_ad7124_cfg_get(ad7124.pdev, CONFIG_0);
	g_chcfg_tbl1.channel = uCH_0;
	g_chcfg_tbl1.config_idx = CONFIG_0;
	g_chcfg_tbl1.AINP = AIN0;
	g_chcfg_tbl1.AINM = AIN_AVSS;
	bsp_ad7124_channel_set(ad7124.pdev, &g_chcfg_tbl1);//channel_0ʹ�����üĴ���0 ����

	g_chcfg_tbl1.channel = uCH_1;
	g_chcfg_tbl1.AINP = AIN1;
	g_chcfg_tbl1.AINM = AIN_AVSS;
	bsp_ad7124_channel_set(ad7124.pdev, &g_chcfg_tbl1);//channel_1ʹ�����üĴ���0 ����
	g_chcfg_tbl1.channel = uCH_2;
	g_chcfg_tbl1.AINP = AIN2;
	g_chcfg_tbl1.AINM = AIN_AVSS;
	bsp_ad7124_channel_set(ad7124.pdev, &g_chcfg_tbl1);//channel_2ʹ�����üĴ���0 ����
	g_chcfg_tbl1.channel = uCH_3;
	g_chcfg_tbl1.AINP = AIN3;
	g_chcfg_tbl1.AINM = AIN_AVSS;
	bsp_ad7124_channel_set(ad7124.pdev, &g_chcfg_tbl1);//channel_3ʹ�����üĴ���0 ����	
	g_chcfg_tbl1.channel = uCH_4;
	g_chcfg_tbl1.AINP = AIN4;
	g_chcfg_tbl1.AINM = AIN_AVSS;
	bsp_ad7124_channel_set(ad7124.pdev, &g_chcfg_tbl1);//channel_3ʹ�����üĴ���0 ����	
	g_chcfg_tbl1.channel = uCH_5;
	g_chcfg_tbl1.AINP = AIN5;
	g_chcfg_tbl1.AINM = AIN_AVSS;
	bsp_ad7124_channel_set(ad7124.pdev, &g_chcfg_tbl1);//channel_3ʹ�����üĴ���0 ����	
	
	ad7124.channel = uCH_0;//��ʼͨ��
	ad7124.channel_last = uCH_5;//���һ��ͨ��		
	ad7124.status = AD7124_MEASURE_TEMP;		
}
u8 r_channel;
u8 ad7124_err;
float ad_temp;
u32 ad_code;
u32 calc_start=0,calc_time;
u8 StartADDataCollect(void)
{
//	u8 r_channel;
//	u8 ad7124_err;
//	float ad_temp;
//	u32 ad_code;
	u8 delaycnt=0;
	
	ad7124_cs_low();
	while(AD7124_DATA_READY())	{//RDY����Ϊ��ʱ ��ʾת�����
		delaycnt++;
		if(delaycnt>10)	{
			ad7124_cs_high();
			return 0;
		}
	}
	r_channel = bsp_ad7124_conv_ready(ad7124.pdev, &ad7124_err);//�ض���ǰ����ͨ��
	if(ad7124_err==AD7124_ERR_NONE)	{
		if(r_channel == ad7124.channel)	{
			ad_code = bsp_ad7124_value_get(ad7124.pdev); //��ȡADCת�����				
			ad_temp = CalcADCVoltage(ad_code); //�����ѹ			
//			calc_time = HAL_GetTick() - calc_start;
//			calc_start = HAL_GetTick();
			switch(r_channel)	{
				case uCH_0:	//����ͨ�� ��ȷ��mv
					CalcADCVolAverage(uCH_0, ad_temp);
					break;
				case uCH_1:	//����ͨ�� ��ȷ��mv
					CalcADCVolAverage(uCH_1, ad_temp);
					break;
				case uCH_2://����ͨ�� ��ȷ��mv	 
					CalcADCVolAverage(uCH_2, ad_temp);				
					break;
				case uCH_3:	//����ͨ�� ��ȷ��mv
					CalcADCVolAverage(uCH_3, ad_temp);
					break;
				case uCH_4:	
					ad7124.vol[uCH_4] = (u16)ad_temp;
					break;
				case uCH_5:	
					ad7124.vol[uCH_5] = (u16)ad_temp;
					break;
			}
			ad7124.channel ++;
			if(ad7124.channel > ad7124.channel_last)	{
				ad7124.channel = uCH_0;
				ad7124.busy = DEF_Idle;
			}
		}
		else if(r_channel > ad7124.channel_last)	{
			ad7124.channel = uCH_0;
			ad7124.busy = DEF_Idle;
		}
	}
	return 1;
}

u16 GetADCVol(u8 ch)
{
	return ad7124.vol[ch];
}

//���ص�ѹֵmV
static float CalcADCVoltage(u32 adcode)
{
	float adtemp;
	
//	adtemp = (adcode*1.0/0x800000) - 1;//˫���Ե�ѹ���㹫ʽ
//	return (adtemp * AD7124_REF_VOLTAGE);
	adtemp = (adcode*1.0)/0x1000000;//�����Ե�ѹ���㹫ʽ
	return adtemp*AD7124_REF_VOLTAGE;
}

//����ÿ��ͨ����ѹ��ֵ ȥ��1�����ֵ 1����Сֵ ȡƽ��
static void CalcADCVolAverage(u8 ch, float vol)
{	
	u16 advol;
	u8 idx;
	
	advol = (u16)vol;
	idx = AdcVolAver[ch].idx;
	AdcVolAver[ch].buf[idx] = advol;
	AdcVolAver[ch].idx ++;
	if(AdcVolAver[ch].idx >= AVER_MAX)	{
		u32 temp;
		 /*---------------- ð������,��С�������� -----------------*/
    	u8 i,j,flag;
		i=0;

		do{
			flag=0;
			for (j=0;j<idx-i-1;j++)
			{
				if (AdcVolAver[ch].buf[j] > AdcVolAver[ch].buf[j+1])
				{
					temp = AdcVolAver[ch].buf[j];
					AdcVolAver[ch].buf[j]   = AdcVolAver[ch].buf[j+1];
					AdcVolAver[ch].buf[j+1] = temp;
					flag = 1;
				}
				if (AdcVolAver[ch].buf[j] > AdcVolAver[ch].buf[j+1])
				{
					temp = AdcVolAver[ch].buf[j];
					AdcVolAver[ch].buf[j]   = AdcVolAver[ch].buf[j+1];
					AdcVolAver[ch].buf[j+1] = temp;
					flag = 1;
				}
			}
			i++;
		}while ((i<AdcVolAver[ch].idx) && flag);
		/*---------------- end -----------------*/
		/*----------- ȥ��1�����ֵ����Сֵ,��¼���ƫ����ݽ���ƽ�� --------------*/
		temp = 0;
		j = AdcVolAver[ch].idx - DISCARD_NUM;
		for (i=DISCARD_NUM;i<j;i++)
		{
			temp += AdcVolAver[ch].buf[i];
		}
		j = AdcVolAver[ch].idx - DISCARD_NUM*2;
		ad7124.vol[ch] = temp/j;
//		*paver = temp/j;
		/*---------------- end -----------------*/
		AdcVolAver[ch].idx = 0;
//		return 1;
	}
//	return 0;
}

