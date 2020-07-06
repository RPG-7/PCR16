#include "ad7124.h"
#include "PD_DataProcess.h"

#define	AD7124_REF_VOLTAGE		(3300) //参考电压 mV 3304
#define	DEFAULT_VDD						(float)(3.3)
_ad7124_t ad7124;

#define	AVER_MAX		15
#define	TEC_AVER_MAX		AVER_MAX
#define	PD_AVER_MAX		4
#define	TEC_DISCARD_NUM		4
#define	PD_DISCARD_NUM		1
#define	CH_AVERNUMS			6//需要算平均的通道个数
struct _AdcCodeAver_t {
	u32 buf[AVER_MAX];
	u8 idx;
}AdcCodeAver[CH_AVERNUMS];//adc 二进制数据 均值处理

static float CalcADCVoltage(u32 adcode);
static void AD7124ChannelEnable(void);
static void CalcADCVolAverage(u8 ch, u32 advol, u8 aver_max, u8 discard);

u8 ad7124_id;
void AD7124Init(void)
{
//	u8 ad7124_id;

	ad7124.status = AD7124_IDLE;
	ad7124.busy = DEF_Idle;
	ad7124.channel = uCH_0;
	ad7124.channel_last = uCH_0;
	
	AdcCodeAver[uCH_0].idx = 0;
	AdcCodeAver[uCH_1].idx = 0;
	AdcCodeAver[uCH_2].idx = 0;
	AdcCodeAver[uCH_3].idx = 0;
	
	ad7124.pdev = bsp_ad7124_init(AD7124_ID1);
	ad7124_id = bsp_ad7124_id_get(ad7124.pdev);
	if(ad7124_id == AD7124_ID)	{
		SysError.Y1.bits.b2 = DEF_Active;
	}else	{
		SysError.Y1.bits.b2 = DEF_Inactive;//AD7124异常
	}
	bsp_ad7124_filterreg_set(ad7124.pdev);
	bsp_ad7124_control_set(ad7124.pdev);//连续采样模式
	AD7124ChannelEnable();//通道配置
}
//u32 ad7124reg_r;
static void AD7124ChannelEnable(void)
{
	ad7124_chcfg_t g_chcfg_tbl1=\
	{uCH_0, DEF_Enable, CONFIG_0,  UNIPOLAR, 0, AIN0, AIN_AVSS, 1, AD7124_GAIN_1_MUL};//channel_0 配置寄存器0 单极性 内部增益64
	
	bsp_ad7124_cfg_set(ad7124.pdev, &g_chcfg_tbl1);//初始化配置寄存器0 使用内部参考电压 内部增益64 用于压力采集
//	ad7124reg_r = bsp_ad7124_cfg_get(ad7124.pdev, CONFIG_0);
	g_chcfg_tbl1.channel = uCH_0;
	g_chcfg_tbl1.config_idx = CONFIG_0;
	g_chcfg_tbl1.AINP = AIN0;
	g_chcfg_tbl1.AINM = AIN_AVSS;
	bsp_ad7124_channel_set(ad7124.pdev, &g_chcfg_tbl1);//channel_0使用配置寄存器0 配置

	g_chcfg_tbl1.channel = uCH_1;
	g_chcfg_tbl1.AINP = AIN1;
	g_chcfg_tbl1.AINM = AIN_AVSS;
	bsp_ad7124_channel_set(ad7124.pdev, &g_chcfg_tbl1);//channel_1使用配置寄存器0 配置
	g_chcfg_tbl1.channel = uCH_2;
	g_chcfg_tbl1.AINP = AIN2;
	g_chcfg_tbl1.AINM = AIN_AVSS;
	bsp_ad7124_channel_set(ad7124.pdev, &g_chcfg_tbl1);//channel_2使用配置寄存器0 配置
//	g_chcfg_tbl1.channel = uCH_3;
//	g_chcfg_tbl1.AINP = AIN3;
//	g_chcfg_tbl1.AINM = AIN_AVSS;
//	bsp_ad7124_channel_set(ad7124.pdev, &g_chcfg_tbl1);//channel_3使用配置寄存器0 配置	
	g_chcfg_tbl1.channel = uCH_4;
	g_chcfg_tbl1.AINP = AIN4;
	g_chcfg_tbl1.AINM = AIN_AVSS;
	bsp_ad7124_channel_set(ad7124.pdev, &g_chcfg_tbl1);//channel_3使用配置寄存器0 配置	
//	g_chcfg_tbl1.channel = uCH_5;
//	g_chcfg_tbl1.AINP = AIN5;
//	g_chcfg_tbl1.AINM = AIN_AVSS;
//	bsp_ad7124_channel_set(ad7124.pdev, &g_chcfg_tbl1);//channel_3使用配置寄存器0 配置	
	
	ad7124.channel = uCH_0;//起始通道
	ad7124.channel_last = uCH_5;//最后一个通道		
	ad7124.status = AD7124_MEASURE_TEMP;		
}
//打开/关闭测量PD的AD通道
void AD7124_PDChannelONOFF(u8 ch, u8 flag)
{
	ad7124_chcfg_t g_chcfg_tbl1=\
	{uCH_0, DEF_Enable, CONFIG_0,  UNIPOLAR, 0, AIN0, AIN_AVSS, 1, AD7124_GAIN_1_MUL};//channel_0 配置寄存器0 单极性 内部增益64
	
	g_chcfg_tbl1.enable = flag;
	g_chcfg_tbl1.channel = ch;
	g_chcfg_tbl1.config_idx = CONFIG_0;
	g_chcfg_tbl1.AINP = ch;
	g_chcfg_tbl1.AINM = AIN_AVSS;
	bsp_ad7124_channel_set(ad7124.pdev, &g_chcfg_tbl1);//channel_0使用配置寄存器0 配置
}
u8 r_channel;
u8 ad7124_err;
float ad_temp;
u32 ad_code;
//u32 calc_start=0,calc_time;
u8 StartADDataCollect(void)
{
//	u8 r_channel;
//	u8 ad7124_err;
//	float ad_temp;
//	u32 ad_code;
//	u8 delaycnt=0;
	
//	ad7124_cs_low();
//	while(AD7124_DATA_READY())	{//RDY引脚为低时 表示转换完成
//		delaycnt++;
//		if(delaycnt>10)	{
//			ad7124_cs_high();
//			return 0;
//		}
//	}
	r_channel = bsp_ad7124_conv_ready(ad7124.pdev, &ad7124_err);//回读当前采样通道
	if(ad7124.channel == r_channel)	{
		return 0;
	}
	ad7124.channel = r_channel;
//	HAL_GPIO_WritePin(GPIOC, Fluo_Green_Pin,GPIO_PIN_SET);
	if(ad7124_err==AD7124_ERR_NONE)	{
		if(r_channel <= ad7124.channel_last)	
		{
			ad_code = bsp_ad7124_value_get(ad7124.pdev); //读取ADC转换结果				
//			ad_temp = CalcADCVoltage(ad_code); //计算电压			
			switch(r_channel)	{
				case uCH_0:	//测温通道 精确到mv
					CalcADCVolAverage(uCH_0, ad_code, TEC_AVER_MAX, TEC_DISCARD_NUM);
					break;
				case uCH_1:	//测温通道 精确到mv
					CalcADCVolAverage(uCH_1, ad_code, TEC_AVER_MAX, TEC_DISCARD_NUM);
					break;
				case uCH_2://测温通道 精确到mv	 
					CalcADCVolAverage(uCH_2, ad_code, TEC_AVER_MAX, TEC_DISCARD_NUM);				
					break;
				case uCH_3:	//测温通道 精确到mv
					CalcADCVolAverage(uCH_3, ad_code, TEC_AVER_MAX, TEC_DISCARD_NUM);
					break;
				case uCH_4:	//PD通道
//					CalcADCVolAverage(uCH_4, ad_code, PD_AVER_MAX, PD_DISCARD_NUM);
					ad_temp = CalcADCVoltage(ad_code); //计算电压	
					PD_DataCollect(ad_temp, LED_BLUE);
					break;
				case uCH_5:	//PD通道
//					CalcADCVolAverage(uCH_5, ad_code, PD_AVER_MAX, PD_DISCARD_NUM);
					break;
				default:
					break;
			}
		}
	}
//	HAL_GPIO_WritePin(GPIOC, Fluo_Green_Pin,GPIO_PIN_RESET);
	return 1;
}

u32 GetADCVol(u8 ch)
{
	return ad7124.vol[ch];
}

//返回电压值mV
static float CalcADCVoltage(u32 adcode)
{
	float adtemp;
	
//	adtemp = (adcode*1.0/0x800000) - 1;//双极性电压计算公式
//	return (adtemp * AD7124_REF_VOLTAGE);
	adtemp = (adcode*1.0)/0x1000000;//单极性电压计算公式
	return adtemp*AD7124_REF_VOLTAGE;
}

//计算每个通道电压均值（先计算adc二进制数据均值） 去掉1个最大值 1个最小值 取平均
static void CalcADCVolAverage(u8 ch, u32 adcode, u8 aver_max, u8 discard)
{	
	u8 idx;
	
	idx = AdcCodeAver[ch].idx;
	AdcCodeAver[ch].buf[idx] = adcode;
	AdcCodeAver[ch].idx ++;
	if(AdcCodeAver[ch].idx >= aver_max)	{
		u32 temp;
		 /*---------------- 冒泡排序,由小到大排序 -----------------*/
    	s8 i,j,flag;
		i=0;
		idx = AdcCodeAver[ch].idx;

		do{
			flag=0;
			for (j=0;j<idx-i-1;j++)
			{
				if (AdcCodeAver[ch].buf[j] > AdcCodeAver[ch].buf[j+1])
				{
					temp = AdcCodeAver[ch].buf[j];
					AdcCodeAver[ch].buf[j]   = AdcCodeAver[ch].buf[j+1];
					AdcCodeAver[ch].buf[j+1] = temp;
					flag = 1;
				}
			}
			i++;
		}while ((i<idx) && flag);
		/*---------------- end -----------------*/
		/*----------- 去掉1个最大值和最小值,记录最大偏差，数据进行平均 --------------*/
		temp = 0;
		j = idx - discard;
		for (i=discard;i<j;i++)
		{
			temp += AdcCodeAver[ch].buf[i];
		}
		j = idx - discard*2;
		temp /= j;
		/*---------------- end -----------------*/
		ad7124.vol[ch] = CalcADCVoltage(temp)*100; //计算电压 0.01mv
		AdcCodeAver[ch].idx = 0;
	}
}

