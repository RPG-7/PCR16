#include "app_spiflash.h"
#include "rw_spiflash.h"

//堆栈
__align(4) OS_STK  TASK_SPIFLASH_STK[STK_SIZE_SPIFLASH]; //任务堆栈声?
#define N_MESSAGES		5
#define LOG_BUF		512
_spiflash_t spiflash;
void    *SpiFlashMSG_Q[N_MESSAGES];//消息队列数组
static  message_pkt_t    msg_pkt_spiflash;
static void TaskSPIFLASH(void * ppdata);

void AppSpiFlashInit(void)
{
	OSTaskCreate(TaskSPIFLASH,  (void * )0, (OS_STK *)&TASK_SPIFLASH_STK[STK_SIZE_SPIFLASH-1], TASK_PRIO_SPIFLASH);
}

static void DataInit(void)
{
	spiflash.lock         =  OSSemCreate(1);
	spiflash.MSG_Q 			 = OSQCreate(&SpiFlashMSG_Q[0],N_MESSAGES);//
	LogInfor.pbuf = (char *)user_malloc(LOG_BUF);//日志缓存
}
//u8 devid[2];
void CheckSPIFlash(void)
{
	u8 devid[2];
	
	BSP_W25Qx_Read_ID(devid);
	if(devid[0] == W25QXX_MANUFACTURER_ID)	{//devid[1] -- DEVICE_ID; devid[0] -- MANUFACTURER_ID 
		BSP_PRINTF("SPI FLASH OK.");
		SysError.Y1.bits.b7 = DEF_Active;
	}else	{
		BSP_PRINTF("SPI FLASH error.");
		SysError.Y1.bits.b7 = DEF_Inactive;
	}
}
//通过任务写log
u8 WriteLogToBuff(char *str)
{
	char *pbuf;
	u8 len, log_len;
//	RTC_TimeTypeDef sTime = {0};
//	RTC_DateTypeDef sDate = {0};
	
	if(LogInfor.len>=LOG_BUF)
		return 0;
	mutex_lock(spiflash.lock);
	pbuf = LogInfor.pbuf + LogInfor.len;
//	bsp_rtc_get_time(&sTime, &sDate);
	len = sprintf(pbuf,"*%02u/%02u/%02u %02u:%02u:%02u  ",SysTime.tm_year,SysTime.tm_mon, SysTime.tm_mday, SysTime.tm_hour,SysTime.tm_min,SysTime.tm_sec);
	log_len = len + strlen(str)+2;
	if(log_len > ONELOG_SIZE)	{//单条日志长度太大 退出
		mutex_unlock(spiflash.lock);
		return 0;
	}
	sprintf(pbuf+ len, "%s\r\n", str);//加换行
	LogInfor.len += log_len;
	mutex_unlock(spiflash.lock);
	msg_pkt_spiflash.Src = MSG_WRITELOG;
	OSQPost(spiflash.MSG_Q, &msg_pkt_spiflash);
	return 1;
}
#include "json.h"
static void TaskSPIFLASH(void * ppdata)
{
	u8 err;
	message_pkt_t *msg;
	ppdata = ppdata;
	
	DataInit();
	CheckSPIFlash();
	if(SysError.Y1.bits.b7 == DEF_Active)	{
		if(FlashFSInit()==FR_OK)	{
			BSP_PRINTF("filesys init ok");		
			CreateSysFile();//创建系统文件
			ReadLabTemplateList();
			AnalysisCalibrateRes();//从文件解析校准结果
		}
	}
	OSFlagPost(SysFlagGrp, (OS_FLAGS)FLAG_GRP_2, OS_FLAG_SET, &err);
	jansson_pack_test();
	CreateLab_Jsonfile("1:/Tmp/test.json");
//	AnalysisLab_Jsonfile("1:/Tmp/test.json");
	for(;;)
	{
		msg = (message_pkt_t *)OSQPend(spiflash.MSG_Q, 0, &err);//
		if(err==OS_ERR_NONE)    {
			if(SysError.Y1.bits.b7 == DEF_Active)	{
				if(msg->Src == MSG_WRITELOG)	{//写日志
					write_log();
				}
				else if(msg->Src == MSG_WriteLabTemplate)	{//保存灌注数据
					WriteLabTemplate();
				}
				else if(msg->Src == MSG_WriteCaliRes)	{
					WriteCalibrateRes();
				}
			}
		}else if(err==OS_ERR_TIMEOUT)	{
			
		}
	}
}
