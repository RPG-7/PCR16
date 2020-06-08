#include "app_udisk.h"
#include "rw_udisk.h"
#include "bsp.h"
#include "app_display.h"
#include "sys_info.h"
//��ջ
__align(4) OS_STK  TASK_UDISK_STK[STK_SIZE_UDISK]; //�����ջ��?
//#define N_MESSAGES		5
static u8 data_buf[50];
_appudisk_t appudisk;
//static void    *MyArrayOfMsg[N_MESSAGES];//��Ϣ��������
static  message_pkt_t    msg_pkt_udisk;
static void TaskUDISK(void * ppdata);
extern USBH_HandleTypeDef  hUSB_Host;
void CheckUpdateFWName(void);
#define	UPDATE_FW_NAME	"PCR16_UpdateFW.bin"
#define	UPDATE_FW_ATTR_SIZE		0X10
#define	UPDATE_FW_FILE_MINSIZE		0X400

void AppUSBInit(void)
{
	OSTaskCreate(TaskUDISK,  (void * )0, (OS_STK *)&TASK_UDISK_STK[STK_SIZE_UDISK-1], TASK_PRIO_UDISK);
}

static void DataInit(void)
{
	appudisk.Mbox 			 = OSMboxCreate((void *)0);
}
MSC_HandleTypeDef *userMSC_Handle;
//UDISK����
static void TaskUDISK(void * ppdata)
{
	u8 err;
	message_pkt_t *msg;
	ppdata = ppdata;
	
	DataInit();
//	OSFlagPend(SysFlagGrp, (OS_FLAGS)FLAG_GRP_3, OS_FLAG_WAIT_SET_ALL, 8000, &err);	
	MX_USB_HOST_Init();//udisk ����������ſ��Գ�ʼ��u�̣���ԱU�̲��Ͽ���ʱ��ⲻ��
	
	for(;;)
	{
		msg = (message_pkt_t *)OSMboxPend(appudisk.Mbox, 0, &err);
		if(err==OS_ERR_NONE)    {
			if(msg->Src == MSG_USB_START || msg->Src == MSG_USB_READY || msg->Src == MSG_USB_DISCONNECT)	{
				USBH_Process(&hUsbHostFS);
				if(hUsbHostFS.gState != HOST_IDLE)	{
					userMSC_Handle =  (MSC_HandleTypeDef *) hUsbHostFS.pActiveClass->pData;
					//if(userMSC_Handle->state != MSC_IDLE)	
					{
						OSTimeDly(10);
						msg_pkt_udisk.Src = MSG_USB_START;
						OSMboxPost(appudisk.Mbox, &msg_pkt_udisk);		
					}
				}
				if(msg->Src == MSG_USB_READY||msg->Src == MSG_USB_DISCONNECT)	{
					MountUDISK(msg->Src);
					if(udiskfs.flag & UDISKFLAG_MOUNTED && \
						(Sys.devstate == DevState_IDLE||Sys.devstate == DevState_Error))	{//�豸���л��������� ����Ƿ��������̼�
						CheckUpdateFWName();
					}
				}
			}
		}
  }
}
#define		ApplicationAddress			0x0800F000
#define		ApplicationJump				0x200
void CheckUpdateFWName(void)
{
	FRESULT res;
	char file_name[FILE_NAME_LEN];
	char FW_attr[UPDATE_FW_ATTR_SIZE];
	char FW_ver[UPDATE_FW_ATTR_SIZE];
	u32 rsize;
	
	sprintf(file_name, "%s%s", USBHPath, UPDATE_FW_NAME);//�̼�·��
	res = f_open(&udiskfs.fil, file_name, FA_READ);
	if(res==FR_OK)	{
		if(f_size(&udiskfs.fil)<UPDATE_FW_FILE_MINSIZE)	{//�̼�̫С �ж�Ϊ������̼�
			f_close(&udiskfs.fil);
			return;
		}
		f_read(&udiskfs.fil, FW_attr, UPDATE_FW_ATTR_SIZE, &rsize);
		if(0==strcmp(FW_attr, CONFIG_SYSINFO_Manufacturer))  {//У�鹫˾����
			f_read(&udiskfs.fil, FW_attr, UPDATE_FW_ATTR_SIZE, &rsize);
			if(0==strcmp(FW_attr, CONFIG_SYSINFO_Name))        {//У���豸����
				f_lseek(&udiskfs.fil, ApplicationJump);
				f_read(&udiskfs.fil, FW_attr, UPDATE_FW_ATTR_SIZE, &rsize);
				if (((*(volatile INT32U *)(FW_attr)) & 0x2FFE0000) == 0x20000000)	{//У��̼���������					
					msg_pkt_udisk.Src = MSG_WARNING_DSIPLAY;
					msg_pkt_udisk.Data = data_buf;
					f_lseek(&udiskfs.fil, 0x20);
					f_read(&udiskfs.fil, FW_attr, UPDATE_FW_ATTR_SIZE, &rsize);//��������
					f_read(&udiskfs.fil, FW_ver, UPDATE_FW_ATTR_SIZE, &rsize);//���汾��
					if(0==strcmp(FW_attr, "Mainboard"))        {//У������ƣ�����or��������	
						if(0!=strcmp(FW_ver, CONFIG_SYSINFO_FW_Version))	{//�汾��һ�²�����
							Sys.state |= SysState_UpdataFWTB;
							sprintf((char *)data_buf, "��⵽����̼� Ver:%s\r\n�Ƿ�������",FW_ver);					
							OSQPost(appdis.MSG_Q,&msg_pkt_udisk);//֪ͨapp_display��ʾ
						}
					}
				}
			}
		}
		f_close(&udiskfs.fil);
	}
}
