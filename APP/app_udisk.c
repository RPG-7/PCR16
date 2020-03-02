#include "app_udisk.h"
#include "rw_udisk.h"
#include "bsp.h"
//��ջ
__align(4) OS_STK  TASK_UDISK_STK[STK_SIZE_UDISK]; //�����ջ��?
#define N_MESSAGES		5
static u8 data_buf[30];
_appudisk_t appudisk;
static void    *MyArrayOfMsg[N_MESSAGES];//��Ϣ��������
static  message_pkt_t    msg_pkt_udisk;
static void TaskUDISK(void * ppdata);
extern USBH_HandleTypeDef  hUSB_Host;
//void CheckUpdateFWName(void);
//#define	UPDATE_FW_NAME	"PerfuserFirmware.bin"
//#define	UPDATE_FW_ATTR_SIZE		0X10
//#define	UPDATE_FW_FILE_MINSIZE		0X400

void AppUSBInit(void)
{
	OSTaskCreate(TaskUDISK,  (void * )0, (OS_STK *)&TASK_UDISK_STK[STK_SIZE_UDISK-1], TASK_PRIO_UDISK);
}

static void DataInit(void)
{
	appudisk.MSG_Q 			 = OSQCreate(&MyArrayOfMsg[0],N_MESSAGES);//
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
		msg = (message_pkt_t *)OSQPend(appudisk.MSG_Q, 0, &err);
		if(err==OS_ERR_NONE)    {
			if(msg->Src == MSG_USB_START || msg->Src == MSG_USB_READY || msg->Src == MSG_USB_DISCONNECT)	{
				USBH_Process(&hUsbHostFS);
				if(hUsbHostFS.gState != HOST_IDLE)	{
					userMSC_Handle =  (MSC_HandleTypeDef *) hUsbHostFS.pActiveClass->pData;
					if(userMSC_Handle->state != MSC_IDLE)	{
						OSTimeDly(10);
						msg_pkt_udisk.Src = MSG_USB_START;
						OSQPost(appudisk.MSG_Q, &msg_pkt_udisk);		
					}						
				}
				if(msg->Src == MSG_USB_READY||msg->Src == MSG_USB_DISCONNECT)	{
					MountUDISK(msg->Src);
					if(udiskfs.flag & UDISKFLAG_MOUNTED && \
						(sys.devstate == DevState_IDLE||sys.devstate == DevState_Error))	{//�豸���л��������� ����Ƿ��������̼�
//						CheckUpdateFWName();
					}
				}
			}
			else if(msg->Src == MSG_COPY_LOGFILE)	{//ִ��log����

			}
			else if(msg->Src == MSG_COPY_PERFUSEFILE)	{//ִ�й�ע���ݿ���

			}
			else if(msg->Src == MSG_COPY_UPDATE_FW)	{//ִ�������̼�����
//				msg_pkt_udisk.Src = MSG_MESSAGE_DSIPLAY;
//				if(sys.state & SysState_UdiskConnect)	{
//					msg_pkt_udisk.Data = (char *)&Code_Message[2][0];
//					OSQPost(appdis.MSG_Q,&msg_pkt_udisk);//֪ͨapp_display��ʾ
//					if(CopyFileToSpiflash(UPDATE_FW_NAME, UPDATE_FW_NAME)==1)	{
//						msg_pkt_udisk.Data = (char *)&Code_Message[0][0];
//						OSQPost(appdis.MSG_Q,&msg_pkt_udisk);//֪ͨapp_display��ʾ
//						FWUpdate_reboot();
//					}
//				}else	{
//					msg_pkt_udisk.Data = (char *)&Code_Message[1][0];
//				}
//				OSQPost(appdis.MSG_Q,&msg_pkt_udisk);//֪ͨapp_display��ʾ
			}
		}
  }
}
#define		ApplicationAddress			0x0800F000
#define		ApplicationJump				0x200
//void CheckUpdateFWName(void)
//{
//	FRESULT res;
//	char file_name[FILE_NAME_LEN];
//	char FW_attr[UPDATE_FW_ATTR_SIZE];
//	char FW_ver[UPDATE_FW_ATTR_SIZE];
//	u32 rsize;
//	
//	sprintf(file_name, "%s%s", USBHPath, UPDATE_FW_NAME);//�̼�·��
//	res = f_open(&udiskfs.fil, file_name, FA_READ);
//	if(res==FR_OK)	{
//		if(f_size(&udiskfs.fil)<UPDATE_FW_FILE_MINSIZE)	{//�̼�̫С �ж�Ϊ������̼�
//			f_close(&udiskfs.fil);
//			return;
//		}
//		f_read(&udiskfs.fil, FW_attr, UPDATE_FW_ATTR_SIZE, &rsize);
//		if(0==strcmp(FW_attr, firmware_check_string))  {//У�鹫˾����
//			f_read(&udiskfs.fil, FW_attr, UPDATE_FW_ATTR_SIZE, &rsize);
//			if(0==strcmp(FW_attr, firmware_attr))        {//У���豸����
//				f_lseek(&udiskfs.fil, ApplicationJump);
//				f_read(&udiskfs.fil, FW_attr, UPDATE_FW_ATTR_SIZE, &rsize);
//				if (((*(volatile INT32U *)(FW_attr)) & 0x2FFE0000) == 0x20000000)	{//У��̼���������					
//					msg_pkt_udisk.Src = MSG_WARNING_DSIPLAY;
//					msg_pkt_udisk.Data = data_buf;
//					f_lseek(&udiskfs.fil, 0x20);
//					f_read(&udiskfs.fil, FW_attr, UPDATE_FW_ATTR_SIZE, &rsize);//��������
//					f_read(&udiskfs.fil, FW_ver, UPDATE_FW_ATTR_SIZE, &rsize);//���汾��
//					if(0==strcmp(FW_attr, "Mainboard"))        {//У������ƣ�����or��������	
//						if(0!=strcmp(FW_ver, firmware_ver))	{//�汾��һ�²�����
//							rsize = sprintf((char *)data_buf, "��⵽����̼� Ver:%s\r\n",FW_ver);					
//							sys.state |= SYSSTATE_UPDATE_TBC;
//						}
//					}
////					else if(0==strcmp(FW_attr, "Slaveboard"))        {
////						rsize = sprintf((char *)data_buf, "��⵽�Ӱ�̼� Ver:%s\r\n",FW_ver);						
////						sys.state |= SYSSTATE_UPDATE_TBC;
////					}
//					if(sys.state & SYSSTATE_UPDATE_TBC)	{
//						if(sys_data.PowerType == POWERTYPE_DC)	//�е����� ��������					
//							sprintf((char *)data_buf + rsize, "�Ƿ�������");	
//						else	{
//							sprintf((char *)data_buf + rsize, "�������е�.");
//							sys.state &= ~SYSSTATE_UPDATE_TBC;
//						}
//						OSQPost(appdis.MSG_Q,&msg_pkt_udisk);//֪ͨapp_display��ʾ
//					}
//				}
//			}
//		}
//		f_close(&udiskfs.fil);
//	}
//}
