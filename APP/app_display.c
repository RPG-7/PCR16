#include "app_display.h"
#include "display_code.h"

//��ջ
__align(4) OS_STK  TASK_DISPLAY_STK[STK_SIZE_DISPLAY]; //�����ջ��?
#define N_MESSAGES		5
//LIFOBUFF_T ScreenIDLIFO;
_appdisplay_t appdis;

void    *AppDisMSG_Q[N_MESSAGES];//��Ϣ��������
static  message_pkt_t    msg_pkt_appdis[2];
static void TaskDisplay(void * ppdata);


void AppDisplayInit(void)
{
	OSTaskCreate(TaskDisplay,  (void * )0, (OS_STK *)&TASK_DISPLAY_STK[STK_SIZE_DISPLAY-1], TASK_PRIO_DISPLAY);
}

void StartAppDisplay(message_pkt_t *msg)
{
	OSQPost(appdis.MSG_Q, msg);
}

static void DisDatInit(void)
{
//	appdis.sem         = OSSemCreate(0);
	appdis.MSG_Q 			 = OSQCreate(&AppDisMSG_Q[0],N_MESSAGES);//

	appdis.pDaCai = &dacai;
	appdis.pUI = &UI;
//	LIFOBuffer_Init(&ScreenIDLIFO,(u8 *)appdis.pUI->screen_idlifo, 1 ,SCREEN_BUFSIZE);
}

static void ScreenDataProcess(_dacai_usart_t *pUsart);
static  void  UsartCmdParsePkt (_dacai_usart_t *pUsart)
{
	u8 cmd;
	u16 iPara;
	
	cmd = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
	switch(cmd)	{
		case HANDSHAKE_OK://���ֳɹ�
			appdis.pDaCai->state = DEF_ONLINE;
			break;
		case RESTART_OK://�豸��λ ��������
			appdis.pDaCai->state = DEF_OFFLINE;
			appdis.pUI->screen_id = Invalid_UIID;
			break;
		case 0xb1:	{//�������ָ��
			iPara = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			if(iPara==0x01)	{//�ض�screen id
				appdis.pUI->screen_id = UsartRxGetINT16U(pUsart->rx_buf,&pUsart->rx_idx);
			}
			else if(iPara==0x11)	{//��ť״̬
				appdis.pUI->screen_id = UsartRxGetINT16U(pUsart->rx_buf,&pUsart->rx_idx);
				appdis.pUI->ctrl_id = UsartRxGetINT16U(pUsart->rx_buf,&pUsart->rx_idx);
//				UsartRxGetINT16U(pUsart->rx_buf,&pUsart->rx_idx);
//				appdis.pUI->ButtonState = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
				ScreenDataProcess(pUsart);
			}
			break;
		}
		case 0xf7:	{//ʱ���ʽ��BCD��
			SysTime.tm_year = (UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx));
			SysTime.tm_mon = (UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx));
			SysTime.tm_wday = (UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx));
			SysTime.tm_mday = (UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx));
			SysTime.tm_hour = (UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx));
			SysTime.tm_min = (UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx));
			SysTime.tm_sec = (UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx));
			break;
		}
	}
}u8 ctrl_type,subtype,status;
//��Ļ������ݴ���
static void ScreenDataProcess(_dacai_usart_t *pUsart)
{
	s32 temp;
	
	
	ctrl_type = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);//0x10-��ʾ��ť 0x11-��ʾ�ı�	
	if(ctrl_type==0x10)	{		
		subtype = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
		status = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);//��ȡ��ť״̬ ����/����
	}
	if(appdis.pUI->screen_id==Main_UIID)	{//�����水ť��Ӧ		
		if(status == DEF_Releass)	{
			if(appdis.pUI->ctrl_id == 1)	{
				appdis.pUI->screen_id = Lab_UIID;
				DaCai_SwitchUI(appdis.pUI);
			}
			else if(appdis.pUI->ctrl_id == 3||appdis.pUI->ctrl_id == 4)	{//DNA RNA
				appdis.pUI->screen_id = Menu_UIID;
				DaCai_SwitchUI(appdis.pUI);
			}
			else if(appdis.pUI->ctrl_id == 5)	{//����
				appdis.pUI->screen_id = Data_UIID;
				DaCai_SwitchUI(appdis.pUI);
			}
		}
	}
	else if(appdis.pUI->screen_id==Lab_UIID)	{//ʵ�����
		if(status == DEF_Releass)	{
			if(appdis.pUI->ctrl_id == 6)	{//�½�
				appdis.pUI->screen_id = Menu_UIID;
				DaCai_SwitchUI(appdis.pUI);
			}
			else if(appdis.pUI->ctrl_id == 7)	{//ɾ��
				DisplayUIIDAndBackup(Confirm_UIID);//���ݵ�ǰ���� �л�����һ������
				appdis.pUI->ctrl_id = 4;
				appdis.pUI->datlen = sprintf((char *)appdis.pUI->pdata,"%s", &Code_Warning[0][0]);//��ʾ �Ƿ�ɾ�� 
				DaCai_UpdateTXT(appdis.pUI);
				Sys.state |= SysState_DeleteLabTB;
			}
			else if(appdis.pUI->ctrl_id == 19)	{//��������
				DisplayUIIDAndBackup(Confirm_UIID);//���ݵ�ǰ���� �л�����һ������
				appdis.pUI->ctrl_id = 4;
				appdis.pUI->datlen = sprintf((char *)appdis.pUI->pdata,"%s", &Code_Warning[1][0]);//��ʾ �Ƿ�����
				DaCai_UpdateTXT(appdis.pUI);
				Sys.state |= SysState_RunningTB;
			}
		}
	}
	else if(appdis.pUI->screen_id==Temp_UIID)	{
		if(status == DEF_Releass)	{
			if(appdis.pUI->ctrl_id == 9)	{//�༭�ȸ��¶�
				SaveUIEditInfor();//����༭��Ϣ
				DisplayKeyboardUI();//�л���ȫ���̽���					
			}
		}
	}
	else if(appdis.pUI->screen_id==Keyboard_UIID)	{//ȫ���̽���
		if(appdis.pUI->ctrl_id == 43&&ctrl_type==0x11)	{//�û�����ֵ
			strcpy(appdis.pUI->pdata, (const char *)(pUsart->rx_buf+pUsart->rx_idx));
		}
		else if(appdis.pUI->ctrl_id == 42)	{//enter			
			if(appdis.pUI->editinfo.screen_id == Temp_UIID)	
			{
				temp = atoi(appdis.pUI->pdata);
				if(temp>105||temp<=0)	{
					DisplayMessageUI((char *)&Code_Message[3][0]);
				}else	{
					DisplayEditUI();//��ʾ�ϴα༭����
					appdis.pUI->ctrl_id = 6;
					appdis.pUI->datlen = strlen(appdis.pUI->pdata);//��ʾ�û�����ֵ
					DaCai_UpdateTXT(appdis.pUI);
				}
			}
		}
		else if(appdis.pUI->ctrl_id == 44&&status == DEF_Releass)	{//�ر� 
			DisplayEditUI();//��ʾ�ϴα༭����
		}
	}
	else if(appdis.pUI->screen_id == Confirm_UIID&&status == DEF_Releass)	{//ȷ�Ͻ���
		if(appdis.pUI->ctrl_id == 2)	{//ȡ����
			
		}
		else	if(appdis.pUI->ctrl_id == 3)	{//ȷ�ϼ�
			if(Sys.state & SysState_RunningTB)	{//����ʵ��
//				Sys.devstate = DevState_Running;
			}
			else if(Sys.state & SysState_DeleteLabTB)	{//ɾ��ʵ���¼
			
			}
		}
		else 
			return;
		DisplayBackupUIID();		
	}
	else if(appdis.pUI->screen_id == Message_UIID&&status == DEF_Releass)	{//ȷ�Ͻ���
		if(appdis.pUI->ctrl_id == 3)	{//ȷ�ϼ�
			DisplayBackupUIID();
		}
	}
}

static void TaskDisplay(void * ppdata)
{
	u8 err;
	message_pkt_t *msg;
	ppdata = ppdata;
	
	DisDatInit();
	DaCaiProto_init();
	DaCaiAPI_Init();

	for(;;)
	{
		msg = (message_pkt_t *)OSQPend(appdis.MSG_Q, 500, &err);//
		if(err==OS_ERR_NONE)    {
			if(msg->Src==USART_MSG_RX_TASK)	{//�����ڻظ��¼�
				UsartCmdParsePkt(appdis.pDaCai->puart_t);//������������				
			}
		}
		else if(err==OS_ERR_TIMEOUT)	{
			if(appdis.pDaCai->state==DEF_OFFLINE)	{
				DaCai_CheckDevice();//check online
			}else if(appdis.pDaCai->state==DEF_ONLINE)	{
//				if(appdis.pUI->screen_id == Invalid_UIID)	{
//					DaCai_GetScreenID();
//				}
//				else if(appdis.pUI->screen_id == Welcome_UIID)	{
				if(appdis.pUI->screen_id == Invalid_UIID)	{
					OSTimeDly(2000);
					appdis.pUI->screen_id = Main_UIID;							
					DaCai_SwitchUI(appdis.pUI);//��ʾ������
//					OSFlagPost(SysFlagGrp, (OS_FLAGS)FLAG_GRP_3, OS_FLAG_SET, &err);
				}
				else {
//					DaCai_TimeGet();
				}
			}
		}
	}
}
