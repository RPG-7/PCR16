#include "app_display.h"
#include "display_code.h"
#include "app_temp.h"
#include "DaCai_TouchEvent.h"
//��ջ
__align(4) OS_STK  TASK_DISPLAY_STK[STK_SIZE_DISPLAY]; //�����ջ��?
#define N_MESSAGES		5
static u8 databuf[64];
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

void StartSysLab(void)
{
	StartAPPTempCtrl();
//	StartAppADTask();
	StartCollFluo();
	Sys.devstate = DevState_Running;
	Sys.devsubstate = DevSubState_TempUp;
}

void StopSysLab(void)
{
	Sys.devstate = DevState_IDLE;
	Sys.devsubstate = DevSubState_Unkown;
//	StopAppADTask();
	StopCollFluo();
	StopAPPTempCtrl();
}
//u8 touchid;
static void ButtonClickProcess(u8 button);
static void ScreenDataProcess(_dacai_usart_t *pUsart);
static  void  UsartCmdParsePkt (_dacai_usart_t *pUsart)
{
	u8 cmd;
	u16 iPara;
	
	cmd = DaCaiRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
	switch(cmd)	{
		case HANDSHAKE_OK://���ֳɹ�
			appdis.pDaCai->state = DEF_ONLINE;
			break;
		case RESTART_OK://�豸��λ ��������
			appdis.pDaCai->state = DEF_OFFLINE;
			appdis.pUI->screen_id = Invalid_UIID;
			break;
		case 0x01:
			if(appdis.pUI->screen_id == Temp_UIID&&Sys.devstate != DevState_Running)	{
				u16 x,y;
				x = DaCaiRxGetINT16U(pUsart->rx_buf,&pUsart->rx_idx);
				y = DaCaiRxGetINT16U(pUsart->rx_buf,&pUsart->rx_idx);
				u8 touchid = TempButtonClick(x,y);
				ButtonClickProcess(touchid);
			}
			break;
		case 0xb1:	{//�������ָ��
			iPara = DaCaiRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			if(iPara==0x01)	{//�ض�screen id
				appdis.pUI->screen_id = DaCaiRxGetINT16U(pUsart->rx_buf,&pUsart->rx_idx);
				if(appdis.pUI->screen_id == Main_UIID)	{
					if(Sys.devstate != DevState_Running)	{
						ResetLabDataDefault();
						ResetTempDataDefault();
					}
				}
				else if(appdis.pUI->screen_id == Temp_UIID)	{//�¶ȳ������ ˢ���¶�ͼ��
					DisplayTempProgramUI(1,1);//ˢ���¶Ƚ���
				}
			}
			else if(iPara==0x11||iPara==0x14)	{//��ť״̬
				appdis.pUI->screen_id = DaCaiRxGetINT16U(pUsart->rx_buf,&pUsart->rx_idx);
				appdis.pUI->ctrl_id = DaCaiRxGetINT16U(pUsart->rx_buf,&pUsart->rx_idx);
				ScreenDataProcess(pUsart);
			}
			break;
		}
		case 0xf7:	{//ʱ���ʽ��BCD��
			SysTime.tm_year = BCD_Decimal(DaCaiRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx)) + 2000;
			SysTime.tm_mon = BCD_Decimal(DaCaiRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx));
			SysTime.tm_wday = BCD_Decimal(DaCaiRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx));
			SysTime.tm_mday = BCD_Decimal(DaCaiRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx));
			SysTime.tm_hour = BCD_Decimal(DaCaiRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx));
			SysTime.tm_min = BCD_Decimal(DaCaiRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx));
			SysTime.tm_sec = BCD_Decimal(DaCaiRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx));
			break;
		}
	}
}
s8 modify_stageid,modify_stepid;
static void ButtonClickProcess(u8 button)
{
	if(button>=0&&button<=4)	{//����	���༭
		TempButtonPressID = button;
		if(CheckIdFromButton(TempButtonPressID, (u8 *)&modify_stageid, (u8 *)&modify_stepid)==1)	{
			sprintf((char *)databuf,"ѡ���Stage%d/Step%d ������ʽ", modify_stageid+1, modify_stepid+1);
			DisplayWarningUI((char *)databuf, (char *)&Code_Choose[2][0], (char *)&Code_Choose[3][0]);
			Sys.state |= SysState_StepTB;
		}
	}
	else if(button>=5&&button<=9)	{//����	�׶α༭
		TempButtonPressID = button - 5;
		if(CheckIdFromButton(TempButtonPressID, (u8 *)&modify_stageid, (u8 *)&modify_stepid)==1)	{
			sprintf((char *)databuf,"ѡ���Stage%d ������ʽ", modify_stageid+1);
			DisplayWarningUI((char *)databuf, (char *)&Code_Choose[2][0], (char *)&Code_Choose[3][0]);
			Sys.state |= SysState_StageTB;
		}
	}
//	else if(button==11)	{
//		if(temp_data.HeatCoverEnable == DEF_False)	{									
//			temp_data.HeatCoverEnable = DEF_True;
//		}
//		else if(temp_data.HeatCoverEnable == DEF_True)	{
//			temp_data.HeatCoverEnable = DEF_False;
//		}
//		DisplayHeatCoverIcon();		
//	}
}

//��Ļ������ݴ���
static void ScreenDataProcess(_dacai_usart_t *pUsart)
{
	u8 ctrl_type,subtype,status;
	s32 temp;
	u8 iPara;
	
	ctrl_type = DaCaiRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);//0x10-��ʾ��ť 0x11-��ʾ�ı� 0x1a-��ʾ������
	if(ctrl_type==0x10||ctrl_type==0x1a)	{		
		subtype = DaCaiRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
		status = DaCaiRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);//��ȡ��ť״̬ ����/����
	}
	if(appdis.pUI->screen_id==Main_UIID)	{//�����水ť��Ӧ		
		if(status == DEF_Release)	{
			if(appdis.pUI->ctrl_id == 1)	{//ʵ��
				DisplayLabUI();
			}
			else if(appdis.pUI->ctrl_id == 2)	{//ϵͳ
				
			}
			else if(appdis.pUI->ctrl_id == 3)	{//DNA 
				DisplayMenuUI();
				if(Sys.devstate != DevState_Running)	{
					ResetTempDataDNA();
				}
			}
			else if(appdis.pUI->ctrl_id == 4)	{//RNA
				DisplayMenuUI();
				if(Sys.devstate != DevState_Running)	{
					ResetTempDataRNA();
				}
			}
			else if(appdis.pUI->ctrl_id == 5)	{//����
				appdis.pUI->screen_id = Data_UIID;
				DaCai_SwitchUI(appdis.pUI);
			}
		}
	}
	else if(appdis.pUI->screen_id==Lab_UIID)	{//ʵ�����		
		if(appdis.pUI->ctrl_id >= 1&&appdis.pUI->ctrl_id <= 5)	{
			if(status == DEF_Press)	
				appdis.pUI->index = appdis.pUI->ctrl_id-1;
			else
				appdis.pUI->index = 0x0f;
		}		
		else if(status == DEF_Release)	{
			if(appdis.pUI->ctrl_id == 11)	{//�½�
				if(Sys.devstate == DevState_Running)	{
					DisplayMessageUI((char *)&Code_Message[5][0],1);
				}
				else	{
					ResetLabDataDefault();//�½�ʵ�� �ָ�Ĭ������
					ResetSampleDataDefault();
					ResetTempDataDefault();
					DisplayMenuUI();
				}
			}
			else if(appdis.pUI->ctrl_id == 12)	{//ɾ��
				if(Sys.devstate == DevState_Running)
					DisplayMessageUI((char *)&Code_Message[5][0],1);//������ �ܾ�ɾ��
				else if(appdis.pUI->index >= gLabTemplatelist.num)	
					DisplayMessageUI((char *)&Code_Message[3][0],1);//��Ч����
				else {
					sprintf((char *)appdis.pUI->pdata,"%sʵ�� %s ?", &Code_Warning[0][0], gLabTemplatelist.list[appdis.pUI->index].name);//��ʾ �Ƿ�ɾ��ʵ��xxx
					DisplayWarningUI((char *)appdis.pUI->pdata, (char *)&Code_Choose[0][0], (char *)&Code_Choose[1][0]);
					Sys.state |= SysState_DeleteLabTB;
				}
			}
			else if(appdis.pUI->ctrl_id == 13)	{//��ʵ��
				if(Sys.devstate == DevState_Running)	{
//					DisplayQiTingLab();
					DisplayMessageUI((char *)&Code_Message[5][0],1);//������
				}
				else if(appdis.pUI->index >= gLabTemplatelist.num)	
					DisplayMessageUI((char *)&Code_Message[3][0],1);//��Ч����
				else	{
					ResetLabDataDefault();//�½�ʵ�� �ָ�Ĭ������
					ResetSampleDataDefault();
					ResetTempDataDefault();					
					AnalysisLabTemplate(appdis.pUI->index);//����ʵ��ģ��	
					appdis.pUI->index = 0x0f;				
					DisplayMenuUI();					
//					DisplayQiTingLab();//���ݵ�ǰʵ��״̬����ʾֹͣʵ�黹������ʵ��	
				}					
			}
		}
	}
	else if(appdis.pUI->screen_id==Menu_UIID&&status == DEF_Release)	{//�˵�����
		if(appdis.pUI->ctrl_id == 1)	{//����������Ϣ
			DisplaySampleInforUI();
		}
		else if(appdis.pUI->ctrl_id == 2)	{//�����¶ȳ���		
			ClearTempProgramIdx();		
			DisplayTempProgramUI(0,1);	//ˢ���¶Ƚ���	����
		}
		else if(appdis.pUI->ctrl_id == 3)	{//����ʵ������			
			DisplayLabAttrUI();
		}			
		else if(appdis.pUI->ctrl_id == 19)	{//����ʵ��
			DisplayQiTingLab();//���ݵ�ǰʵ��״̬����ʾֹͣʵ�黹������ʵ��
		}
	}
	else if(appdis.pUI->screen_id==SampleInfor_UIID)	{//������Ϣ	
		if(appdis.pUI->ctrl_id > 0&&appdis.pUI->ctrl_id <= HOLE_NUM)	{//���Ƿ�ѡ�д���
			iPara = appdis.pUI->ctrl_id - 1;
			if(status == DEF_Release)
				appdis.pUI->button_id &= ~(DEF_BIT00_MASK << iPara);
			else
				appdis.pUI->button_id |= DEF_BIT00_MASK << iPara;
		}
		else if(appdis.pUI->ctrl_id==17&&status == DEF_Release)	{//������������
			if(subtype==0)	{//ȡ��ѡ��
				DisableSampleData(appdis.pUI->button_id);
				ClearButtonInSampleInfor();
				UpdateSampleInfor();
			}
			else	{
				SetSampleType(appdis.pUI->button_id, subtype);
				appdis.pUI->button_id |= DEF_BIT31_MASK;
				if(appdis.pUI->button_id & DEF_BIT30_MASK)	{					
					ClearButtonInSampleInfor();
				}
				UpdateSampleInfor();
			}
		}
		else if(appdis.pUI->ctrl_id==21&&status == DEF_Release)	{//ͨ������
			SetSampleChannel(appdis.pUI->button_id, subtype+1);
			appdis.pUI->button_id |= DEF_BIT30_MASK;
			if(appdis.pUI->button_id & DEF_BIT31_MASK)	{				
				ClearButtonInSampleInfor();
			}
			UpdateSampleInfor();
		}
		else if(appdis.pUI->ctrl_id==44&&status == DEF_Release)	{//��ʾ������Ϣ �б�
			DisplaySampleInforUIByList();
		}
	}
	else if(appdis.pUI->screen_id==SampleList_UIID)	{//������Ϣ	�б�
		if(appdis.pUI->ctrl_id > 0&&appdis.pUI->ctrl_id <= 5)	{//
			iPara = appdis.pUI->ctrl_id - 1 + appdis.pUI->index;
//			DisableHole(iPara);
//			if(status == DEF_Release)
//				sample_data.enable &= ~(DEF_BIT00_MASK << iPara);
//			else
//				sample_data.enable |= DEF_BIT00_MASK << iPara;
		}
		else if(appdis.pUI->ctrl_id==66&&status == DEF_Release)	{//��ҳ
			appdis.pUI->index -= 5;
			if(appdis.pUI->index<0)	appdis.pUI->index = 0;
			UpdateSampleInforList(appdis.pUI->index);
		}
		else if(appdis.pUI->ctrl_id==67&&status == DEF_Release)	{//��ҳ
			appdis.pUI->index += 5;
			if(appdis.pUI->index>=HOLE_NUM)	appdis.pUI->index = HOLE_NUM-1;
			UpdateSampleInforList(appdis.pUI->index);
		}
	}
	else if(appdis.pUI->screen_id==Temp_UIID)	{//�¶ȳ���	
		if(status == DEF_Release)	{
			if(appdis.pUI->ctrl_id == 9)	{//�༭�ȸ��¶�
				SaveUIEditInfor();//����༭��Ϣ
				DisplayKeyboardUI();//�л���ȫ���̽���					
			}
			else if(appdis.pUI->ctrl_id == 1)	{//�ӽ�	
				s8 m;
				m = temp_data.StageNum-1;
				if(m<0)	m=0;				
				DisplayStageUI(m);
				modify_stageid = temp_data.StageNum;
				Sys.state |= SysState_AddStage;
				Sys.state &= ~SysState_ReadTXT;
			}
			else if(appdis.pUI->ctrl_id == 4)	{//�Ӳ�
				s8 m,n;
				m = temp_data.StageNum-1;
				if(m<0)	m=0;
				n = temp_data.stage[m].StepNum-1;
				if(n<0)	n=0;
				DisplayStepUI(m, n);
				modify_stageid = temp_data.StageNum;
				modify_stepid = temp_data.stage[m].StepNum;				
				Sys.state |= SysState_AddStep;
				Sys.state &= ~SysState_ReadTXT;
			}
			else if(appdis.pUI->ctrl_id == 33)	{//��һҳ
				ClearTempProgramIdx();
				DisplayTempProgramUI(0,1);
			}
			else if(appdis.pUI->ctrl_id == 34)	{//��һҳ
				DisplayTempProgramUI(1,1);
			}
		}
	}
	else if(appdis.pUI->screen_id==LabAttr_UIID&&status == DEF_Press)	{//ʵ������
		if(appdis.pUI->ctrl_id == 12)	{//�û�����ֵ
			SaveUIEditInfor();//����༭��Ϣ
			DisplayKeyboardUI();//�л���ȫ���̽���
		}
		else if(appdis.pUI->ctrl_id == 6)	{
			lab_data.type = LabTypeNegativeOrPositive;
		}
		else if(appdis.pUI->ctrl_id == 7)	{
			lab_data.type = LabTypeQuantify;
		}
		else if(appdis.pUI->ctrl_id == 8)	{
			lab_data.type = LabTypeGeneticTyping;
		}
		else if(appdis.pUI->ctrl_id == 9)	{
			lab_data.method = LabMethodStandard;
		}
		else if(appdis.pUI->ctrl_id == 10)	{
			lab_data.method = LabMethodCompare;
		}
	}
	else if(appdis.pUI->screen_id==Stage_UIID)	{//�׶�����
		if(Sys.state&SysState_ReadTXT)	{
			strcpy(appdis.pUI->pdata, (const char *)(pUsart->rx_buf+pUsart->rx_idx));
			temp = atoi(appdis.pUI->pdata);	
			if(appdis.pUI->ctrl_id == 4)	{//ѭ��������			
				if(temp > STAGE_REPEAT_MAX||temp <= 0)	{
					DisplayMessageUI((char *)&Code_Message[3][0],1);
				}
				else	{
					temp_data.stage[modify_stageid].RepeatNum = temp;
					appdis.pUI->ctrl_id = 8;
					DaCai_ReadTXT(appdis.pUI);
				}
			}
			else if(appdis.pUI->ctrl_id == 8)	{//�ܲ�������				
				if(temp > STEP_MAX||temp <= 0)	{
					DisplayMessageUI((char *)&Code_Message[3][0],1);
				}
				else	{
					temp_data.stage[modify_stageid].StepNum = temp;
					Sys.state &= ~SysState_ReadTXT;
					DisplayMessageUI((char *)&Code_Message[4][0],1);	
					Sys.state |= SysState_ReadTXTOK;				
				}
			}
		}
		else if(appdis.pUI->ctrl_id == 24)	{//enter
			appdis.pUI->ctrl_id = 4;
			DaCai_ReadTXT(appdis.pUI);
			Sys.state |= SysState_ReadTXT;
			Sys.state &= ~SysState_ReadTXTOK;
		}
		else if(appdis.pUI->ctrl_id == 25)	{//�رմ���
			if(status == DEF_Press)	{
//				Sys.state &= ~SysState_ReadTXT;
				if(Sys.state & SysState_ReadTXTOK)	{
					Sys.state &= ~SysState_ReadTXTOK;
					if(Sys.state & SysState_AddStage)	{
						temp_data.StageNum++;
						Sys.state &= ~SysState_AddStage;
					}
				}
			}
			appdis.pUI->screen_id = Temp_UIID;
		}
	}
	else if(appdis.pUI->screen_id==Step_UIID)	{//������
		if(Sys.state&SysState_ReadTXT)	{//�ض��ı��ؼ�����
			strcpy(appdis.pUI->pdata, (const char *)(pUsart->rx_buf+pUsart->rx_idx));
			temp = (int)(atof(appdis.pUI->pdata)*100);
			if(appdis.pUI->ctrl_id == 8)	{//�¶�ֵ����
				if(temp > HOLE_TEMPMAX||temp < HOLE_TEMPMIN)
					DisplayMessageUI((char *)&Code_Message[3][0],1);
				else	{
					temp_data.stage[modify_stageid].step[modify_stepid].temp = temp;
					appdis.pUI->ctrl_id = 9;
					DaCai_ReadTXT(appdis.pUI);
				}
			}	
			else if(appdis.pUI->ctrl_id == 9)	{//����ʱ�� min	
				temp /= 100;
				if(temp > 10||temp < 0)	
					DisplayMessageUI((char *)&Code_Message[3][0],1);
				else	{
					temp_data.stage[modify_stageid].step[modify_stepid].tim = temp*60;
					appdis.pUI->ctrl_id = 10;
					DaCai_ReadTXT(appdis.pUI);
				}
			}
			else if(appdis.pUI->ctrl_id == 10)	{//����ʱ�� sec	
				temp /= 100;
				if(temp > 60||temp < 0)	
					DisplayMessageUI((char *)&Code_Message[3][0],1);
				else	{
					temp_data.stage[modify_stageid].step[modify_stepid].tim += temp;
					Sys.state &= ~SysState_ReadTXT;
					DisplayMessageUI((char *)&Code_Message[4][0],1);					
					Sys.state |= SysState_ReadTXTOK;
				}
			}
		}
		else if(appdis.pUI->ctrl_id == 24)	{//enter
			appdis.pUI->ctrl_id = 8;
			DaCai_ReadTXT(appdis.pUI);
			Sys.state |= SysState_ReadTXT;//�ض������ı�����
			Sys.state &= ~SysState_ReadTXTOK;
		}
		if(appdis.pUI->ctrl_id == 25)	{//�رմ��� 	
			if(status == DEF_Press)	{
//				Sys.state &= ~SysState_ReadTXT;
				if(Sys.state & SysState_ReadTXTOK)	{//��ȡ�ı��ؼ�����ok
					Sys.state &= ~SysState_ReadTXTOK;
					if(Sys.state & SysState_AddStep)	{
						iPara = temp_data.StageNum;
						temp_data.stage[iPara].StepNum = 1;//��һ��
						temp_data.StageNum++;
						Sys.state &= ~SysState_AddStep;
					}
				}
			}
			appdis.pUI->screen_id = Temp_UIID;
		}
		else if(appdis.pUI->ctrl_id == 1)	{
			if(status == DEF_Release)	{
				temp_data.stage[modify_stageid].step[modify_stepid].CollEnable = DEF_False;
			}
			else if(status == DEF_Press)	{
				temp_data.stage[modify_stageid].step[modify_stepid].CollEnable = DEF_True;
			}
		}		
	}
	else if(appdis.pUI->screen_id==Keyboard_UIID)	{//ȫ���̽���
		if(appdis.pUI->ctrl_id == 43&&ctrl_type==0x11)	{//�û�����ֵ
			strcpy(appdis.pUI->pdata, (const char *)(pUsart->rx_buf+pUsart->rx_idx));		
			if(appdis.pUI->editinfo.screen_id == Temp_UIID)	
			{
				if(appdis.pUI->editinfo.ctrl_id == 9)	{//�༭�ȸ��¶�
					temp = atoi(appdis.pUI->pdata);
					if(temp > HEATCOVER_TEMPMAX || temp < HEATCOVER_TEMPMIN)	{
						DisplayMessageUI((char *)&Code_Message[3][0], 1);
					}else	{
						DisplayEditUI();//��ʾ�ϴα༭����
						appdis.pUI->ctrl_id = 6;
						appdis.pUI->datlen = strlen(appdis.pUI->pdata);//��ʾ�û�����ֵ
						DaCai_UpdateTXT(appdis.pUI);
						temp_data.HeatCoverTemp = temp;//�����û�����ֵ
					}
				}
			}
			else if(appdis.pUI->editinfo.screen_id == LabAttr_UIID)	{
				DisplayEditUI();//��ʾ�ϴα༭����
				appdis.pUI->ctrl_id = 5;
				appdis.pUI->datlen = strlen(appdis.pUI->pdata);//��ʾ�û�����ֵ
				DaCai_UpdateTXT(appdis.pUI);
				strcpy(lab_data.name, appdis.pUI->pdata);//�����û�����ֵ
			}
		}
		else if(appdis.pUI->ctrl_id == 44&&status == DEF_Release)	{//�ر� 
			DisplayEditUI();//��ʾ�ϴα༭����
			if(appdis.pUI->screen_id==Temp_UIID&&appdis.pUI->ctrl_id == 9)	
				DisplayTempProgramUI(0,0);//ˢ���¶Ƚ���
		}
	}
	else if(appdis.pUI->screen_id == Confirm_UIID&&status == DEF_Release)	{//ȷ�Ͻ���
		if(appdis.pUI->ctrl_id == 2)	{//ȡ����
			appdis.pUI->button_id = 0xff;
			if(Sys.state & SysState_StepTB)	{//ɾ����
				DeleTempProgam(TempButtonPressID, 1);
				TempButtonPressID = 0xff;
			}
			else if(Sys.state & SysState_StageTB)	{//ɾ���׶�
				DeleTempProgam(TempButtonPressID, 0);////ɾ������ type=0 ɾ�׶Σ�type=1 ɾ����
				TempButtonPressID = 0xff;
			}
		}
		else	if(appdis.pUI->ctrl_id == 3)	{//ȷ�ϼ�
			if(Sys.state & SysState_RunningTB)	{//����ʵ��
				Sys.state &= ~SysState_RunningTB;				
				StartSysLab();//�������� ����ʵ�� 
				DisplayBackupUIID();	
			}
			else if(Sys.state & SysState_StopTB)	{//ֹͣʵ��
				StopSysLab();
			}
			else if(Sys.state & SysState_DeleteLabTB)	{//ɾ��ʵ���¼
				DisplayMessageUI((char *)&Code_Message[6][0], 0);//��ʾ ɾ����
				DeleteLabTemplate(appdis.pUI->index);
				appdis.pUI->index = 0x0f;
			}
			else if(Sys.state & SysState_StepTB)	{//�޸Ĳ�
				Sys.state &= ~SysState_StepTB;
				DisplayStepUI(modify_stageid, modify_stepid);
				Sys.state &= ~SysState_AddStep;
				return;
			}
			else if(Sys.state & SysState_StageTB)	{//�޸Ľ׶�
				Sys.state &= ~SysState_StageTB;
				DisplayStageUI(modify_stageid);
				Sys.state &= ~SysState_AddStage;
				return;
			}
			else if(Sys.state & SysState_UpdataFWTB)	{//�����̼�
				Sys.state &= ~SysState_UpdataFWTB;
				FWUpdate_reboot();//����IAP �����̼�
				return;
			}
		}
		else	if(appdis.pUI->ctrl_id == 6)	{//�˳���
			
		}
		ClearAllSysStateTB();
		DisplayBackupUIID();		
	}
	else if(appdis.pUI->screen_id == Message_UIID&&status == DEF_Release)	{//ȷ�Ͻ���
		if(appdis.pUI->ctrl_id == 3)	{//ȷ�ϼ�
			DisplayBackupUIID();
		}
	}
}

static void time_event(void)
{
	static u8 event_cnt;
	
	event_cnt++;
	if((event_cnt%2)==0)	{
		DaCai_TimeGet();
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
			else if(msg->Src == MSG_MESSAGE_DSIPLAY)	{//��ʾ����������Ҫ��ʾ������
				DisplayMessageUI((char *)msg->Data, 1);
			}
			else if(msg->Src == MSG_WARNING_DSIPLAY)	{
				DisplayWarningUI((char *)msg->Data, (char *)&Code_Choose[0][0], (char *)&Code_Choose[1][0]);
			}
			else if(msg->Src == MSG_NONE_DSIPLAY)	{
				DaCai_SwitchUI(appdis.pUI);
				appdis.pUI->sub_screen_id = Invalid_UIID;
			}
		}
		else if(err==OS_ERR_TIMEOUT)	{
			if(appdis.pDaCai->state==DEF_OFFLINE)	{
				DaCai_CheckDevice();//check online
			}else if(appdis.pDaCai->state==DEF_ONLINE)	{
				if(appdis.pUI->screen_id == Invalid_UIID)	{
					OSTimeDly(2000);
					appdis.pUI->screen_id = Main_UIID;							
					DaCai_SwitchUI(appdis.pUI);//��ʾ������
//					DaCai_ConfigTouch(0x31);//�رմ��������ϴ�
//					DaCai_ClearScreenAuto(DEF_Disable);
//					OSFlagPost(SysFlagGrp, (OS_FLAGS)FLAG_GRP_3, OS_FLAG_SET, &err);
				}
				else {
					time_event();
				}
			}
		}
	}
}
