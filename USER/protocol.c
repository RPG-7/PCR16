#include "includes.h"
#include "app_usart.h"
#include "app_display.h"
#include "app_motor.h"
#include "protocol.h"
#include "PD_DataProcess.h"
#include "app_temp.h"

//static  message_pkt_t    msg_pkt_pro;
static u8 data_buf[100];

u8 UsartCmdProcess(usart_t *pUsart, message_pkt_t msg[])
{
	u8 iPara,data,idx;
	u8 ack_state;
	s32 temp;
	
	ack_state = ACK_NONE;
	switch(msg[0].Cmd)	{
		case _CMD_RW_SYS_INFOR:
			break;
		case _CMD_EXECUTE_SYS_INFOR:
		
			break;	
		case _CMD_READ_DevState://0X03,//��ȡ�豸����״̬
			idx = 0;
			data_buf[idx++] = Sys.devstate;
			data_buf[idx++] = Sys.devsubstate;
			msg[1].Data = data_buf;
			msg[1].dLen = idx;
			OSMboxPost(usart.mbox, &msg[1]);
			break;
		case _CMD_READ_RunningLabName://0X04,//��ȡ��ǰʵ������
			if(Sys.devstate == DevState_Running||Sys.devstate == DevState_Pause)	{
				strcpy((char *)data_buf,lab_data.name);				
			}
			else	{
				strcpy((char *)data_buf,"NONE");
			}		
			msg[1].Data = data_buf;
			msg[1].dLen = strlen((char *)data_buf);
			OSMboxPost(usart.mbox, &msg[1]);
			break;
		case _CMD_READ_RunningLabData://0X05,//��ȡ��ǰʵ������
		{
//			u8 m,n;
//			
//			idx = 0;
//			m = temp_data.CurStage;
//			n = temp_data.stage[m].CurStep;
//			data_buf[idx++] = m<<4 | (n&0x0f);//��ǰstage �� step
//			data_buf[idx++] = temp_data.stage[data].CurRepeat;//��ǰrepeat
//			temp = (u16)app_temp.current_t[TEMP_ID3];//�ȸ��¶�
//			memcpy(data_buf+idx,(u8 *)&temp, 2);
//			idx += 2;
//			temp = (u16)app_temp.current_t[TEMP_ID1];//ģ���¶�1
//			memcpy(data_buf+idx,(u8 *)&temp, 2);
//			idx += 2;
//			temp = (u16)app_temp.current_t[TEMP_ID2];//ģ���¶�2
//			memcpy(data_buf+idx,(u8 *)&temp, 2);
//			idx += 2;
//			data_buf[idx++] = temp_data.stage[m].step[n].tim/60;
//			data_buf[idx++] = temp_data.stage[m].step[n].tim%60;
//			data_buf[idx++] = HOLE_NUM;
//			memcpy(data_buf+idx,(u8 *)gPD_Data.PDVol,sizeof(gPD_Data.PDVol));
//			idx += sizeof(gPD_Data.PDVol);
//			msg[1]->Data = data_buf;
//			msg[1]->dLen = idx;
//			OSMboxPost(usart.mbox, &msg[1]);
			break;
		}
		case _CMD_READ_SysError://0X06,//��ȡ����
			break;
		case _CMD_SET_LabState://0X07,//����ʵ����ͣ
			iPara = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			if(iPara==0)	{}
			else if(iPara==1)	{//����ʵ��
//				StartSysLab();
			}
			else if(iPara==2)	{//��ͣʵ��
			}
			else if(iPara==3)	{//ֹͣʵ��
			}
			ack_state = ACK_OK;
			break;
		case _CMD_CALIBRATE://0x08 ִ��У׼
			iPara = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			if(iPara==0)	{//У׼�տ�PD�����ź�				
				msg[0].Src = MSG_CaliHolePDBase_EVENT;//������� У׼�տ�PD�����ź�	
				OSMboxPost(tMotor[MOTOR_ID1].Mbox, &msg[0]);	
			}
			else if(iPara==1)	{//��λ��У׼				
				msg[0].Src = MSG_CaliHolePostion_EVENT;//������� ��ʼУ׼��λ��
				OSMboxPost(tMotor[MOTOR_ID1].Mbox, &msg[0]);	
			}
			else if(iPara==2)	{
				memset(gPD_Data.PDVol, 0, HOLE_NUM);
				msg[0].Src = MSG_CollectHolePD_EVENT;//������� ��ʼ�ɼ���PDֵ
				OSMboxPost(tMotor[MOTOR_ID1].Mbox, &msg[0]);	
			}
			ack_state = ACK_OK;
			break;
		case _CMD_READ_CalibrateRes://0X09 ��ȡУ׼���
			iPara = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			if(iPara==0)	{//��ȡ�տ�ӫ�����ֵ��Сֵ
				msg[1].Data = (u8 *)gPD_Data.PDBaseBlue;
				msg[1].dLen = sizeof(gPD_Data.PDBaseBlue);
			}
			else if(iPara==1)	{//��ȡ��λ��
				msg[1].Data = (u8 *)HolePos.pos;
				msg[1].dLen = sizeof(HolePos.pos);
			}
			else 
				break;
			OSMboxPost(usart.mbox, &msg[1]);
			break;
		case _CMD_FILETRANSMIT_DOWNLOAD://0X0A,//�����ļ�
			msg[0].Src = MSG_FILETRANSMIT_DOWNLOAD;
			msg[0].Data = (u8 *)(pUsart->rx_buf+pUsart->rx_idx);
			msg[0].dLen = pUsart->rx_cnt - 1;
			OSMboxPost(usart.mbox, &msg[0]);
			break;
		case _CMD_FILETRANSMIT_UPLOAD://0X0B ��ȡ�ļ�
			msg[0].Src = MSG_FILETRANSMIT_UPLOAD;
			msg[0].Data = (u8 *)(pUsart->rx_buf+pUsart->rx_idx);
			msg[0].dLen = pUsart->rx_cnt - 1;
			OSMboxPost(usart.mbox, &msg[0]);
			break;
		case _CMD_LED_CTRL://0x0d LED�ƿ���
		{
			u8 led_type;
			
			led_type = LED_NONE;
			iPara = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			data = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			if(iPara==0)	{//0--��ɫ�ƣ�1����ɫ�ƣ�2�����е�
				led_type = LED_BLUE;
			}
			else if(iPara==1)	{
				led_type = LED_GREEN;
			}
			else if(iPara==2)	{
				led_type = LED_GREEN | LED_BLUE;
			}
			FluoLED_OnOff(led_type, data);//0����1������
			ack_state = ACK_OK;
			break;
		}
		case _CMD_RESET_MOTOR://0X0E,//�����λ
			if(tMotor[MOTOR_ID1].status.is_run == MotorState_Run)
				ack_state = ACK_Fail;	
			else	{
				OSMboxPost(tMotor[MOTOR_ID1].Mbox, &msg[0]);
				ack_state = ACK_OK;		
			}
			break;
		case _CMD_DBG_MoveAnyPosAtReset://0x0F,//�ƶ����	
			if(tMotor[MOTOR_ID1].status.is_run == MotorState_Run)
				ack_state = ACK_Fail;	
			else	{
				tMotor[MOTOR_ID1].dst_pos = (s32)UsartRxGetINT16U(pUsart->rx_buf,&pUsart->rx_idx);
				OSMboxPost(tMotor[MOTOR_ID1].Mbox, &msg[0]);
				ack_state = ACK_OK;
			}
			break;
		case _CMD_GetMotorStatus://0x10,//��ѯ���״̬
			data_buf[0] = tMotor[MOTOR_ID1].status.is_run;
			msg[1].Data = data_buf;
			msg[1].dLen = 1;
			OSMboxPost(usart.mbox, &msg[1]);
			break;
		case _CMD_GetMotorPositon://0x11,//��ȡ���λ��
			temp = StepsToLen(&tMotor[MOTOR_ID1]);
			memcpy(data_buf,(u8 *)&temp, 2);
			msg[1].Data = data_buf;
			msg[1].dLen = 2;
			OSMboxPost(usart.mbox, &msg[1]);
			break;
		case _CMD_SetTemp:// 0x13,//����ģ���¶�
			iPara = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			data = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			temp = (s16)UsartRxGetINT16U(pUsart->rx_buf,&pUsart->rx_idx);
			if(iPara==0)	{//����ģ���¶�			
				SetPIDTarget(PID_ID1, temp);
			}else if(iPara==1)	{//�����ȸ��¶�
				SetPIDTarget(PID_ID2, temp);
			}
			if(data==0)	{//ֹͣ
				Sys.devstate = DevState_IDLE;
				Sys.devsubstate = DevSubState_Unkown;
			}else if(data==1)	{//����
				Sys.devstate = DevState_Debug;
				Sys.devsubstate = DevSubState_DebugTemp;
			}						
			break;
		case _CMD_GetTemp://0x14,//��ȡģ���¶�
			iPara = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			if(iPara==0)	{//��ȡģ���¶�	
				temp = GetHoleTemperature();				
			}
			else if(iPara==1)	{//��ȡ�ȸ��¶�
				temp = GetCoverTemperature();
			}
			else 
				break;
			memcpy(data_buf,(u8 *)&temp, 2);
			msg[1].Data = data_buf;
			msg[1].dLen = 2;
			OSMboxPost(usart.mbox, &msg[1]);
			break;
		default:
			break;
	}
	return ack_state;
}

