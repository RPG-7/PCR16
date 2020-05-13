#include "includes.h"
#include "app_usart.h"
#include "app_display.h"
#include "app_motor.h"
#include "protocol.h"
#include "PD_DataProcess.h"

static  message_pkt_t    msg_pkt_pro;
static u8 data_buf[15];

u8 UsartCmdProcess(usart_t *pUsart, message_pkt_t *msg)
{
	u8 iPara,data;
	u8 ack_state;
	
	ack_state = ACK_OK;
	iPara = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
	switch(msg->Cmd)	{
		case _CMD_RW_SYS_INFOR:
			break;
		case _CMD_EXECUTE_SYS_INFOR:
		
			break;				
		case _CMD_CALIBRATE://0x08 ִ��У׼
			if(iPara==0)	{//�տ�ӫ��ֵУ׼				
				msg_pkt_pro.Src = MSG_CollTemplateHolePD_EVENT;//������� ��ʼ�ɼ��տ�PD���ֵ ��Сֵ
				OSMboxPost(tMotor[MOTOR_ID1].Mbox, &msg_pkt_pro);	
			}
			else if(iPara==1)	{//��λ��У׼				
				msg_pkt_pro.Src = MSG_CaliHolePostion_EVENT;//������� ��ʼУ׼��λ��
				OSMboxPost(tMotor[MOTOR_ID1].Mbox, &msg_pkt_pro);	
			}
			else if(iPara==2)	{
				msg_pkt_pro.Src = MSG_CollectHolePD_EVENT;//������� ��ʼ�ɼ���PDֵ
				OSMboxPost(tMotor[MOTOR_ID1].Mbox, &msg_pkt_pro);	
			}
			break;
		case _CMD_LED_CTRL://0x0d LED�ƿ���
		{
			u8 led_type;
			led_type = LED_NONE;
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
			break;
		}
		default:
			break;
	}
	return ack_state;
}

