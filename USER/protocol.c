#include "includes.h"
#include "app_usart.h"
#include "app_display.h"
#include "app_motor.h"
#include "protocol.h"
#include "PD_DataProcess.h"
#include "app_temp.h"
#include "app_filetransmit.h"
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
		case _CMD_READ_DevState://0X03,//读取设备运行状态
			idx = 0;
			data_buf[idx++] = Sys.devstate;
			data_buf[idx++] = Sys.devsubstate;
			msg[1].Data = data_buf;
			msg[1].dLen = idx;
			OSMboxPost(usart.mbox, &msg[1]);
			break;
		case _CMD_READ_RunningLabName://0X04,//读取当前实验名称
			if(Sys.devstate == DevState_Running||Sys.devstate == DevState_Pause)	{
				if(Sys.state & SysState_LabFromServer)//当前实验来自上位机
					data_buf[0] = LabTypeNone+2;
				else//当前实验由下位机自建
					data_buf[0] = LabTypeNone+1;
				strcpy((char *)data_buf,lab_data.name);
				idx = strlen((char *)data_buf) + 1;
			}
			else	{
				data_buf[0] = LabTypeNone;//当前无实验运行
				idx = 1;
			}
			msg[1].Data = data_buf;
			msg[1].dLen = idx;
			OSMboxPost(usart.mbox, &msg[1]);
			break;
		case _CMD_READ_RunningLabData://0X05,//读取当前实验运行数据
		{
			u8 m,n;
			
			idx = 0;
			m = temp_data.CurStage;//当前实验阶段
			n = temp_data.stage[m].CurStep;//当前实验步骤
			data_buf[idx++] = m<<4 | (n&0x0f);//当前stage 和 step
			data_buf[idx++] = temp_data.stage[data].CurRepeat;//当前实验循环repeat
			temp = (u16)GetCoverTemperature();//热盖温度
			memcpy(data_buf+idx,(u8 *)&temp, 2);
			idx += 2;
			temp = (u16)GetHoleTemperature(1);//模块温度
			memcpy(data_buf+idx,(u8 *)&temp, 2);
			idx += 2;
			temp = (u16)GetHoleTemperature(2);//模块温度2
			memcpy(data_buf+idx,(u8 *)&temp, 2);
			idx += 2;
			data_buf[idx++] = temp_data.stage[m].step[n].tim/60;
			data_buf[idx++] = temp_data.stage[m].step[n].tim%60;
			msg[1].Data = data_buf;
			msg[1].dLen = idx;
			OSMboxPost(usart.mbox, &msg[1]);
			break;
		}
		case _CMD_GetFluo://0x15,//获取荧光数据
			
			break;
		case _CMD_READ_SysError://0X06,//读取故障
			break;
		case _CMD_SET_LabState://0X07,//设置实验启停
			iPara = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			if(iPara==0)	{}
			else if(iPara==1)	{//启动实验
//				StartSysLab();
			}
			else if(iPara==2)	{//暂停实验
			}
			else if(iPara==3)	{//停止实验
			}
			ack_state = ACK_OK;
			break;
		case _CMD_CALIBRATE://0x08 执行校准
			iPara = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			if(iPara==0)	{//校准空孔PD本底信号				
				msg[0].Src = MSG_CaliHolePDBase_EVENT;//启动电机 校准空孔PD本底信号	
				OSMboxPost(tMotor[MOTOR_ID1].Mbox, &msg[0]);	
			}
			else if(iPara==1)	{//孔位置校准				
				msg[0].Src = MSG_CaliHolePostion_EVENT;//启动电机 开始校准孔位置
				OSMboxPost(tMotor[MOTOR_ID1].Mbox, &msg[0]);	
			}
			else if(iPara==2)	{
				memset(gPD_Data.PDVol, 0, HOLE_NUM);
				msg[0].Src = MSG_CollectHolePD_EVENT;//启动电机 开始采集孔PD值
				OSMboxPost(tMotor[MOTOR_ID1].Mbox, &msg[0]);	
			}
			ack_state = ACK_OK;
			break;
		case _CMD_READ_CalibrateRes://0X09 读取校准结果
			iPara = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			if(iPara==0)	{//读取空孔荧光最大值最小值
				msg[1].Data = (u8 *)gPD_Data.PDBaseBlue;
				msg[1].dLen = sizeof(gPD_Data.PDBaseBlue);
			}
			else if(iPara==1)	{//读取孔位置
				msg[1].Data = (u8 *)HolePos.pos;
				msg[1].dLen = sizeof(HolePos.pos);
			}
			else 
				break;
			OSMboxPost(usart.mbox, &msg[1]);
			break;
		case _CMD_FILETRANSMIT_DOWNLOAD://0X0A,//下载文件
			if(GetTransmitState() == DEF_Busy)	{
				ack_state = ACK_BUSY;
				break;
			}
			msg[0].Src = MSG_FILETRANSMIT_DOWNLOAD;
			msg[0].Data = (u8 *)(pUsart->rx_buf+pUsart->rx_idx);
			msg[0].dLen = pUsart->rx_cnt - 1;
			OSMboxPost(app_filetransmit.Mbox, &msg[0]);
			break;
		case _CMD_FILETRANSMIT_UPLOAD://0X0B 读取文件
			if(GetTransmitState() == DEF_Busy)	{
				ack_state = ACK_BUSY;
				break;
			}
			msg[0].Src = MSG_FILETRANSMIT_UPLOAD;
			msg[0].Data = (u8 *)(pUsart->rx_buf+pUsart->rx_idx);
			msg[0].dLen = pUsart->rx_cnt - 1;
			OSMboxPost(app_filetransmit.Mbox, &msg[0]);
			break;
		case _CMD_UPDATE_FW://0x0c 固件升级
			if(GetTransmitState() == DEF_Busy)	{
				ack_state = ACK_BUSY;
				break;
			}
			iPara = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			if(iPara==1)	{//主板固件升级
				msg[0].Data = (u8 *)(pUsart->rx_buf+pUsart->rx_idx);
				msg[0].dLen = pUsart->rx_cnt - 1;
				msg[0].Src = MSG_JUMP_IAP;
				OSMboxPost(app_filetransmit.Mbox, &msg[0]);
			}
			ack_state = DEF_Busy;
			break;
		case _CMD_LED_CTRL://0x0d LED灯控制
		{
			u8 led_type;
			
			led_type = LED_NONE;
			iPara = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			data = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			if(iPara==0)	{//0--蓝色灯，1—绿色灯，2—所有灯
				led_type = LED_BLUE;
			}
			else if(iPara==1)	{
				led_type = LED_GREEN;
			}
			else if(iPara==2)	{
				led_type = LED_GREEN | LED_BLUE;
			}
			FluoLED_OnOff(led_type, data);//0—灭，1—点亮
			ack_state = ACK_OK;
			break;
		}
		case _CMD_RESET_MOTOR://0X0E,//电机复位
			if(tMotor[MOTOR_ID1].status.is_run == MotorState_Run)
				ack_state = ACK_Fail;	
			else	{
				OSMboxPost(tMotor[MOTOR_ID1].Mbox, &msg[0]);
				ack_state = ACK_OK;		
			}
			break;
		case _CMD_DBG_MoveAnyPosAtReset://0x0F,//移动电机	
			if(tMotor[MOTOR_ID1].status.is_run == MotorState_Run)
				ack_state = ACK_Fail;	
			else	{
				tMotor[MOTOR_ID1].dst_pos = (s32)UsartRxGetINT16U(pUsart->rx_buf,&pUsart->rx_idx);
				OSMboxPost(tMotor[MOTOR_ID1].Mbox, &msg[0]);
				ack_state = ACK_OK;
			}
			break;
		case _CMD_GetMotorStatus://0x10,//查询电机状态
			data_buf[0] = tMotor[MOTOR_ID1].status.is_run;
			msg[1].Data = data_buf;
			msg[1].dLen = 1;
			OSMboxPost(usart.mbox, &msg[1]);
			break;
		case _CMD_GetMotorPositon://0x11,//获取电机位置
			temp = StepsToLen(&tMotor[MOTOR_ID1]);
			memcpy(data_buf,(u8 *)&temp, 2);
			msg[1].Data = data_buf;
			msg[1].dLen = 2;
			OSMboxPost(usart.mbox, &msg[1]);
			break;
		case _CMD_SetTemp:// 0x13,//设置模块温度
			iPara = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			data = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			temp = (s16)UsartRxGetINT16U(pUsart->rx_buf,&pUsart->rx_idx);
			if(iPara==0)	{//控制模块温度			
				SetTempCtrlTarget(HOLE_ID, temp);
			}else if(iPara==1)	{//控制热盖温度
				SetTempCtrlTarget(COVER_ID, temp);
			}
			if(data==0)	{//停止
				Sys.devstate = DevState_IDLE;
				Sys.devsubstate = DevSubState_Unkown;
			}else if(data==1)	{//启动
				Sys.devstate = DevState_Debug;
				Sys.devsubstate = DevSubState_DebugTemp;
			}
			ack_state = ACK_OK;
			break;
		case _CMD_GetTemp://0x14,//读取模块温度
			iPara = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			if(iPara==0)	{//读取模块温度	
				temp = GetHoleTemperature(1);				
			}
			else if(iPara==1)	{//读取热盖温度
				temp = GetCoverTemperature();
			}
			else 
				break;
			memcpy(data_buf,(u8 *)&temp, 2);
			msg[1].Data = data_buf;
			msg[1].dLen = 2;
			OSMboxPost(usart.mbox, &msg[1]);
			break;
////////////////////////////////////////////DEBUG 指令//////////////////////////////////////////////
		case _CMD_SetPIDVal:	{//0xE0 PID参数设置
			BIT32 P,I,D;
			iPara = UsartRxGetINT8U(pUsart->rx_buf,&pUsart->rx_idx);
			P.uword = UsartRxGetINT32U(pUsart->rx_buf,&pUsart->rx_idx);
			I.uword = UsartRxGetINT32U(pUsart->rx_buf,&pUsart->rx_idx);
			D.uword = UsartRxGetINT32U(pUsart->rx_buf,&pUsart->rx_idx);
			if(iPara==0)	{//设置模块PID
				SetPIDVal(PID_ID1, P.fp32, I.fp32, D.fp32);
			}
			else if(iPara==1)	{//设置热盖PID
				SetPIDVal(PID_ID2, P.fp32, I.fp32, D.fp32);
			}
			else 
				break;
			ack_state = ACK_OK;
			break;
		}
		default:
			break;
	}
	return ack_state;
}

