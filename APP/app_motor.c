#include "app_motor.h"
#include "PD_DataProcess.h"
#include "app_spiflash.h"
//堆栈
__align(4) OS_STK  TASK_MOTOR_STK[STK_SIZE_MOTOR]; //任务堆栈声?
extern  message_pkt_t    msg_pkt_motor;
static void AppMotorTask (void *parg);

void AppMotorInit (void)
{
	OSTaskCreate(AppMotorTask,  (void * )0, (OS_STK *)&TASK_MOTOR_STK[STK_SIZE_MOTOR-1], TASK_PRIO_MOTOR);
}

static void MotorDatInit (void)
{
	tMotor[MOTOR_ID1].id                  = MOTOR_ID1;
    //tMotor[X_MOTOR_ID].PWMBASE             = PWM0_BASE;
    tMotor[MOTOR_ID1].Sem                 = OSSemCreate(0);
    tMotor[MOTOR_ID1].Mbox                = OSMboxCreate((void *)0);
    tMotor[MOTOR_ID1].status.is_run       = MotorState_Stop;
    tMotor[MOTOR_ID1].status.action       = MotorAction_Resetting;
    tMotor[MOTOR_ID1].status.abort_type   = MotorAbort_Normal;
    tMotor[MOTOR_ID1].Dir                 = MOTOR_TO_MIN;
	tMotor[MOTOR_ID1].tmr = &htim3;
	tMotor[MOTOR_ID1].tmc260dev = TMC260_get_dev((TMC260_ID)MOTOR_ID1);
	tMotor[MOTOR_ID1].StepsCallback = &MotorArrivedCheck;
	tMotor[MOTOR_ID1].if_acc             = DEF_False;
	tMotor[MOTOR_ID1].ConSteps = 10;//匀速步数
	tMotor[MOTOR_ID1].CurSteps = 0;
}

static void MotorReset(void)
{
    tMotor[MOTOR_ID1].status.action     = MotorAction_Resetting;
	if(!Motor_MinLimit())	{//已经在下限位 前进
		StartMotor(&tMotor[MOTOR_ID1], MOTOR_TO_MAX, 30*Motor_StepsPerum, DEF_False);//前进10mm
		OSTimeDly(500);
	}
	tMotor[MOTOR_ID1].status.abort_type = MotorAbort_Normal;
//	CalcAnyPosAtResetSteps(&tMotor[MOTOR_ID1], Motor_Move_MAX_STEP);
	StartMotor(&tMotor[MOTOR_ID1], MOTOR_TO_MIN, Motor_Move_MAX_STEP, DEF_False);
	if(tMotor[MOTOR_ID1].status.abort_type == MotorAbort_Min_LimitOpt)	{//复位过程碰到零点 成功
		tMotor[MOTOR_ID1].status.action     = MotorAction_ResetOK;
		SysError.Y1.bits.b1 = DEF_Active;//零点信号正常
	}
	else 	{
		tMotor[MOTOR_ID1].status.action     = MotorAction_ResetFail;
		SysError.Y1.bits.b1 = DEF_Inactive;//未找到电机零点
	}
	OSTimeDly(500);//等待一段时间清零位置 消除过充误差
	tMotor[MOTOR_ID1].CurSteps = 0;
}

static void DealUsartMessage(message_pkt_t *pmsg)
{
    INT8U  cmd;

    cmd = pmsg->Cmd;

    switch (cmd)
	{
		case _CMD_RESET_MOTOR://0X0E,//电机复位		
			MotorReset();
			break;
		case _CMD_DBG_MoveAnyPosAtReset:
		{
			u32 steps = LenToSteps(&tMotor[MOTOR_ID1], tMotor[MOTOR_ID1].dst_pos);
			CalcAnyPosAtResetSteps(&tMotor[MOTOR_ID1], steps);
			StartMotor(&tMotor[MOTOR_ID1], tMotor[MOTOR_ID1].Dir, tMotor[MOTOR_ID1].MoveTotalSteps, DEF_True);
			break;
		}
		default:
			break;
	}
}
extern u8 HolePositionCaliFlag;
static void AppMotorTask (void *parg)
{
    INT8U err;
    message_pkt_t *msg;
	
	MotorDatInit();
	Motor_Init();
	__HAL_TIM_ENABLE_IT(tMotor[MOTOR_ID1].tmr, TIM_IT_UPDATE);
	TMC260_install(tMotor[MOTOR_ID1].tmc260dev);
	TMC260_read_status(tMotor[MOTOR_ID1].tmc260dev);
//	MotorReset();
	
	for(;;)
    {
		msg = (message_pkt_t *)OSMboxPend(tMotor[MOTOR_ID1].Mbox, 0, &err);
		/*if(err==OS_ERR_TIMEOUT)	{
			StartMotor(&tMotor[MOTOR_ID1], MOTOR_TO_MAX, Motor_Move_MAX_STEP, DEF_True);
			OSTimeDly(1000);
			StartMotor(&tMotor[MOTOR_ID1], MOTOR_TO_MIN, Motor_Move_MAX_STEP, DEF_True);
		}
		else	{*/
			if(msg->Src == MSG_CollectHolePD_EVENT)	{//采集实验过程孔PD值
				gPD_Data.ch = LED_BLUE;//蓝光扫描
				FluoLED_OnOff(LED_BLUE, DEF_ON);
				StartCollPDData();//孔PD数据采集准备
				StartMotor(&tMotor[MOTOR_ID1], MOTOR_TO_MAX, Motor_Move_MAX_STEP, DEF_True);
				OSTimeDly(50);
				gPD_Data.ch = LED_GREEN;//绿光扫描
				FluoLED_OnOff(LED_GREEN, DEF_ON);
				StartMotor(&tMotor[MOTOR_ID1], MOTOR_TO_MIN, Motor_Move_MAX_STEP, DEF_True);
				StopCollPDData();
			}
			else if(msg->Src == MSG_CaliHolePostion_EVENT)	{//使用蓝光扫描 校准孔位置
				Sys.devstate = DevState_Debug;
				StartCaliHolePosition();//开启孔位置校准
				StartMotor(&tMotor[MOTOR_ID1], MOTOR_TO_MAX, (u32)(25*Motor_StepsPerum), DEF_False);
				StopCaliHolePosition();//停止孔位置校准
				OSTimeDly(100);
				StartMotor(&tMotor[MOTOR_ID1], MOTOR_TO_MIN, (u32)(25*Motor_StepsPerum), DEF_True);//回零点
				CalcHolePositon();//计算孔位置
				Sys.devstate = DevState_IDLE;
			}
			else if(msg->Src == MSG_CaliHolePDBase_EVENT)	{//使用蓝光LED扫描 校准空孔PD本底信号
				Sys.devstate = DevState_Debug;
				gPD_Data.ch = LED_BLUE;//蓝光扫描
				FluoLED_OnOff(LED_BLUE, DEF_ON);
				StartCaliHolePDBase();//校准空孔PD本底信号
				HolePos.idx = 0;
				StartMotor(&tMotor[MOTOR_ID1], MOTOR_TO_MAX, Motor_Move_MAX_STEP, DEF_True);
				CalcHolePDBase(LED_BLUE);//计算蓝光本底信号	
				gPD_Data.ch = LED_GREEN;//绿光扫描
				FluoLED_OnOff(LED_GREEN, DEF_ON);
				OSTimeDly(100);
				StartMotor(&tMotor[MOTOR_ID1], MOTOR_TO_MIN, tMotor[MOTOR_ID1].CurSteps, DEF_True);
				StopCaliHolePDBase();//校准结束				
				CalcHolePDBase(LED_GREEN);//计算绿光本底信号		
				Sys.devstate = DevState_IDLE;
//				msg_pkt_motor.Src = MSG_WriteCaliRes;//校准结果写入文件
//				OSQPost(spiflash.MSG_Q, &msg_pkt_motor);
			}
			else if (msg->Src == USART_MSG_RX_TASK)	{
				DealUsartMessage(msg);
			}
		//}
	}
}


