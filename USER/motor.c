#include "motor.h"

struct _motor_port{
	struct _io_map en;
	struct _io_map dir;
	struct _io_map step;
};

typedef struct _motor_priv{
	struct _motor_port *pport;
}motor_priv_t;
velprofile_t VelProfile;
#define ARRLEN	40
u16 vel_arr[ARRLEN];
/*
*********************************************************************************************************
*                                       PRIVATE FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static void     UpdateMotorTimer        (MOTOR_ID id, INT16U val);
//static void     CalcSpedingProcedure    (MOTOR_ID id,INT8U if_acc);
static void UpdateMotorPWM(MOTOR_ID id, INT16U val);
static void StartMotorPWM(MOTOR_ID id);
static void StopMotorPWM(MOTOR_ID id);
static void StartMotorAccDec(MOTOR_ID id);
static void StopMotorAccDec(MOTOR_ID id);
/*
*********************************************************************************************************
*                                  PRIVATE GLOBAL CONSTANTS & VARIABLES
*********************************************************************************************************
*/

static struct _motor_port g_motor_port[MOTOR_ID_NUMS]={
	[MOTOR_ID1] = {
		.en ={MOTOR_ID1_EN_PORT ,MOTOR_ID1_EN_PIN},
		.dir={MOTOR_ID1_DIR_PORT,MOTOR_ID1_DIR_PIN},
		.step={MOTOR_ID1_STEP_PORT,MOTOR_ID1_STEP_PIN}
	},
/*	[MOTOR_ID2] = {
		.en ={MOTOR_ID2_EN_PORT ,MOTOR_ID2_EN_PIN},
		.dir={MOTOR_ID2_DIR_PORT,MOTOR_ID2_DIR_PIN},
		.step={MOTOR_ID2_STEP_PORT,MOTOR_ID2_STEP_PIN}
	},*/
};

static motor_priv_t g_motor_priv[MOTOR_ID_NUMS]={
	[MOTOR_ID1] = {
		.pport = &g_motor_port[MOTOR_ID1]
	},
/*	[MOTOR_ID2] = {
		.pport = &g_motor_port[MOTOR_ID2]
	},*/
};
TMotor tMotor[MOTOR_ID_NUMS];

#define ACC_TIME	1//����ʱ����1ms
static void VelProfile_Init(void)
{
	u8 t;
	u16 vel_profile[ARRLEN];
	
	VelProfile.Vo = 1000;
	VelProfile.Vmax = 3000;
	VelProfile.Acc = 100;
	VelProfile.SaStep = 40;
	for(t=0;;t++)	{
		vel_profile[t] = VelProfile.Acc*t + VelProfile.Vo;
		if(vel_profile[t]>=VelProfile.Vmax)	{
			VelProfile.MaxIdx = t;//���ٴ���
			break;
		}
	}
	for(t=0;t<=VelProfile.MaxIdx;t++)	{
		vel_arr[t] = 80000/vel_profile[t];
	}
	VelProfile.pVelBuf = vel_arr;
	tMotor[MOTOR_ID1].pCurve = &VelProfile;
}

void Motor_Init(void)
{
	VelProfile_Init();
}

void enable_motor(TMotor *pdev)
{
  struct _io_map const *m_en = &g_motor_port[pdev->id].en;
  
  SET_L(m_en);
}

void disable_motor(TMotor *pdev)
{
  struct _io_map const *m_en = &g_motor_port[pdev->id].en;
  
  SET_H(m_en);
}
//ֹͣ���
void StopMotor(TMotor *pMotor)
{
	StopMotorPWM(pMotor->id);
	if(pMotor->status.is_run==MotorState_Run)	{
		OSSemPost(pMotor->Sem);
		pMotor->status.is_run        = MotorState_Stop;
		StopMotorAccDec(MOTOR_ID1);//ֹͣ�Ӽ���
	}
}
//���õ������
static INT32S SetMotorDir(TMotor *pMotor,INT32S step)
{
    if(step<0){
        step = -step;
        pMotor->Dir = MOTOR_TO_MIN;
    }else{
    	pMotor->Dir = MOTOR_TO_MAX;
    }
    return step;
}

//�����Ϊ�ο��㣬��ǰ�ƶ�len����
void CalcAnyPosAtResetSteps(TMotor *pMotor,INT32S step)
{
    INT32S tmp,curSteps;

    pMotor->SysAbsoluteOffsetStep = step;
    if((pMotor->SysAbsoluteOffsetStep > Motor_Move_MAX_STEP))//x y��������г�����
		pMotor->SysAbsoluteOffsetStep = Motor_Move_MAX_STEP;
	
	curSteps = pMotor->CurSteps;
    tmp = pMotor->SysAbsoluteOffsetStep - curSteps;//Ŀ��λ���뵱ǰλ�õĲ� ����0ǰ��
    pMotor->MoveTotalSteps = SetMotorDir(pMotor,tmp);
}
//����Ӽ��ٵ� ���һ��ConSteps����ǿ������
static void CalcSpedingProcedure(TMotor *pMotor,INT8U if_acc)
{
    s32 updn_steps;

    if(!if_acc || pMotor->MoveTotalSteps <= pMotor->ConSteps)	{//ǿ������ ���� �ܾ���С��ĳֵ
    	pMotor->AccSteps = -1;//��ʾ�����˶�		
    }
	else	{
		updn_steps = pMotor->MoveTotalSteps - pMotor->ConSteps;//��ȥ�������·��
		pMotor->AccSteps = 0;//��㿪ʼ��������
		if(updn_steps <= (pMotor->pCurve->SaStep*2))    {//������ٵ�
			pMotor->DecSteps = updn_steps/2 + pMotor->AccSteps;
		}
		else        {
			pMotor->DecSteps = (updn_steps - pMotor->pCurve->SaStep) + pMotor->AccSteps;
		}
	}
}
//��������ж�
static void MotorArrivedCheck(TMotor *pMotor)
{
	if(pMotor->StepCnt >= pMotor->MoveTotalSteps)	{
		StopMotor(pMotor);		
	}
}
u8 teat;
//����Ӽ��� �ȼ��� 1ms
void MotorAccDec(TMotor *pMotor)
{
	u16 arr;
	
	if(pMotor->status.is_run != MotorState_Run||pMotor->AccSteps == -1)	{
		return;		
	}

	if(pMotor->StepCnt >= pMotor->DecSteps)	{//������ٵ� ����
		if(pMotor->TableIndex > 0)
			pMotor->TableIndex --;		
	}
	else if(pMotor->TableIndex < pMotor->pCurve->MaxIdx)	{//û������ٵ� ����
		pMotor->TableIndex ++;
	}
	else {//����
		teat = 0;
		return;
	}
	arr = pMotor->pCurve->pVelBuf[pMotor->TableIndex];
	UpdateMotorPWM(pMotor->id,arr);	
}

u8 StartMotor(TMotor *pMotor, INT8U dir, INT32U steps,INT8U if_acc)
{
    INT8U err;
//    INT32U len;

   if(pMotor->status.is_run == MotorState_Run)
	   return 0;
   struct _io_map const *m_dir = &g_motor_port[pMotor->id].dir;
    pMotor->StepCnt       = 0;
	if (steps)
    {
        pMotor->Dir = dir;
        if(pMotor->Dir) {//ǰ��
            if(!Motor_MaxLimit())//�����޿���λ��
                goto _end;
            SET_H(m_dir);
        }
        else    {//����
            if(!Motor_MinLimit())//�����޿���λ��
                goto _end;
            SET_L(m_dir);
        }
        enable_motor(pMotor);
        pMotor->MoveTotalSteps    = steps;
		pMotor->status.is_run = MotorState_Run;
        CalcSpedingProcedure(pMotor,if_acc);
        pMotor->TableIndex    = 0;
        pMotor->StepsCallback = &MotorArrivedCheck;
        StartMotorPWM(pMotor->id);
		if(if_acc)//�Ƿ����Ӽ���
			StartMotorAccDec(pMotor->id);
		OSSemPend(pMotor->Sem, 0, &err);//�ȴ��¼�����
		disable_motor(pMotor);
	}
_end:
	
	return 1;
}

static void StartMotorAccDec(MOTOR_ID id)
{
	__HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
	__HAL_TIM_ENABLE(&htim6);
	__HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
}

static void StopMotorAccDec(MOTOR_ID id)
{
	__HAL_TIM_DISABLE(&htim6);
}

static void StopMotorPWM(MOTOR_ID id)
{
	HAL_TIM_PWM_Stop(tMotor[id].tmr, TIM_CHANNEL_3);
}

static void UpdateMotorPWM(MOTOR_ID id, INT16U val)
{
//	__HAL_TIM_SET_COUNTER(tMotor[id].tmr, 0);
    __HAL_TIM_SET_AUTORELOAD(tMotor[id].tmr, val);
    __HAL_TIM_SET_COMPARE(tMotor[id].tmr, TIM_CHANNEL_3, val/2);
}

static void StartMotorPWM(MOTOR_ID id)
{
	__HAL_TIM_CLEAR_FLAG(tMotor[id].tmr, TIM_FLAG_UPDATE);
	UpdateMotorPWM(id, tMotor[id].pCurve->pVelBuf[tMotor[id].TableIndex]);
	HAL_TIM_PWM_Start(tMotor[id].tmr, TIM_CHANNEL_3);	
}
