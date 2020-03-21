#ifndef _MOTOR_H
#define _MOTOR_H

#include "includes.h"
#include "bsp_tmc260.h"

/*
*********************************************************************************************************
*                                            GLOBAL DATA TYPES
*********************************************************************************************************
*/
#define Motor_DriverRatio             1           // ���������
#define Motor_MicroSteps              4          // ���ϸ����
#define Motor_StepsPerRound           200         // ���ÿȦ������
#define Motor_StepsPerRound_With_MicroSteps     (Motor_MicroSteps*Motor_StepsPerRound)//800
#define Motor_NumPerRound             12		  // ���ÿȦ mm
#define Motor_NumPerStep              ((FP32)Motor_NumPerRound/(FP32)Motor_StepsPerRound_With_MicroSteps)//1������mm��12/800=0.015mm
#define Motor_StepsPerum              ((FP32)Motor_StepsPerRound_With_MicroSteps/(FP32)Motor_NumPerRound)//1um���ٲ���800/12=66.6
//#define Motor_InitSpeed                0x57e3

#define Motor_Move_MAX_LEN      80//97//mm
#define Motor_Move_MAX_STEP     Motor_Move_MAX_LEN*Motor_StepsPerum  //    
//#define Motor_Timer_PSC         1
//#define Motor_Timer_CLK         (rcc_clocks.PCLK2_Frequency/Motor_Timer_PSC)
//#define DEF_FREQ_MIN            2000

#define Motor_Constant1_Steps          0//�����׶�����·��
#define Motor_Constant2_Steps          600//���׶�����·�̣�1600//С��1600������С�ڰ�Ȧ���������У���Ȧ1600����

typedef enum {
    MOTOR_ID1      = 0,
//    MOTOR_ID2      = 1
} MOTOR_ID;
#define MOTOR_ID_MIN 			MOTOR_ID1
#define MOTOR_ID_MAX 			MOTOR_ID1
#define MOTOR_ID_NUMS       	(MOTOR_ID_MAX-MOTOR_ID_MIN+1)
#define MOTOR_FLAGS_INIT		1

#define MOTOR_TO_MAX         DEF_False        // To Max
#define MOTOR_TO_MIN         DEF_True       // To Min

#define MOTOR_ID1_EN_PORT	TMC260_EN_GPIO_Port
#define MOTOR_ID1_EN_PIN	TMC260_EN_Pin
#define MOTOR_ID1_DIR_PORT	TMC260_DIR_GPIO_Port
#define MOTOR_ID1_DIR_PIN	TMC260_DIR_Pin
#define MOTOR_ID1_STEP_PORT	TMC260_STEP_GPIO_Port
#define MOTOR_ID1_STEP_PIN	TMC260_STEP_Pin

#define	Motor_MaxLimit()				HAL_GPIO_ReadPin(LimitSwitchLeft_GPIO_Port, LimitSwitchLeft_Pin)
#define	Motor_MinLimit()				HAL_GPIO_ReadPin(LimitSwitchRgiht_GPIO_Port, LimitSwitchRgiht_Pin)

enum eMotorState {
    MotorState_Stop         = 0,    // Motor State:stop
    MotorState_Run          = 1,     // Motor State:run
    MotorState_Stuck        = 2,    // motor stuck
    MotorState_Stop_Reset_Failed  = 4,//�Ҳ�����㣬��λʧ��
    MotorState_Stop_Unreachable  = 5,     // unreachable
    MotorState_Stop_MoveOutDownLimitFailed  = 6,     // z��λʱ���޷��Ƴ���������
    MotorState_Unkown = 0xff
};

enum eActionState {
    ActionState_Unknown     = 0,    // Action State:Unknown
    ActionState_Doing       = 1,    // Action State:Doing
    ActionState_OK          = 2,    // Action State:OK
    ActionState_Fail        = 3     // Action State:Fail
};

enum eMotorAbort {
	MotorAbort_Normal    = 0,       // Motor Abort:Normal
	MotorAbort_OverSteps = 1,       // Motor Abort:Over steps
	MotorAbort_Stuck   = 2,       // Motor Abort:door opened
	MotorAbort_LimitOpt  = 3,        // Motor Abort:touch limit opt
	MotorAbort_Min_LimitOpt  = 4,
	MotorAbort_Max_LimitOpt  = 5,
	MotorAbort_Reset_err,
	//MotorAbort_YReset_err,
	MotorAbort_Zero_err,
	//MotorAbort_YZero_err,
	MotorAbort_LostSteps,
};

enum eMotorAction {
    MotorAction_Unknown             = 0,    // Motor Action:Unknown
    MotorAction_Resetting           = 1,          // Motor Action:Reset doing
    MotorAction_ResetOK             = 2,          // Motor Action:Reset OK
    MotorAction_ResetFail           = 3,          // Motor Action:Reset Fail
	MotorAction_Moving              = 4, // Motor Action:Move doing
    MotorAction_MoveOK              = 5, // Motor Action:Move OK
    MotorAction_MoveFail            = 6, // Motor Action:Move Fail
};

typedef struct {
volatile    INT8U   is_run;
volatile    INT8U   action;
volatile    INT8U   abort_type;
} motor_state_t;

typedef struct _velprofile_t {
	const u16 *pVelBuf;
	u8 MaxIdx;
	u16 SaStep;//����·��
	u16 Vo;
	u16 Vmax;
	u16 AccStep;
}velprofile_t;

typedef struct Motor_t {
     MOTOR_ID     	id;
	    OS_EVENT            *Sem;
    OS_EVENT            *Mbox;
    TIM_HandleTypeDef   *tmr;                        
    motor_state_t       status;       
    INT8U               Dir;                        
    INT16S              TableIndex;                 
    INT8U               max_idx;
    INT8U		err_num;					
 
    INT8U		CurrentScale;	//����
    INT16U		micro_step;		//ϸ��	
    INT16U		speed;			//�ٶ�
    INT16U		cur_speed;
    INT16U		acceleration;	//���ٶ�
//    INT8U               lock_current;
    INT32S		CurSteps;//��ǰ�������Ƕ�ֵ

    INT32U              StepCnt;                    
    INT32U              MoveTotalSteps;      
	INT32S              SysAbsoluteOffsetStep;//ϵͳ����ƫ��������0��Ϊ��׼, 0.1umΪ��λ    
//    INT32S              SysAbsoluteOffset;
    INT32S              AccSteps;                    
    INT32U              DecSteps;
    INT32U              ConSteps;//����
//    INT32U              ConSteps2;//����
    tmc260_dev_t        *tmc260dev;
	velprofile_t *pCurve;
    void (*StepsCallback)(struct Motor_t *);    
    void (*OptCallback)(struct Motor_t *);      
} TMotor;
extern TMotor tMotor[MOTOR_ID_NUMS];

void Motor_Init(void);
u8 StartMotor(TMotor *pMotor, INT8U dir, INT32U steps,INT8U if_acc);
void StopMotor(TMotor *pMotor);
void MotorAccDec(TMotor *pMotor);
#endif

