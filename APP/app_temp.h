#ifndef __APP_TEMP_H__
#define __APP_TEMP_H__

#include "includes.h"
#include "TempCalc.h"

#define	HOLE_TEMP		0
#define	COVER_TEMP		1

#define	PIDCTRL_NUM		2//��Ҫpid��������


typedef struct _pid_ctrl	{
	u8 PIDid;
	TIM_HandleTypeDef *pTECPWM;	
	u8 TimCH;
	u16 TimPluse;
	u8 DutyMax;
	s16 target_t;//Ŀ���¶� 0.1
	float PIDParam;//tec pwmռ�ձ�
}pid_ctrl_t;

typedef struct _app_temp	{
	OS_EVENT           *MSG_Q;
	OS_EVENT           *lock;
	
	s32 current_t[TEMP_ID_NUMS];//��ǰ�¶� 0.1
//	pid_ctrl_t *pPid;
}_app_temp_t;

void AppTempInit (void);

#endif

