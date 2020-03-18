#ifndef __APP_TEMP_H__
#define __APP_TEMP_H__

#include "includes.h"
#include "TempCalc.h"

#define	HOLE_TEMP		0
#define	COVER_TEMP		1

#define	PIDCTRL_NUM		2//��Ҫpid��������
#define	TECPWMDUTY_MAX		50//TEC pwmռ�ձ����ֵ

typedef struct _app_temp	{
	OS_EVENT           *MSG_Q;
	OS_EVENT           *lock;
	TIM_HandleTypeDef *pTECPWM;
	s16 target_t[PIDCTRL_NUM];//Ŀ���¶� 0.1
	s16 current_t[TEMP_ID_NUMS];//��ǰ�¶� 0.1
	s16 duty[PIDCTRL_NUM];//tec pwmռ�ձ�
}_app_temp_t;

void AppTempInit (void);

#endif

