#ifndef __TIM_PROGRAM_H__
#define __TIM_PROGRAM_H__

#include "stm32l4xx_hal.h"

typedef struct _SYS_TIM
{
    volatile uint32_t  SumNumber;       //�ۼӼ���
    volatile uint32_t  SumMs;          //�ۼ�ms��
    volatile uint32_t  SumSec;         //�ۼ�����
    volatile uint32_t  SumMinute;      //�ۼӷ�����
}SYS_TIM;
extern SYS_TIM  SysTim;
//extern struct SYS_TIM  deviceFlowTim;
//extern struct SYS_TIM  deviceSendDataTim;
extern volatile uint16_t	_5MS_Event;
extern volatile uint16_t	_10MS_Event;
extern volatile uint16_t	_100MS_Event;
extern volatile uint16_t	_1S_Event;
extern volatile uint16_t	_2S_Event;

void TIMDataInit(void);
void TIM_Event(void);

#endif
