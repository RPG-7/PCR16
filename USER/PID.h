#ifndef __PID_H__
#define __PID_H__

#include "includes.h"

typedef enum {
    PID_ID1      = 0,
    PID_ID2      = 1
} PID_ID;

#define PID_NUMS       	(PID_ID2+1)

typedef struct _PID{
    float    Kp,Ki,Kd;         // ����ϵ��,����ϵ��,΢��ϵ��
    s32      period;     // ��������
    s32    Target;         // Ŀ��
    s32    LastInput;         // ʵ��
    s32    diff;      // ƫ��ֵ
    s32    diff_last;      // ��һ��ƫ��ֵ
    s32    diff_llast;      // ����һ��ƫ��ֵ
	s32	OutputMin;
	s32 OutputMax;
    float integral;
	float increment;
	float PIterm;
    u8 issue_cnt;
} _PID_t;

extern _PID_t PID[PID_NUMS];
void PIDParamInit(void);
void SetPIDVal(u8 id, float P,float I,float D);
void SetPIDTarget(u8 id, s32 data);
void SetPIDOutputLimits(u8 id, s32 min, s32 max);
float PID_control(u8 id, s32 input_dat);
float GetPIDIncrement(u8 id);
s32 GetPIDDiff(u8 id);
void ClearPIDDiff(u8 id);
#endif

