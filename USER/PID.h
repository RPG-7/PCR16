#ifndef __PID_H__
#define __PID_H__

#include "includes.h"

#define	P1_PARAM		0.35f
#define	I1_PARAM		0.01f
#define	D1_PARAM		5.4f

#define	P2_PARAM		0.35f
#define	I2_PARAM		0.01f
#define	D2_PARAM		5.4f
typedef enum {
    PID_ID1      = 0,
    PID_ID2      = 1
} PID_ID;

#define PID_NUMS       	(PID_ID2+1)

typedef struct _PID{
    float    Kp,Ki,Kd;         // 比例系数,积分系数,微分系数
    s32      period;     // 控制周期
    s32    Target;         // 目标
    s32    LastInput;         // 实际
    s32    diff;      // 偏差值
    s32    diff_last;      // 上一个偏差值
    s32    diff_llast;      // 上上一个偏差值
	s32	OutputMin;
	s32 OutputMax;
    float integral;
	float increment;
	float PIterm;
    u8 issue_cnt;
//	u8 enable;
} _PID_t;

extern _PID_t PID[PID_NUMS];
void PIDParamInit(void);
void SetPIDVal(u8 id, float P,float I,float D);
u8 SetPIDTarget(u8 id, s32 data);
void SetPIDOutputLimits(u8 id, s32 min, s32 max);
float PIDControl(u8 id, s32 input_dat);
void StopPIDControl(u8 id);
float GetPIDIncrement(u8 id);
s32 GetPIDDiff(u8 id);
//void ClearPIDDiff(u8 id);
#endif

