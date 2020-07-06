#include "PID.h"


_PID_t PID[PID_NUMS];
void PIDParamInit(void)
{	
	memset(&PID[PID_ID1],0,sizeof(PID[PID_ID1]));
	memset(&PID[PID_ID2],0,sizeof(PID[PID_ID2]));
}
//设置pid控制参数
void SetPIDVal(u8 id, float P,float I,float D)
{
	_PID_t *pPid = &PID[id];

	pPid->Kp = P;
	pPid->Ki = I;
	pPid->Kd = D;
//	ClearPIDDiff(id);
}
//设置pid控制目标
void SetPIDTarget(u8 id, s32 data)
{
	_PID_t *pPid = &PID[id];
	
	pPid->Target = data;
}

void SetPIDOutputLimits(u8 id, s32 min, s32 max)
{
	_PID_t *pPid = &PID[id];
	if(min>max)		return;
	pPid->OutputMin = min;
	pPid->OutputMax = max;
}

#if 0
//位置式计算法，基本公式 Pout(t)=Kp*e(t)+Ki*∑e(t)+Kd*(e(t)-e(t-1));
//这里对标准公式做以下调整：
//1. 调整微分项, 为了消除一种被称为“微分冲击”的现象,记住lastInput，而不是记住lastError
//2. 运行过程可以更改PID参数，让计算结果不会产生剧烈变化：用复合积分项变量替换偏差求和变量，对 Ki * 偏差 求和
//3. 钳位积分项和输出, 消除积分饱和
//PID控制 设置值：set_dat  实际值：input_dat
float PID_control(u8 id, s32 input_dat)
{
	s32 dInput;
	_PID_t *pPid = &PID[id];

	pPid->diff = pPid->Target - input_dat;// 本次误差
	pPid->integral += (pPid->Ki*pPid->diff);//积分值
	if(pPid->integral> pPid->OutputMax) pPid->integral= pPid->OutputMax;//消除积分饱和
    else if(pPid->integral< pPid->OutputMin) pPid->integral= pPid->OutputMin;
	dInput = input_dat - pPid->LastInput;//消除微分冲击
	pPid->increment = pPid->Kp*pPid->diff + pPid->integral - pPid->Kd*dInput;//计算位置量
	if(pPid->increment> pPid->OutputMax) pPid->increment = pPid->OutputMax;//消除积分饱和
    else if(pPid->increment < pPid->OutputMin) pPid->increment = pPid->OutputMin;
//	pPid->diff_last = pPid->diff;// 上次误差
	pPid->LastInput = input_dat;//上次输入值	
	return pPid->increment;
}
#else
//增量法计算公式：Pdlt=Kp*(e(t)-e(t-1))+Ki*e(t)+Kd*(e(t)-2*e(t-1)+e(t-2));
float PID_control(u8 id, s32 input_dat)
{
	_PID_t *pPid = &PID[id];
	s32 pError,iError,dError;
	float P,I,D,PIterm;
	s32 dInput,absdiff;
	
	pPid->diff_llast = pPid->diff_last;// 上上次误差
	pPid->diff_last = pPid->diff;// 上次误差
	pPid->diff = pPid->Target - input_dat;// 本次误差
	pError = pPid->diff - pPid->diff_last;
	iError = 0;
	dError = pPid->diff - 2*pPid->diff_last + pPid->diff_llast;
	absdiff = abs(pPid->diff);
	if(absdiff <= 50)	{
		P = pPid->Kp*0.6f;
		I = pPid->Ki*0.5f;
		D = pPid->Kd*0.5f;
	}
	else	{
		P = pPid->Kp;
		I = pPid->Ki;
		D = pPid->Kd;
	}
	if(pPid->increment> pPid->OutputMax) {//消除积分饱和
		if(pPid->diff <= 0)
			iError = pPid->diff;
		dInput = input_dat - pPid->LastInput;
		pPid->PIterm -= P * dInput;
		PIterm = pPid->PIterm;
	}
    else if(pPid->increment < pPid->OutputMin) {
		if(pPid->diff >= 0)
			iError = pPid->diff;
		dInput = input_dat - pPid->LastInput;//误差大于1度 启用比例抑制 消除超调
		pPid->PIterm -= P * dInput;
		PIterm = pPid->PIterm;
	}
	else {
		iError = pPid->diff;
		pPid->PIterm = 0;
		PIterm = P*pError;
	}
	pPid->LastInput = input_dat;//上次输入值	
	pPid->increment += PIterm + I*iError + D*dError; //	temp = pPid->Kp * pError + pPid->Ki * iError + pPid->Kd * dError;

	return pPid->increment;
}
#endif
void ClearPIDDiff(u8 id)
{
	_PID_t *pPid = &PID[id];
	
	pPid->diff = 0;
	pPid->diff_last = 0;
	pPid->diff_llast = 0;
	pPid->integral = 0;
	pPid->LastInput = 0;
	pPid->increment = 0;
	pPid->PIterm = 0;
}

//获取pid本次误差
s32 GetPIDDiff(u8 id)
{
	_PID_t *pPid = &PID[id];
	
	return pPid->diff;
}
//获取pid本次误差
float GetPIDIncrement(u8 id)
{
	_PID_t *pPid = &PID[id];
	
	return pPid->increment;
}
