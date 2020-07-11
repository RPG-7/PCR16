#include "PID.h"

_PID_t PID[PID_NUMS];
void PIDParamInit(void)
{	
	memset(&PID[PID_ID1],0,sizeof(PID[PID_ID1]));
	memset(&PID[PID_ID2],0,sizeof(PID[PID_ID2]));
}
//����pid���Ʋ���
void SetPIDVal(u8 id, float P,float I,float D)
{
	_PID_t *pPid = &PID[id];

	pPid->Kp = P;
	pPid->Ki = I;
	pPid->Kd = D;
//	ClearPIDDiff(id);
}
//����pid����Ŀ��
u8 SetPIDTarget(u8 id, s32 data)
{
	_PID_t *pPid;
	
	if(id == PID_ID1)	{
		if(data<HOLE_TEMPMIN || data>HOLE_TEMPMAX)	
			return 0;
	}
	else if(id == PID_ID2)	{
		if(data<HEATCOVER_TEMPMIN || data>HEATCOVER_TEMPMAX)
			return 0;
	}
	else 
		return 0;
	pPid = &PID[id];
	pPid->Target = data;
//	pPid->enable = DEF_True;
	return 1;
}

void SetPIDOutputLimits(u8 id, s32 min, s32 max)
{
	_PID_t *pPid = &PID[id];
	if(min>max)		return;
	pPid->OutputMin = min;
	pPid->OutputMax = max;
}

#if 0
//λ��ʽ���㷨��������ʽ Pout(t)=Kp*e(t)+Ki*��e(t)+Kd*(e(t)-e(t-1));
//����Ա�׼��ʽ�����µ�����
//1. ����΢����, Ϊ������һ�ֱ���Ϊ��΢�ֳ����������,��סlastInput�������Ǽ�סlastError
//2. ���й��̿��Ը���PID�������ü���������������ұ仯���ø��ϻ���������滻ƫ����ͱ������� Ki * ƫ�� ���
//3. ǯλ����������, �������ֱ���
//PID���� ����ֵ��set_dat  ʵ��ֵ��input_dat
float PIDControl(u8 id, s32 input_dat)
{
	s32 dInput;
	_PID_t *pPid = &PID[id];

	pPid->diff = pPid->Target - input_dat;// �������
	pPid->integral += (pPid->Ki*pPid->diff);//����ֵ
	if(pPid->integral> pPid->OutputMax) pPid->integral= pPid->OutputMax;//�������ֱ���
    else if(pPid->integral< pPid->OutputMin) pPid->integral= pPid->OutputMin;
	dInput = input_dat - pPid->LastInput;//����΢�ֳ��
	pPid->increment = pPid->Kp*pPid->diff + pPid->integral - pPid->Kd*dInput;//����λ����
	if(pPid->increment> pPid->OutputMax) pPid->increment = pPid->OutputMax;//�������ֱ���
    else if(pPid->increment < pPid->OutputMin) pPid->increment = pPid->OutputMin;
//	pPid->diff_last = pPid->diff;// �ϴ����
	pPid->LastInput = input_dat;//�ϴ�����ֵ	
	return pPid->increment;
}
#else
//���������㹫ʽ��Pdlt=Kp*(e(t)-e(t-1))+Ki*e(t)+Kd*(e(t)-2*e(t-1)+e(t-2));
float PIDControl(u8 id, s32 input_dat)
{
	_PID_t *pPid = &PID[id];
	s32 pError,iError,dError;
	float P,I,D,PIterm;
	s32 dInput,absdiff;
	
//	if(pPid->enable==DEF_False)
//		return 0;
	pPid->diff_llast = pPid->diff_last;// ���ϴ����
	pPid->diff_last = pPid->diff;// �ϴ����
	pPid->diff = pPid->Target - input_dat;// �������
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
	if(pPid->increment> pPid->OutputMax) {//�������ֱ���
		if(pPid->diff <= 0)
			iError = pPid->diff;
		if(absdiff<300)	{//��Ŀ����3�� ���ñ������� ��������
			dInput = input_dat - pPid->LastInput;
			pPid->PIterm -= P * dInput;
			PIterm = pPid->PIterm;
		}else {
			pPid->PIterm = 0;
			PIterm = P*pError;
		}
	}
    else if(pPid->increment < pPid->OutputMin) {
		if(pPid->diff >= 0)
			iError = pPid->diff;
		if(absdiff<300)	{//��Ŀ����3�� ���ñ������� ��������
			dInput = input_dat - pPid->LastInput;
			pPid->PIterm -= P * dInput;
			PIterm = pPid->PIterm;
		}else {
			pPid->PIterm = 0;
			PIterm = P*pError;
		}
	}
	else {
		iError = pPid->diff;
		pPid->PIterm = 0;
		PIterm = P*pError;
	}
	pPid->LastInput = input_dat;//�ϴ�����ֵ	
	pPid->increment += PIterm + I*iError + D*dError; //	temp = pPid->Kp * pError + pPid->Ki * iError + pPid->Kd * dError;

	return pPid->increment;
}
#endif
void StopPIDControl(u8 id)
{
	_PID_t *pPid = &PID[id];
	
//	pPid->enable = DEF_False;
	pPid->diff = 0;
	pPid->diff_last = 0;
	pPid->diff_llast = 0;
	pPid->integral = 0;
	pPid->LastInput = 0;
	pPid->increment = 0;
	pPid->PIterm = 0;
}

//��ȡpid�������
s32 GetPIDDiff(u8 id)
{
	_PID_t *pPid = &PID[id];
	
	return pPid->diff;
}
//��ȡpid�������
float GetPIDIncrement(u8 id)
{
	_PID_t *pPid = &PID[id];
	
	return pPid->increment;
}
