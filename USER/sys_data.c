#include "sys_data.h"

_sys_t Sys;
_syserror_t SysError;
//_sys_data_t SysData;
RTC_TIME_ST SysTime;
tlsf_t UserMem;
void SysDataInit(void)
{
	UserMem = tlsf_create_with_pool((void *)0x20015000, 0x3000);//�ڴ�0x15000 - 0x18000 ����12KB�ڴ�ʹ��tlsf����
	
	SysError.Y1.ubyte = 0x0;//���Դ�������Ĺ��� 
	SysError.Y2.ubyte = 0;//�������
//	SysData.HeatCoverTemp = 0;
//	SysData.PD_1 = 0;
//	SysData.PD_2 = 0;
	
	Sys.state = SysState_None;
	Sys.devstate = DevState_IDLE;
	
	SysTime.tm_year = 2019;
	SysTime.tm_mon = 7;
	SysTime.tm_mday = 17;
	SysTime.tm_hour = 17;
	SysTime.tm_min = 44;
	SysTime.tm_sec = 20;
	SysTime.tm_wday = 3;
}

