#include "sys_data.h"

_sys_data_t SysData;
tlsf_t UserMem;
void SysDataInit(void)
{
	UserMem = tlsf_create_with_pool((void *)0x20015000, 0x3000);//�ڴ�0x15000 - 0x18000 ����12KB�ڴ�ʹ��tlsf����
	
	SysData.HeatCoverTemp = 0;
	SysData.PD_1 = 0;
	SysData.PD_2 = 0;
}

