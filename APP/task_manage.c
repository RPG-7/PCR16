/* Includes -----------------------------------------------------------------*/
#include "task_manage.h"
#include "app_udisk.h"
#include "app_spiflash.h"
#include "app_display.h"
//#include "sys_monitor.h"
//#include "globalvariable.h"
#include "app_motor.h"
//#include "app_usart.h"
#include "app_ad.h"
//////////////////////////////////////////////////////////
__align(4) OS_STK  TASK_START_STK[STK_SIZE_START]; //�����ջ����

//////////////////////////////////////////////////////////
static void TaskCreateOther(void);             //�����������������������
static void TaskStart(void * ppdata);

void CreatMainTask(void)
{				
	OSTimeSet(0);				
	OSInit(); 
	OSTaskCreate(TaskStart,  									//start_task??
							(void*)0,    									//??
							(OS_STK*)&TASK_START_STK[STK_SIZE_START-1], 	//??????
							TASK_PRIO_START);  								//?????
	OSStart();  
}
//////////////////////////////////////////////////////////
//��ʼ���� ����������ٴ�����������
static void TaskStart(void * ppdata)
{
	//OS_CPU_SR cpu_sr;
	ppdata = ppdata;

	//OS_ENTER_CRITICAL();
	__set_PRIMASK(0);
    //ʹ��ucos ��ͳ������
    #if (OS_TASK_STAT_EN > 0)
    OSStatInit();                       //ͳ�������ʼ������   
    #endif  
      
    SysDataInit();                   //�û�Ӧ�ó���ȫ�ֱ�����ʼ��
	bsp_init();                              //����Ӳ����ʼ��
//	SysAlarmDataInit();
	msg_init();
    TaskCreateOther();			        //��������������
    OSTaskSuspend(OS_PRIO_SELF);	    //suspend but not delete ��������
		//OS_EXIT_CRITICAL();
}

/////////////////////////////////////////////////////////////////////////////
//���������������� �������ҽ����˸�LED��˸����������
static void TaskCreateOther(void)
{
	AppADInit();	
//	AppSensorBoardInit();	
//	AppKeyScanInit();
	AppMotorInit();		
//	AppDisplayInit();
//	AppPerfusemonitorInit();
//	AppSysmonitorInit();
	AppSpiFlashInit();
	AppUSBInit();	
}


      
