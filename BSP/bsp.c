#include "bsp.h"
//#include "tim.h"
#include "bsp_spi.h"
#include "bsp_w25qxx.h"
//#include "bsp_ad7091.h"
#include "sys_data.h"

////////////////////////////////////
//IO���ú���
void	bsp_init(void)
{
	SysDataInit();
	
	bsp_spi_init();
	BSP_W25Qx_Init();
//	BspAD7091Init();
	//bsp_mlx90614_init();
}
//io����
void ioconfig(const struct _io_map *pio, cpu_bool_t sw)
{
    GPIO_InitTypeDef  GPIO_InitStructure={0};

    GPIO_InitStructure.Pin   = pio->pin;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Mode  = (sw == DEF_OUT)? GPIO_MODE_OUTPUT_PP: GPIO_MODE_INPUT;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(pio->port, &GPIO_InitStructure);
}
////�޸Ĵ��ڲ�����
//void UartBaudrateSet(UART_HandleTypeDef *phuart, u32 baudrate)
//{
//	__HAL_UART_DISABLE(phuart);
//	 phuart->Init.BaudRate = baudrate;
//	UART_SetConfig(phuart);
//	__HAL_UART_ENABLE(phuart);
//}
////ʹ��time6����us��ʱ
//void delay_us(u16 us)
//{
//#if OS_CRITICAL_METHOD == 3
//	OS_CPU_SR   cpu_sr = 0;
//#endif
//  uint16_t value;
//	
//	OS_ENTER_CRITICAL();
//	value = 0xffff-us;
//	__HAL_TIM_SET_COUNTER(&htim6,value);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=0xfffe);
//	HAL_TIM_Base_Stop(&htim6);
//	OS_EXIT_CRITICAL();
//}

////���ȵ���һ��
//void SysStandbyEnter(void)
//{
//	sys.state |= SYSSTATE_STANDBYMODE;
//}

//void SysStandbyQuit(void)
//{
//	sys.StandbyFlag = 0;
//	sys.state &= ~SYSSTATE_STANDBYMODE;	
//}

////ϵͳ�ػ�
//void SysShutDown(void)
//{
//	OSTimeDly(500);
//	SlaveboardPowerDisable();
//	DisplayPowerDisable();
//	UsbPowerDisable();
//	SysPowerDisable();
//}
////ϵͳ��λ
//void SoftReset(void)
//{
//	__set_FAULTMASK(1);      // �ر������ж�
//	NVIC_SystemReset();// ��λ
//}
//#define 	RunIAPKeyword               0xA5A55A5A
//#define		ApplicationUpgradeKeywordAddr		0x080FF800
//void FWUpdate_reboot(void)
//{
//	FLASH_EraseInitTypeDef f;
//	f.TypeErase = FLASH_TYPEERASE_PAGES;
//	f.Page = 511;
//	f.NbPages = 1;
//	f.Banks = FLASH_BANK_2;
//	//����PageError
//	uint32_t PageError = 0;
//	HAL_FLASH_Unlock();
//	HAL_FLASHEx_Erase(&f, &PageError);	//���ò�������	
//	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,ApplicationUpgradeKeywordAddr, RunIAPKeyword) == HAL_OK) {
//		HAL_FLASH_Lock();
//		SYS_PRINTF("SYS Reboot");
//		OSTimeDly(1000);
//		SoftReset();//RUN IAP
//	}
//	HAL_FLASH_Lock();
//}
