/*
*********************************************************************************************************
*	                                  
*	ģ������ : printfģ��    
*	�ļ����� : bsp_printf.c
*	��    �� : V2.0
*	˵    �� : ʵ��printf��scanf�����ض��򵽴���1����֧��printf��Ϣ��USART1
*				ʵ���ض���ֻ��Ҫ����2������:
*				int fputc(int ch, FILE *f);
*				int fgetc(FILE *f);
*				����KEIL MDK������������ѡ������Ҫ��MicorLibǰ��򹳣����򲻻������ݴ�ӡ��USART1��
*				
*				���cģ���޶�Ӧ��h�ļ���
*
*	�޸ļ�¼ :
*		�汾��  ����       ����    ˵��
*		v1.0    2013-11-24 armfly  �׷�
*
*	Copyright (C), 2013-2014,
*
*********************************************************************************************************
*/

#include "includes.h"
#include <stdio.h>

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
int _sys_exit(int x) 
{ 
	x = x; 
	return 0;
} 
/*
*********************************************************************************************************
*	�� �� ��: fputc
*	����˵��: �ض���putc��������������ʹ��printf�����Ӵ���1��ӡ���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int fputc(int ch, FILE *f)
{
	/* дһ���ֽڵ�USART1 */
	while((USART1->ISR &0X40)==0);//ѭ������,ֱ���������   
	USART1->TDR  = (uint8_t) ch;  
	return ch;
}

/*
*********************************************************************************************************
*	�� �� ��: fgetc
*	����˵��: �ض���getc��������������ʹ��scanff�����Ӵ���1��������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int fgetc(FILE *f)
{
	/* �ȴ�����1�������� */
	//while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

	//return (int)USART_ReceiveData(USART2_Handler);
	return 0;
}

void _ttywrch(int ch)
{
}