#ifndef APP_SYSTEM_UPDATE_H
#define APP_SYSTEM_UPDATE_H

#include "includes.h"
#include "protocol.h"

#define	TransmitFileMaxSize 	102400//100kb

typedef enum {
    UpdateCmd_Query     = 0x01,     // ��ѯ״̬
    UpdateCmd_Start     = 0x02,     // ��������
    UpdateCmd_Transmit  = 0x03,     // ����bin�ļ�
    UpdateCmd_End       = 0x04,     // �������
    UpdateCmd_JumpToIap = 0x05,     // jump to IAP
    UpdateCmd_JumpToApp = 0x06      // jump to APP
} update_cmd_t;

typedef struct _update_ctrl {
    INT8U   state;                          // Firmware bin��д״̬
    INT8U   ack;                            // Firmware bin��дACK
    INT8U   burn_start;                     // ��д����
    INT8U   burn_last;                      // ��д����
    INT8U   update_is_ok;                   // ��д�Ƿ�ɹ�
    INT16U  burn_cnt;                       // һ����д���ռ�����
    INT8U  *const buf_addr;                 // Firmware bin������ʼ��ַ
    INT32U  burn_addr;                      // ��д��ַ
    INT32U const size_max;                  // �ɴ洢�����
    INT32U  size_total;                     // Firmware bin�ļ��Ĵ�С
    INT32U  size_cnt;                       // Firmware bin�ļ�������
    INT32U  frame_cnt;                      // Firmware bin֡������
    INT32U  calcsum;                        // Firmware bin����У���
    INT32U  chksum;                         // Firmware binУ���
//	INT8U	trans_type;
    void (* const call_begin)(struct _update_ctrl *);     // �����ص�
    void (* const call_transmit)(struct _update_ctrl *);  // ����ص�
    void (* const call_end)(struct _update_ctrl *);       // �����ص�
} update_ctrl_t;

typedef struct _system_update	{
	OS_EVENT           *mbox;
}_system_update_t;
extern _system_update_t app_system_update;


void    SystemUpdateTaskInit    (void);
void    run_application         (void);
void    FirmwareBinOpt          (update_ctrl_t *pBin, message_pkt_t *pmsg);
u8 		GetUpdateState(void);
#endif    /* APP_SYSTEM_UPDATE_H */

/*
*********************************************************************************************************
*                                                No More!
*********************************************************************************************************
*/
