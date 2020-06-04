#ifndef __APP_FILETRANSMIT_H__
#define __APP_FILETRANSMIT_H__

#include "includes.h"

#define	TransmitBufMaxSize 	1024
#define	TransmitFileMaxSize 	102400//100kb
#define	TransmitFilePathSize 	32

typedef struct _app_filetransmit_t {
    OS_EVENT            *Mbox;
}app_filetransmit_t;

typedef enum {
    TransmitCmd_Query     = 0x01,     // ��ѯ״̬
    TransmitCmd_Start     = 0x02,     // ��������
    TransmitCmd_Transmit  = 0x03,     // ����bin�ļ�
    TransmitCmd_End       = 0x04,     // �������
} Transmit_cmd_t;

typedef enum {
	TransmitType_Unkown = 0,
	TransmitType_FW = 1,
	TransmitType_File = 2,
}TransmitType;

typedef enum {
	TYPE_JumpToIap = 0X05,////jump to IAP
	TYPE_JumpToApp = 0x06,//jump to APP
} UpdateType;


typedef struct _transmit_ctrl {
    INT8U   state;                          // transmit״̬
    INT8U   ack;                            // transmit ACK
    INT8U   trans_start;                     // transmit����
    INT8U   trans_type;                      // transmit����
    INT8U   trans_is_ok;                   // transmit�Ƿ�ɹ�
    INT16U  trans_cnt;                       // һ���շ�������
    INT8U  *const buf_addr;                 // transmit������ʼ��ַ
    char  filepath[TransmitFilePathSize];                      // transmit �ļ�����·��
	FILE	*fp;
    INT32U const size_max;                  // �ɴ洢�����
    INT32U  size_total;                     // �ļ��ܴ�С
    INT32U  size_cnt;                       // ��֡�ֽ���
    INT32U  frame_cnt;                      // transmit֡������
	INT32U  frame_total;
    INT32U  calcsum;                        // transmit����У���
    INT32U  chksum;                         // У���
    void (* call_begin)(struct _transmit_ctrl *);     // �����ص�
    void (* call_transmit)(struct _transmit_ctrl *);  // ����ص�
    void (* call_end)(struct _transmit_ctrl *);       // �����ص�
} transmit_ctrl_t;

extern app_filetransmit_t	app_filetransmit;
void AppFileTransmitInit(void);
u8 GetTransmitState(void);
#endif

