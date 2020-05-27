#ifndef __APP_FILETRANSMIT_H__
#define __APP_FILETRANSMIT_H__

#include "includes.h"

#define	TransmitMaxBufSize 	1024

typedef struct _app_filetransmit_t {
    OS_EVENT            *Mbox;
}app_filetransmit_t;

typedef enum {
    TransmitCmd_Query     = 0x01,     // ��ѯ״̬
    TransmitCmd_Start     = 0x02,     // ��������
    TransmitCmd_Transmit  = 0x03,     // ����bin�ļ�
    TransmitCmd_End       = 0x04,     // �������
} Transmit_cmd_t;

typedef struct _transmit_ctrl {
    INT8U   state;                          // transmit״̬
    INT8U   ack;                            // transmit ACK
    INT8U   trans_start;                     // transmit����
    INT8U   trans_last;                      // transmit����
    INT8U   trans_is_ok;                   // transmit�Ƿ�ɹ�
    INT16U  trans_cnt;                       // һ���շ�������
    INT8U  *const buf_addr;                 // transmit������ʼ��ַ
    char  filepath[64];                      // transmit �ļ�����·��
	FILE	*fp;
    INT32U const size_max;                  // �ɴ洢�����
    INT32U  size_total;                     // �ļ��ܴ�С
    INT32U  size_cnt;                       // ��֡�ֽ���
    INT32U  frame_cnt;                      // transmit֡������
    INT32U  calcsum;                        // transmit����У���
    INT32U  chksum;                         // У���
    void (* call_begin)(struct _transmit_ctrl *);     // �����ص�
    void (* call_transmit)(struct _transmit_ctrl *);  // ����ص�
    void (* call_end)(struct _transmit_ctrl *);       // �����ص�
} transmit_ctrl_t;

void AppFileTransmitInit(void);
void UploadFileOpt(transmit_ctrl_t *ptfile, message_pkt_t *pmsg);
#endif

