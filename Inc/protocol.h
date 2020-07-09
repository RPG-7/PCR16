#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__


#define BUFF_SIZE                      100u

//#define MSG_SYSTEM_ACK_CMD              0x78    // ϵͳӦ������
//#define MSG_ABNORMAL_CMD                0x79    // ��Ϣ��������
#define  PROTOCOL_RX_SD0        0x7D           /* Start delimiters                                */
#define  PROTOCOL_RX_SD1        0x7D
#define  PROTOCOL_RX_END        0x0D           /* End   delimiter                                 */
                                                     /* Outbound packets (to NIOS-II)                   */
#define  PROTOCOL_TX_SD0        0x7D           /* Start delimiters                                */
#define  PROTOCOL_TX_SD1        0x7D
#define  PROTOCOL_TX_END        0x0D           /* End   delimiter                                 */

#define	 PRO_CMD_POSITION     6
#define	 PRO_LENGTH_SIZE     4                                               ///< ��Ϣ��Length��ռ�ֽ���
#define  PRO_START_LEN       5                                               ///< 4�ֽڳ��� + 1�ֽ�����
#define  PRO_EXTENT_LEN      4                                               ///< 1�ֽ����� + 2�ֽ�У��� + 1�ֽڽ�����
#define  PRO_END_LEN         3                                               ///< 2�ֽ�У��� + 1�ֽڽ�����

#define  PRO_RX_STATE_SD0              0           /* waiting for start first  start delimiter (SD0)  */
#define  PRO_RX_STATE_SD1              1           /* waiting for start second start delimiter (SD1)  */
#define  PRO_RX_STATE_LEN0             2           /* waiting for len0  byte                       */
#define  PRO_RX_STATE_LEN1             3           /* waiting for len1  byte                      */
//#define  PRO_RX_STATE_LEN2             4           /* waiting for len2  byte                       */
//#define  PRO_RX_STATE_LEN3             5           /* waiting for len3  byte                      */
//#define  PRO_RX_STATE_CMD             4
#define  PRO_RX_STATE_DATA             6           /* waiting for data                                */
#define  PRO_RX_STATE_CHKSUM0          7           /* waiting for checksum0 low byte                  */
#define  PRO_RX_STATE_CHKSUM1          8           /* waiting for checksum1 high byte                 */
#define  PRO_RX_STATE_END              9           /* waiting for end delimiter                       */

#define  PRO_TX_STATE_SD0              0           /* Transmit state machine states                   */
#define  PRO_TX_STATE_SD1              1
#define  PRO_TX_STATE_LEN0             2
#define  PRO_TX_STATE_LEN1             3
#define  PRO_TX_STATE_LEN2             4
#define  PRO_TX_STATE_LEN3             5
#define  PRO_TX_STATE_DATA             6
#define  PRO_TX_STATE_CHKSUM0          7
#define  PRO_TX_STATE_CHKSUM1          8
#define  PRO_TX_STATE_END              9

#define  _CMD_READ          1
#define  _CMD_WRITE         2

typedef enum { 
    _CMD_RW_SYS_INFOR			=	0X01,//�޸�ϵͳ������Ϣ
    _CMD_EXECUTE_SYS_INFOR			=	0X02,//ִ��ϵͳ������Ϣ�޸�
	_CMD_READ_DevState		=	0X03,//��ȡ�豸����״̬
	_CMD_READ_RunningLabName		=	0X04,//��ȡ��ǰʵ������
	_CMD_READ_RunningLabData		=	0X05,//��ȡ��ǰʵ������
	_CMD_READ_SysError		=	0X06,//��ȡ����
	_CMD_SET_LabState		=	0X07,//����ʵ����ͣ
	_CMD_CALIBRATE = 0X08,//У׼
	_CMD_READ_CalibrateRes = 0X09,//��ȡУ׼���
	_CMD_FILETRANSMIT_DOWNLOAD	= 0X0A,//�����ļ�
	_CMD_FILETRANSMIT_UPLOAD	= 0X0B,	//��ȡ�ļ�
	_CMD_UPDATE_FW	= 0X0C,	//�̼�����
	_CMD_LED_CTRL	= 0X0D,//LED on/off control
	_CMD_RESET_MOTOR	= 0X0E,//�����λ
	_CMD_DBG_MoveAnyPosAtReset = 0x0F,//�ƶ����
	_CMD_GetMotorStatus	= 0x10,//��ѯ���״̬
	_CMD_GetMotorPositon	= 0x11,//��ȡ���λ��
	_CMD_ReadFlashSpace	= 0x12,//��ȡ�洢��Ϣ
	_CMD_SetTemp	= 0x13,//����ģ���¶�
	_CMD_GetTemp	= 0x14,//��ȡģ���¶�
	_CMD_GetFluo	= 0x15,//��ȡӫ������
	_CMD_SetPIDVal	= 0xE0,//PID��������
	_CMD_ACK	= 0XFA,//ack cmd
} EMessageCmd;

enum MSG_ERR {
    MSG_ERR_NONE              =  0x0000u, // ����Ϣ����
    MSG_ERR_SLAVE_NO_ACK      =  0xFFF8u, // �Ӱ�����Ӧ
	MSG_ERR_LENGTH      		 =  0xFFF9u, // ��Ϣ���ȴ������
	MSG_ERR_CHECKSUM          =  0xFFFAu, // ��ϢУ��ʹ������
	MSG_ERR_ETX_WORD          =  0xFFFBu, // ��Ϣ�������������
	MSG_ERR_UNDEFINITION      =  0xFFFCu, // ��Ϣ����δ����������
	MSG_ERR_IAP_MODE          =  0xFFFDu, // ��ǰ����IAPģʽ
	MSG_ERR_STATE             =  0xFFFEu, // ��Ϣδ����״̬
	MSG_ERR_TIMEOUT		     =  0xFFF7u, // ��Ϣ���ճ�ʱ
};

enum ACK_MSG	{
	ACK_Fail=0,
	ACK_OK=1,
	ACK_Error=2,
	ACK_BUSY=3,
	ACK_NONE=0XFF,
};

void GetProductInfor(void);
#endif
