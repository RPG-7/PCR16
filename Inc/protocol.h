#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__


#define BUFF_SIZE                      100u

//#define MSG_SYSTEM_ACK_CMD              0x78    // 系统应答命令
//#define MSG_ABNORMAL_CMD                0x79    // 消息错误命令
#define  PROTOCOL_RX_SD0        0x7D           /* Start delimiters                                */
#define  PROTOCOL_RX_SD1        0x7D
#define  PROTOCOL_RX_END        0x0D           /* End   delimiter                                 */
                                                     /* Outbound packets (to NIOS-II)                   */
#define  PROTOCOL_TX_SD0        0x7D           /* Start delimiters                                */
#define  PROTOCOL_TX_SD1        0x7D
#define  PROTOCOL_TX_END        0x0D           /* End   delimiter                                 */

#define	 PRO_CMD_POSITION     6
#define	 PRO_LENGTH_SIZE     4                                               ///< 消息中Length所占字节数
#define  PRO_START_LEN       5                                               ///< 4字节长度 + 1字节命令
#define  PRO_EXTENT_LEN      4                                               ///< 1字节命令 + 2字节校验和 + 1字节结束符
#define  PRO_END_LEN         3                                               ///< 2字节校验和 + 1字节结束符

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
    _CMD_RW_SYS_INFOR			=	0X01,//修改系统配置信息
    _CMD_EXECUTE_SYS_INFOR			=	0X02,//执行系统配置信息修改
	_CMD_READ_DevState		=	0X03,//读取设备运行状态
	_CMD_READ_RunningLabName		=	0X04,//读取当前实验名称
	_CMD_READ_RunningLabData		=	0X05,//读取当前实验数据
	_CMD_READ_SysError		=	0X06,//读取故障
	_CMD_SET_LabState		=	0X07,//设置实验启停
	_CMD_CALIBRATE = 0X08,//校准
	_CMD_READ_CalibrateRes = 0X09,//读取校准结果
	_CMD_FILETRANSMIT_DOWNLOAD	= 0X0A,//下载文件
	_CMD_FILETRANSMIT_UPLOAD	= 0X0B,	//读取文件
	_CMD_UPDATE_FW	= 0X0C,	//固件升级
	_CMD_LED_CTRL	= 0X0D,//LED on/off control
	_CMD_RESET_MOTOR	= 0X0E,//电机复位
	_CMD_DBG_MoveAnyPosAtReset = 0x0F,//移动电机
	_CMD_GetMotorStatus	= 0x10,//查询电机状态
	_CMD_GetMotorPositon	= 0x11,//获取电机位置
	_CMD_ReadFlashSpace	= 0x12,//获取存储信息
	_CMD_SetTemp	= 0x13,//设置模块温度
	_CMD_GetTemp	= 0x14,//读取模块温度
	_CMD_GetFluo	= 0x15,//获取荧光数据
	_CMD_SetPIDVal	= 0xE0,//PID参数设置
	_CMD_ACK	= 0XFA,//ack cmd
} EMessageCmd;

enum MSG_ERR {
    MSG_ERR_NONE              =  0x0000u, // 无消息错误
    MSG_ERR_SLAVE_NO_ACK      =  0xFFF8u, // 从板无响应
	MSG_ERR_LENGTH      		 =  0xFFF9u, // 消息长度错误代码
	MSG_ERR_CHECKSUM          =  0xFFFAu, // 消息校验和错误代码
	MSG_ERR_ETX_WORD          =  0xFFFBu, // 消息结束符错误代码
	MSG_ERR_UNDEFINITION      =  0xFFFCu, // 消息命令未定义错误代码
	MSG_ERR_IAP_MODE          =  0xFFFDu, // 当前处于IAP模式
	MSG_ERR_STATE             =  0xFFFEu, // 消息未定义状态
	MSG_ERR_TIMEOUT		     =  0xFFF7u, // 消息接收超时
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
