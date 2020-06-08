#ifndef __SYS_INFO_H__
#define __SYS_INFO_H__

#include "includes.h"

#define SYSINFO_SIZE          30

/*
********************************************************************************
* Pre-processor Definitions
********************************************************************************
*/
#define CONFIG_SYSINFO_Name                            "PCR16"
#define CONFIG_SYSINFO_Type                            "xxxxxx"
#define CONFIG_SYSINFO_SN                              "281090312893"
#define CONFIG_SYSINFO_BoardSN                  		"********"
#define CONFIG_SYSINFO_HW_Version   				   	"V0.1"
#define CONFIG_SYSINFO_BOARD_Version                  	"V0.1"
#define CONFIG_SYSINFO_FW_Version   				   	"V0.0.3M"
#define CONFIG_SYSINFO_BuildDate                      	__DATE__
#define CONFIG_SYSINFO_BuildTime                     	__TIME__
#define CONFIG_SYSINFO_Manufacturer                    "TechWay, Inc."

#define SYS_FW_Type                            "Debug"

enum IG_INDEX {
	IDX_PRODUCT_NAME            	=   0x00,   // ��������
    IDX_PRODUCT_TYPE,   // �����ͺ�
    IDX_PRODUCT_SN,   // �������к�
	IDX_MAINBOARD_FWVer,//����̼��汾
    IDX_MAINBOARD_PCBVer,   // ����PCB�汾
    IDX_MAINBOARD_PCBAVer,   // ����PCBA�汾
    IDX_MAINBOARD_SN ,   // �������к�
    IDX_SENSORBOARD_FWVer,   // ��������̼��汾
    IDX_SENSORBOARD_PCBVer,//��������PCB�汾
    IDX_SENSORBOARD_PCBAVer ,//��������PCBA�汾
    IDX_SENSORBOARD_SN ,//�����������к�
};
/*
enum PassWard_Type	{
	ChangeProductorType  =0x01,//�޸Ĳ�Ʒ�ͺ�
	ChangeProductorSN   =	0x02,//�޸Ĳ�Ʒ���к�
	ChangeMainbordPCBVer =0x03,//�޸�����PCB�汾��
	ChangeMainbordPCBAVer =	0x04,//�޸�����PCBA�汾��
	ChangeProductorDate 	= 0x05,//�޸Ĳ�Ʒ����
	ChangeSlavebordPCBVer  = 0x06,//�޸ĴӰ�PCB�汾��
	ChangeSlavebordPCBAVer =0x07,//�޸ĴӰ�PCBA�汾��
	ChangePlateType = 0x08,   //�޸�plate��������
};
*/
typedef struct _ident_infor {
    char    type[SYSINFO_SIZE];
    char    product_sn[SYSINFO_SIZE];
    char    pcbver[SYSINFO_SIZE];
    char    pcbaver[SYSINFO_SIZE];
    char    board_sn[SYSINFO_SIZE];
	//u16 	crc;
} ident_infor_t;

typedef struct _sensorboard_infor {
    char    fwver[SYSINFO_SIZE];   
    char    pcbver[SYSINFO_SIZE];
    char    pcbaver[SYSINFO_SIZE];
	char    sn[SYSINFO_SIZE];
} sensorboard_infor_t;

typedef struct _sys_infor {
    char        *pname;
    char        *pfwver;
	char 		*pfwtype;
    char        *pbuilddate;
    char        *pbuildtime;
	char 		*pmanufacturer;
    ident_infor_t *pident;
	sensorboard_infor_t	*psensorinfor;
} sys_infor_t;

//extern sensorboard_infor_t sensor_infor;
//extern ident_infor_t g_ident_infor;
extern const sys_infor_t g_sys_infor;
extern const u8 SYSINFOR_PASSWORD[6];

void show_sys_info(void);
INT8U SystemInfoRead(INT8U idxInfo, char *pbuf);
INT8U SystemInfoWrite(INT8U idxInfo, char *pbuf);
#endif
