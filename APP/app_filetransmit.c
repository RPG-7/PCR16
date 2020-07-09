#include "app_filetransmit.h"
#include "app_usart.h"
#include "rw_spiflash.h"

__align(4) OS_STK  TASK_FILETRANSMIT_STK[STK_SIZE_FILETRANSMIT]; //任务堆栈声?
u8 transmit_buff[TransmitBufMaxSize];

app_filetransmit_t	app_filetransmit;
static  message_pkt_t    msg_pkt_filetrans;

static void TaskFileTransmit(void * ppdata);
static void upload_start(transmit_ctrl_t *ptfile);
static void upload_transmit(transmit_ctrl_t *ptfile);
static void upload_end(transmit_ctrl_t *ptfile);

transmit_ctrl_t   transmit_ctrl = {
    .state         = DEF_Idle,
    .ack           = ACK_OK,
    .trans_is_ok  = DEF_No,
    .trans_cnt      = 0,
    .buf_addr      = transmit_buff,
    .filepath     = {0},
    .size_max      = TransmitBufMaxSize,
    .size_total    = 0,
    .size_cnt      = 0,
    .frame_cnt     = 0,
	.frame_total = 0,
    .calcsum       = 0,
    .chksum        = 0,
    .call_begin    = upload_start,
    .call_transmit = upload_transmit,
    .call_end      = upload_end
};

void AppFileTransmitInit(void)
{
	OSTaskCreate(TaskFileTransmit,  (void * )0, (OS_STK *)&TASK_FILETRANSMIT_STK[STK_SIZE_FILETRANSMIT-1], TASK_PRIO_FILETRANSMIT);
}

static void DatInit(void)
{
	app_filetransmit.Mbox                = OSMboxCreate((void *)0);
}
//文件上传开始
static void upload_start(transmit_ctrl_t *ptfile)
{
	FRESULT res;
	
	ptfile->state = DEF_Busy;
    ptfile->trans_type = TransmitType_Unkown;
    ptfile->trans_is_ok = DEF_No;
	ptfile->trans_cnt = 0;
    ptfile->size_total  = 0;
    ptfile->size_cnt  = 6;                        // 计数器
    ptfile->calcsum   = 0;
    ptfile->chksum    = 0;                        //  校验和
    ptfile->frame_cnt = 0;                        // 帧计数器
	ptfile->frame_total = 0;
			
	res = f_open(flashfs.fil, (const char *)ptfile->filepath, FA_READ);
	if(res != FR_OK)	{//文件不存在 回复文件大小0
		memset(ptfile->buf_addr, 0, ptfile->size_cnt);
		ptfile->trans_start = DEF_No;
	}
	else	{
		ptfile->size_total = f_size(flashfs.fil);
		memcpy(ptfile->buf_addr, (u8 *)&ptfile->size_total, 4);//文件大小
		ptfile->frame_total = ptfile->size_total/TransmitBufMaxSize;
		if((ptfile->size_total%TransmitBufMaxSize)>0)	ptfile->frame_total += 1;//文件分几个序列传输
		memcpy(ptfile->buf_addr+4, (u8 *)&ptfile->frame_total, 2);
		ptfile->trans_start = DEF_Yes;
		f_close(flashfs.fil);
	}
    ptfile->ack = ACK_NONE;               // 烧写ACK  
	ptfile->state = DEF_Idle;	
}
//文件上传
static void upload_transmit(transmit_ctrl_t *ptfile)
{
	INT8U *pbuf;
	u32 i;
	INT32U  chksum = 0;
	
	ptfile->state = DEF_Busy;
	if(ptfile->frame_cnt >= ptfile->frame_total)
		ptfile->call_end(ptfile);
	else	{
		ptfile->ack = ACK_NONE;
		memcpy(ptfile->buf_addr, (u8 *)&ptfile->frame_cnt, 2);//上传帧序列号
		pbuf = ptfile->buf_addr+2;
		f_open(flashfs.fil, (const char *)ptfile->filepath, FA_READ);
		f_read(flashfs.fil, pbuf, TransmitBufMaxSize, &ptfile->size_cnt);
		f_close(flashfs.fil);
		for(i=0;i<ptfile->size_cnt;i++)	{//计算校验和
			 chksum += pbuf[i];
		}
		ptfile->calcsum = chksum;
		ptfile->frame_cnt ++;
		ptfile->trans_cnt += ptfile->size_cnt;
		ptfile->size_cnt += 2;
	}
	ptfile->state = DEF_Idle;	
}
//文件上传结束
static void upload_end(transmit_ctrl_t *ptfile)
{
	ptfile->state = DEF_Busy;
		
	if(ptfile->trans_cnt == ptfile->size_total&&ptfile->calcsum == ptfile->chksum)	{//传输文件大小和校验和ok 传输完成
		ptfile->ack = ACK_OK;
		ptfile->trans_is_ok = DEF_Yes;
	}else	{
		ptfile->ack = ACK_Error;
	}
	ptfile->state = DEF_Idle;
	ptfile->trans_start = DEF_No;
}
//上位机读取文件操作
void UploadFileOpt(transmit_ctrl_t *ptfile, message_pkt_t *pmsg)
{
    INT8U  *pdata;
//    INT32U  len;
    BIT32 tmp;

    pdata = (INT8U *)pmsg->Data;
    switch (pdata[0])
    {
        case TransmitCmd_Start://传输启动
            if (ptfile->state == DEF_Idle) {
				if(strlen((char *)&pdata[1]) >= TransmitFilePathSize)	{
					ptfile->ack = ACK_Error;
					break;
				}
				strcpy(ptfile->filepath, (char *)&pdata[1]);//上传文件路径
				ptfile->call_begin    = &upload_start;
				ptfile->call_transmit = &upload_transmit;
				ptfile->call_end      = &upload_end;
                ptfile->call_begin(ptfile);
            }
            break;
        case TransmitCmd_Transmit://请求传输
            if (ptfile->trans_start == DEF_Yes) {          
                ptfile->call_transmit(ptfile);    
			}				
            else    {
                ptfile->ack = ACK_Error;
                ptfile->trans_start = DEF_No;
            }
            break;
        case TransmitCmd_End://传输结束
            if (ptfile->trans_start == DEF_Yes) {
                tmp.ubyte[0] = pdata[1];
                tmp.ubyte[1] = pdata[2];
                tmp.ubyte[2] = pdata[3];
                tmp.ubyte[3] = pdata[4];
                ptfile->chksum = tmp.uword;
                ptfile->call_end(ptfile);
            }else    {
                ptfile->ack = ACK_Error;
                ptfile->trans_start = DEF_No;
            }
            break;
        case TransmitCmd_Query:
			if(ptfile->state == DEF_Busy)	{
				ptfile->ack = ACK_BUSY;				
			} else	{
				ptfile->ack = ACK_OK;		
			}
			break;
        default:
            break;
    }
	msg_pkt_filetrans.Src = USART_MSG_RX_TASK;
	msg_pkt_filetrans.Cmd = _CMD_FILETRANSMIT_UPLOAD;
	if(ptfile->ack == ACK_NONE)	{
		msg_pkt_filetrans.dLen = ptfile->size_cnt;
		msg_pkt_filetrans.Data    = ptfile->buf_addr;
		OSMboxPost(usart.mbox, &msg_pkt_filetrans);
	}
	else	{
		UsartSendAck(&msg_pkt_filetrans, ptfile->ack);
	}
}
//文件下载开始
static void download_start(transmit_ctrl_t *ptfile)
{
	FRESULT res;
	char filepath[FILE_NAME_LEN];
	char *buf[2];
	u16 num;
	
	ptfile->state = DEF_Busy;
    ptfile->trans_is_ok = DEF_No;
	ptfile->trans_cnt = 0;
    ptfile->size_cnt  = 0;                        // 计数器
    ptfile->calcsum   = 0;
    ptfile->chksum    = 0;                        //  校验和
    ptfile->frame_cnt = 0;                        // 帧计数器
	ptfile->trans_start = DEF_No;
			
	split(ptfile->filepath, "/", buf, &num);
	if(ptfile->trans_type == TransmitType_File)	{//下发文件类型为实验文件
		if(num > 0)	{//创建目录 上位机创建的实验信息保存路径：./Tmp/Lab
			sprintf(filepath, "%s%s/%s", USERPath, ServerLabFolderName, buf[0]);
			f_mkdir((const char *)filepath);
		}
		else	{
			ptfile->ack = ACK_Error;    
			ptfile->state = DEF_Idle;	
			return;
		}
	}
	sprintf(filepath, "%s%s/%s/%s", USERPath, TmpFolderName, buf[0], buf[1]);
	res = f_open(flashfs.fil, (const char *)filepath, FA_CREATE_ALWAYS | FA_WRITE);
	if(res == FR_OK || res == FR_EXIST)	{				 
		ptfile->trans_start = DEF_Yes;
		ptfile->ack = ACK_OK;
		strcpy(ptfile->filepath, (const char *)filepath);
		f_close(flashfs.fil);
	}
	else	{
		ptfile->ack = ACK_Error;   
	}
	ptfile->state = DEF_Idle;	
}
//文件下载
//static void download_transmit(transmit_ctrl_t *ptfile)
//{
//	INT8U *pbuf;
//	u32 i;
//	INT32U  chksum = 0;
//	
//	ptfile->state = DEF_Busy;
//	f_write(flashfs.fil, LogInfor.pbuf, ptfile->trans_cnt, &len);
//	ptfile->state = DEF_Idle;
//}
//文件下载结束
static void download_end(transmit_ctrl_t *ptfile)
{
	ptfile->state = DEF_Busy;
		
	if(ptfile->trans_cnt == ptfile->size_total&&ptfile->calcsum == ptfile->chksum)	{//传输文件大小和校验和ok 传输完成
		ptfile->ack = ACK_OK;
	}else	{
		ptfile->ack = ACK_Error;
	}
	ptfile->state = DEF_Idle;
	ptfile->trans_start = DEF_No;
	ptfile->trans_type = TransmitType_Unkown;
}

void DownloadFileOpt(transmit_ctrl_t *ptfile, message_pkt_t *pmsg)
{
    INT8U  *pdata;
    INT32U  len;
    BIT32 tmp;

    pdata = (INT8U *)pmsg->Data;
    switch (pdata[0])
    {
        case TransmitCmd_Start:
            if (ptfile->state == DEF_Idle) {
				ptfile->trans_type = pdata[1];//传输文件类型
				tmp.ubyte[0] = pdata[1];
                tmp.ubyte[1] = pdata[2];
                tmp.ubyte[2] = pdata[3];
                tmp.ubyte[3] = pdata[4];
                ptfile->size_total = tmp.uword;//文件大小				
				if(ptfile->size_total>0&&ptfile->size_total<TransmitFileMaxSize&&strlen((char *)&pdata[5])<TransmitFilePathSize)
				{//下发文件大小和文件名长度检查，文件小于100kb 文件名长度小于32byte
					strcpy(ptfile->filepath, (char *)&pdata[5]);//保存文件路径
					ptfile->call_begin    = &download_start;
//					ptfile->call_transmit = &download_transmit;
					ptfile->call_end      = &download_end;
					ptfile->call_begin(ptfile);
				}
				else
					ptfile->ack = ACK_Error;
            }
            break;
        case TransmitCmd_Transmit:
            if (ptfile->trans_start == DEF_Yes) {
				u32 i;
				ptfile->state = DEF_Busy;
				ptfile->size_cnt = pmsg->dLen - 1;
				for(i=0;i<ptfile->size_cnt;i++)	{
					ptfile->buf_addr[i] = pdata[i+1];
					ptfile->calcsum += ptfile->buf_addr[i];
				}
				f_open(flashfs.fil, (const char *)ptfile->filepath, FA_OPEN_APPEND | FA_WRITE);
				f_write(flashfs.fil, ptfile->buf_addr, ptfile->size_cnt, &len);
				f_close(flashfs.fil);
				ptfile->trans_cnt += ptfile->size_cnt;				
				ptfile->state = DEF_Idle;
//              ptfile->call_transmit(ptfile);    
			}
            else    {
                ptfile->ack = ACK_Error;
                ptfile->trans_start = DEF_No;
            }
            break;
        case TransmitCmd_End:
            if (ptfile->trans_start == DEF_Yes) {
                tmp.ubyte[0] = pdata[1];
                tmp.ubyte[1] = pdata[2];
                tmp.ubyte[2] = pdata[3];
                tmp.ubyte[3] = pdata[4];
                ptfile->chksum = tmp.uword;
                ptfile->call_end(ptfile);
            }else    {
                ptfile->ack = ACK_Error;
                ptfile->trans_start = DEF_No;
            }
            break;
        case TransmitCmd_Query:
			if(ptfile->state == DEF_Busy)	{
				ptfile->ack = ACK_BUSY;				
			} else	{
				ptfile->ack = ACK_OK;		
			}
			break;
        default:
            break;
    }
	msg_pkt_filetrans.Src = USART_MSG_RX_TASK;
	msg_pkt_filetrans.Cmd = _CMD_FILETRANSMIT_DOWNLOAD;
	UsartSendAck(&msg_pkt_filetrans, ptfile->ack);
}

u8 GetTransmitState(void)
{
	return transmit_ctrl.state;
}

static void TaskFileTransmit(void * ppdata)
{
	u8 err;
	message_pkt_t *msg;
	ppdata = ppdata;
	
	DatInit();
	
	for(;;)
	{
		msg = (message_pkt_t *)OSMboxPend(app_filetransmit.Mbox, 0, &err);
		if(msg->Src == MSG_FILETRANSMIT_UPLOAD)	{//文件上传操作
			UploadFileOpt(&transmit_ctrl, msg);
		}
		else if(msg->Src == MSG_FILETRANSMIT_DOWNLOAD)	{//文件下载操作
			DownloadFileOpt(&transmit_ctrl, msg);
		}
		else if(msg->Src == MSG_JUMP_IAP)	{
			 INT8U  *pdata = (INT8U *)msg->Data;
			if(pdata[0] == TYPE_JumpToIap)
				FWUpdate_reboot();//进入IAP 升级固件
		}
	}
}

