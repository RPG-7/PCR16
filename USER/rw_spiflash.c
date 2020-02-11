#include "rw_spiflash.h"
#include "tlsf.h"

#define FORMAT_DISK			0

struct _flashfs	{
	DIR dir;
	FATFS fs;
	FIL	fil;
} flashfs;

u8 GetFlashSpace(u32 *ptotal, u32 *pfree)
{
	FATFS *fs;
	FRESULT res;
    DWORD fre_clust, fre_sect, tot_sect;

    /* Get volume information and free clusters of drive 1 */
    res = f_getfree(USERPath, &fre_clust, &fs);
    if (res!=FR_OK) return 0;

    /* Get total sectors and free sectors */
    tot_sect = (fs->n_fatent - 2) * fs->csize;
    fre_sect = fre_clust * fs->csize;

    /* Print the free space (assuming 4 Kbytes/sector) */
	*ptotal = (u32)(tot_sect * 4);
	*pfree = (u32)(fre_sect * 4);
    //BSP_PRINTF("%10lu KiB total drive space.\n%10lu KiB available.\n", tot_sect, fre_sect);	

	return 1;
}

int FlashFSInit(void)
{
	FRESULT res;
	
	u32 disk_tot, disk_free;
#if FORMAT_DISK == 0	
	res = f_mount(&flashfs.fs, USERPath, 1);
	if(res == FR_OK)	{
		DIR dir;
		FILINFO fn;
		BSP_PRINTF("mount flash ok");
		res = f_opendir(&dir,USERPath);
		if(res==FR_OK)	{
			for(;;)	{
				res=f_readdir(&dir,&fn);
				if(res!=FR_OK||fn.fname[0]==0)	break;
				BSP_PRINTF(" %s",fn.fname);
			}
			f_closedir(&dir);
		}
	}else if(res == FR_NO_FILESYSTEM)	{
#endif
		u8 *work;		
		work = (u8 *)tlsf_malloc(UserMem,_MAX_SS);
		/* Create FAT volume */
		BSP_PRINTF("mount flash failed, format flash");
		BSP_W25Qx_Erase_Chip();//�ȸ�ʽ��
		//OSTimeDly(5000);
		res = f_mkfs(USERPath, FM_ANY, 0, work, _MAX_SS);
		tlsf_free(UserMem,work);
		work = NULL;
		if(res == FR_OK)	{
			BSP_PRINTF("format flash ok");
		}else	{
			BSP_PRINTF("format flash failed");
			return res;
		}
		res = f_mount(&flashfs.fs, USERPath, 1);
		if(res == FR_OK)			
			BSP_PRINTF("mount flash ok");
		else	
			return res;
#if FORMAT_DISK == 0
	}
#endif
	
//	GetPerfuseDataFilename(NULL);
	GetFlashSpace(&disk_tot, &disk_free);
	BSP_PRINTF("SPI Flash Space:");
	BSP_PRINTF("    %u KiB total drive space.\n    %u KiB available.\n", disk_tot, disk_free);
	return res;
}

//����ϵͳ�����ļ�
void CreateLogFile(void)
{
	FILINFO fn;
	FRESULT res;     /* FatFs return code */
	//char datbuf[20] = "Create file.\r\n";
	char filename[FILE_NAME_LEN];
	
	sprintf(filename, "%s%s", USERPath, LOG_FILE_NAME);
	res = f_stat(filename, &fn);
	if(res == FR_OK)	{//If exist, the function returns with FR_OK
		BSP_PRINTF("Time: %u/%02u/%02u, %02u:%02u\n",
               (fn.fdate >> 9)+1980, (fn.fdate >> 5) & 0x0f, fn.fdate & 0x1f,
               fn.ftime >> 11, (fn.ftime >> 5) & 0x3f);
		return;
	}	
	if(res == FR_NO_FILE)	
	{//�ļ������� or ǿ�ƴ�����־��Ч �����ļ�
		res = f_open(&flashfs.fil, filename, FA_CREATE_ALWAYS | FA_WRITE);//create new file and w mode
		if(res != FR_OK)	{
			return;
		}
		f_close(&flashfs.fil);
		write_log("Create file.");
	}
	//BSP_PRINTF("Create logfile ok");
}

//u32 fil_size,rsize,f_tell_pos;
//д��־
u8 write_log(char *str)
{
	FRESULT res;     /* FatFs return code */
	u8 len, log_len;
	char filename[FILE_NAME_LEN];
	char *pLogBufer;
	u32 fil_size,rsize;
//	RTC_TimeTypeDef sTime = {0};
//	RTC_DateTypeDef sDate = {0};
	
	pLogBufer = (char *)tlsf_malloc(UserMem, RLOG_BUFSIZE);//д�뵥��log���ܴ���WLOG_BUFSIZE
	//memset(pLogBufer,0,WLOG_BUFSIZE);
	sprintf(filename, "%s%s", USERPath, LOG_FILE_NAME);//log�ļ���
	res = f_open(&flashfs.fil, filename, FA_OPEN_APPEND | FA_WRITE| FA_READ);//create new file and rw mode
	if(res != FR_OK)
		goto _exit;
	fil_size = f_size(&flashfs.fil);//��ȡlog�ļ���С
	if(fil_size>LOG_FILE_MAXSIZE)	{//�ļ�����10k ɾ��������
		char data;
		fil_size = LOG_FILE_MAXSIZE - LOG_FILE_TRUNCATION_SIZE;
		f_lseek(&flashfs.fil, fil_size);//���ļ�ָ��ָ���ļ�ĩβ1k��λ��
		for(;;)		{//�ҵ�����λ�� Ŀ�����ڽ�ȡ�ļ�ʱ ������������
			f_read(&flashfs.fil, &data, 1, &rsize);
			if(rsize==0)	goto _exit;
			if(data == '\n')	{
				break;
			}
		}
		//f_tell_pos = f_tell(&flashfs.fil);
		f_read(&flashfs.fil, pLogBufer, LOG_FILE_TRUNCATION_SIZE, &rsize);//������Ҫ����������
		f_close(&flashfs.fil);
		f_open(&flashfs.fil, filename, FA_CREATE_ALWAYS | FA_WRITE);//���´����ļ�
		f_write(&flashfs.fil, pLogBufer, rsize, NULL);//����д�뱣��������
		//f_sync(&flashfs.fil);//ǿ��ˢ���ļ�
		f_lseek(&flashfs.fil, f_size(&flashfs.fil));//���ļ�ָ���Ƶ��ļ�β��
	}
//	bsp_rtc_get_time(&sTime, &sDate);
//	len = sprintf(pLogBufer,"*%02u/%02u/%02u %02u:%02u:%02u  ",sDate.Year,sDate.Month, sDate.Date, sTime.Hours,sTime.Minutes,sTime.Seconds);
	log_len = len + strlen(str)+2;
	if(log_len > WLOG_BUFSIZE)//������־����̫�� �˳�
		goto _exit;
	sprintf(pLogBufer + len, "%s\r\n", str);//�ӻ���
	f_write(&flashfs.fil, pLogBufer, log_len, NULL);	//д��log�ļ�
	BSP_PRINTF("Write Log OK");
_exit:
	f_close(&flashfs.fil);
	tlsf_free(UserMem, pLogBufer);
	pLogBufer = NULL;
	return 1;
}

//u32 logPosition,dissize;
//����־ ��ȡlog���һ��LOG_DISPLAY_SIZE
u32 read_log(char *pbuf)
{
	FRESULT res;     /* FatFs return code */
	char filename[FILE_NAME_LEN];
	UINT rsize,line;
	u32 logPosition,dissize;
	char data;
	
	rsize = 0;	
	sprintf(filename, "%s%s", USERPath, LOG_FILE_NAME);
	res = f_open(&flashfs.fil, filename, FA_READ);
	if(res != FR_OK)
		return 0;
	/*logPosition = f_size(&flashfs.fil);
	if(logPosition >= LOG_DISPLAY_SIZE)	{
		logPosition -= LOG_DISPLAY_SIZE;
	}
	f_lseek(&flashfs.fil, f_size(&flashfs.fil));//���ļ�ָ��ָ��ָ��λ��*/
	logPosition = 0;
	dissize = f_size(&flashfs.fil);
	if(dissize>=25)
		logPosition = dissize - 25;
	dissize = 0;
	line = 1;
	for(;;)		{//ֻ��ʾ���13��
		if(logPosition==0)	{
			break;
		}
		f_lseek(&flashfs.fil, logPosition--);
		f_read(&flashfs.fil, &data, 1, &rsize);
		if(rsize==0)	goto _exit;
		dissize += 1;
		if(dissize >= (LOG_DISPLAY_SIZE-25))	{//�ﵽ �����ʾ����
			break;
		}
		if(data == '\n')	{
			line++;
			if(line > LOG_DISPLAY_LINE)	{//�ﵽ�����ʾ���� 
				break;
			}
		}
	}
	res =  f_read(&flashfs.fil, pbuf, LOG_DISPLAY_SIZE, &rsize);//һ�δ洢100�ֽ�������������ԭĿ��С��һ�£������ڴ��ʱ����size	
	if(res != FR_OK) goto _exit;
_exit:
	f_close(&flashfs.fil);
	return rsize;
}









#if 0	//spi flash �����ļ�ϵͳ���Ժ���
FRESULT res;        /* API result code */
FATFS fs;           /* Filesystem object */
void fs_test()
{
    FIL fil;            /* File object */
    DWORD fre_clust;
    UINT bw;            /* Bytes written */
		FATFS *fls = &fs;
    BYTE work[4096]; /* Work area (larger is better for processing time) */
		char w_string[]={"Hello, World!\r\n"};
		char r_string[18];

		//res = f_getfree(USERPath,&fre_clust,&fls);         /* Get Number of Free Clusters */
		f_mkfs(USERPath, FM_ANY, 0, work, sizeof(work));
		res = f_mount(&fs, USERPath, 1);
		if(res == FR_OK)	{
			BSP_PRINTF("mount file ok");
		}else if(res == FR_NO_FILESYSTEM)	{
			/* Create FAT volume */
			res = f_mkfs(USERPath, FM_ANY, 0, work, sizeof(work));
			if(res == FR_OK)	{
				BSP_PRINTF("f_mkfs ok");
			}else	return;
			res = f_mount(&fs, USERPath, 1);
			if(res == FR_OK)			
				BSP_PRINTF("mount file ok");
			else	return;
		}

    /* Create a file as new */
    res = f_open(&fil, "1:/hello.txt", FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
		if(res == FR_OK)	{ 
			BSP_PRINTF("open file ok");
		}else	return;

    /* Write a message */
    res = f_write(&fil, w_string, 15, &bw);
    if (bw == 15) 	{
				BSP_PRINTF("write file ok");
		}else	return;
		f_lseek(&fil, 0);
		f_gets(r_string, sizeof(r_string), &fil);
		BSP_PRINTF("r usb:%s\r\n",r_string);
		
    /* Close the file */
    f_close(&fil);

    /* Unregister work area */
    //f_mount(NULL, USERPath, 1);
}
#endif

